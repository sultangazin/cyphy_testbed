#include "Eigen/Dense"
#include "math.h"
#include <chrono>

#include "cis_supervisor/cis_supervisor_ros.hpp"
#include "cis_supervisor/PerformanceMsg.h"

#include <testbed_msgs/ControlStamped.h>

#include "utilities/timeutils/timeutils.hpp"
#include <mutex>

void thread_fnc(void* p);
std::mutex mx;

// =================================================================
// CLASS
//
CISSupervisorROS::CISSupervisorROS():
	active_(false),
	setpoint_type_("stop"),
	last_state_time_(-1.0),
	initialized_(false) {}

	CISSupervisorROS::~CISSupervisorROS() {};

bool CISSupervisorROS::Initialize(const ros::NodeHandle& n) {
	node_ = n;
	ros::NodeHandle nl(n);

	// Set the node name
	node_name_ = ros::this_node::getName().c_str();

	// Load parameters
	if (!LoadParameters(nl)) {
		ROS_ERROR("%s: Failed to load parameters.", node_name_.c_str());
		return false;
	}

	// Register callbacks
	if (!RegisterCallbacks(n)) {
		ROS_ERROR("%s: Failed to register callbacks.", node_name_.c_str());
		return false;
	}

	// Instantiate classes
	supervisor_ = new CISSupervisor();
	supervisor_->SetK(ctrl_gains_);

	supervisor_->LoadModel();
	supervisor_->LoadCISs();

	// Setup output publications and services
	SetUpPublications(nl);

	initialized_ = true;

	thrust = 0.032 * 9.81;

	periodic_thread_arg_.period = 0.05;
	periodic_thread_arg_.psupervisor = supervisor_;
	periodic_thread_arg_.ctrl_pub = cis_supervisor_ctrl_;
	periodic_thread_arg_.perf_pub = performance_pub_; 
	//periodic_thread_ = std::thread(thread_fnc, (void*) &periodic_thread_arg_);

	ROS_INFO("[%s] Initialized!", node_name_.c_str());

	return true;
}

bool CISSupervisorROS::LoadParameters(const ros::NodeHandle& n) {

	ros::NodeHandle np("~");
	std::string key;

	// Controller name
	np.param<std::string>("param/controller_name", controller_name_, "CISSupervisor");

	np.param<std::string>("param/area_name", area_name_, "area0");

	// Vehicle name
	np.param<std::string>("param/vehicle_name", vehicle_name_, "cf2");

	// Vehicle State 
	np.param<std::string>("topics/in_state_topic", state_topic_,
			"/" + vehicle_name_ + "/external_codom");

	// Vehicle Setpoint
	np.param<std::string>("topics/in_setpoint_topic", setpoint_topic_,
			"/" + vehicle_name_ + "/setpoint");

	// Control Cmd 
	np.param<std::string>("topics/out_ctrl_topic", ctrls_topic_ ,
			"/" + area_name_ + "/controller/" + controller_name_ + "/" +
			vehicle_name_ + "/control");
	// Performance Topic
	np.param<std::string>("topics/out_perf_topic", performance_topic_,
			"/" + controller_name_ + "/" + vehicle_name_ + "/cis_perf");

	std::string param_name;
	if (np.searchParam("param/ctrl_gains", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		std::vector<double> p_vec;
		n.getParam(param_name, p_vec);
		std::copy_n(p_vec.begin(), p_vec.size(), ctrl_gains_.begin());

		std::cout << "K_gain: ";
		for (auto el : ctrl_gains_) {
			std::cout << el << " ";
		}
		std::cout << std::endl;
	} else {
		ROS_INFO("No param 'param/ctrl_gains' found in an upward search");
	}

	return true;
}

bool CISSupervisorROS::SetUpPublications(const ros::NodeHandle& n) {

	ros::NodeHandle nl(n);

	// Output Publications
	// Control Signals (Thrust + Angular Velocities)
	cis_supervisor_ctrl_ = nl.advertise<testbed_msgs::ControlStamped> (ctrls_topic_.c_str(), 5);

	// Eventual Performance Data
	performance_pub_ = nl.advertise<cis_supervisor::PerformanceMsg> (performance_topic_.c_str(), 5);

	// Advertise Services
	cis_supervisor_service = node_.advertiseService(
			"cis_supervisor", &CISSupervisorROS::cis_supervisor_tune, this);

	return true;
}


bool CISSupervisorROS::RegisterCallbacks(const ros::NodeHandle& n) {
	ros::NodeHandle nl(n);
	setpoint_sub_ = nl.subscribe(setpoint_topic_.c_str(), 1,
			&CISSupervisorROS::onNewSetpoint, this);
	state_sub_ = nl.subscribe(state_topic_.c_str(), 1,
			&CISSupervisorROS::onNewState, this);
	return true;
}


// CALLBACKS ----------------------------------------------------------------
void CISSupervisorROS::onNewState(
		const testbed_msgs::CustOdometryStamped::ConstPtr& msg) {
	// Take the time
	//ros::Time current_time = ros::Time::now();
	// Read the timestamp of the message
	//t.tv_sec = msg->header.stamp.sec;
	//t.tv_nsec = msg->header.stamp.nsec;
	XType state_;
	state_(0) = msg->p.x;
	state_(1) = msg->p.y;
	state_(2) = msg->p.z;
	state_(3) = msg->v.x;
	state_(4) = msg->v.y;
	state_(5) = msg->v.z;
	state_(6) = msg->a.x;
	state_(7) = msg->a.y;
	state_(8) = msg->a.z;

	supervisor_->SetState(state_);
	quat_.vec() = Eigen::Vector3d (msg->q.x, msg->q.y, msg->q.z);
	quat_.w() = msg->q.w;
	quat_.normalize();

	periodic_thread_arg_.quat = quat_;

	// In order to reduce the jitter among estimate update and execution of the 
	// Supervisor I could run the control step from this callback.
	// I need to improve this...
	double dt = msg->header.stamp.toSec() - last_state_time_;
	if (dt >= 0.10) {
		ros::Time ctrl_activation = ros::Time::now();
		UType control_cmd;
		UType u_body(UType::Zero()); 
		testbed_msgs::ControlStamped control_msg;
		cis_supervisor::PerformanceMsg ctrl_perf_msg;

		//std::cout << "Dt = " << dt << std::endl;
		last_state_time_ = ctrl_activation.toSec();
		if (supervisor_->isActive()) {
			supervisor_->Step(0.05);

			// Get the desired jerk
			control_cmd = supervisor_->getControls();
			// ... convert the jerk in autopilot commands
			// 1) Convert the jerk in body frame
			u_body = quat_.inverse() * control_cmd;
			// 2) Convert in angular velocity and thrust
			if (thrust > 0.05) {
				control_msg.control.roll = -(u_body(1) / thrust) * 0.032;
				control_msg.control.pitch = (u_body(0) / thrust) * 0.032;
			}
			thrust = std::max(thrust + 0.032 * u_body(2) * dt, 0.0);
			control_msg.control.thrust = thrust;
		} else {
			control_msg.control.thrust = 0.0;
			control_msg.control.roll = 0.0;
			control_msg.control.pitch = 0.0;
			control_msg.control.yaw_dot = 0.0;
		}

		// Performance Message
		ros::Time msg_timestamp = ros::Time::now();
		double PubPeriod = (msg_timestamp - last_sent_time).toSec();
		double SupExeTime = (msg_timestamp - ctrl_activation).toSec();
		last_sent_time = msg_timestamp;

		/*
		std::cout << PubPeriod << std::endl;
		std::cout << SupExeTime << std::endl;
		std::cout << std::endl;
		*/
		
		control_msg.header.stamp = msg_timestamp;
		ctrl_perf_msg.header.stamp = msg_timestamp;

		ctrl_perf_msg.thrust = thrust;
		for (int i = 0; i < 3; i++) {
			ctrl_perf_msg.jerk_body[i] = u_body(i);
		}
		ctrl_perf_msg.ang_velocity[0] = control_msg.control.roll;
		ctrl_perf_msg.ang_velocity[1] = control_msg.control.pitch;
		
		cis_supervisor_ctrl_.publish(control_msg);
		performance_pub_.publish(ctrl_perf_msg);
	}
	return;
}


// Process an incoming setpoint point change.
void CISSupervisorROS::onNewSetpoint(
		const testbed_msgs::ControlSetpoint::ConstPtr& msg) {

	XType ctrl_setpoint;

	if (msg->setpoint_type != "stop") {
		active_ = true;
		ctrl_setpoint(0) = msg->p.x;
		ctrl_setpoint(1) = msg->p.y;
		ctrl_setpoint(2) = msg->p.z;
		ctrl_setpoint(3) = msg->v.x;
		ctrl_setpoint(4) = msg->v.y;
		ctrl_setpoint(5) = msg->v.z;
		ctrl_setpoint(6) = msg->a.x;
		ctrl_setpoint(7) = msg->a.y;
		ctrl_setpoint(8) = msg->a.z;

		// Copy the setpoint structure into the controller class
		supervisor_->SetActive(true);
		supervisor_->SetSetpoint(ctrl_setpoint);

		// Set the flag about the setpoint
	} else {
		supervisor_->SetActive(false);
		active_ = false;
	}
}

bool CISSupervisorROS::cis_supervisor_tune(
		cis_supervisor::CISSupervisorTune::Request& req,
		cis_supervisor::CISSupervisorTune::Response& res) {
	std::cout << "[" << node_name_ << "]" << " Changing Settings. " << std::endl;

	return true;
}


void thread_fnc(void* p) {
	// Convert the pointer to pass information to the thread.
	Thread_arg* pArg = (Thread_arg*) p;

	double dt = pArg->period;
	CISSupervisor* psupervisor = pArg->psupervisor;
	ros::Publisher ctrl_pub = pArg->ctrl_pub;
	ros::Publisher perf_pub = pArg->perf_pub;

	struct timespec time;
	struct timespec next_activation;

	struct timespec period_tms; 
	create_tspec(period_tms, dt);

	UType control_cmd;
	testbed_msgs::ControlStamped control_msg;

	double thrust = 0.032 * 9.81;

	cis_supervisor::PerformanceMsg ctrl_perf_msg;

	ros::Time last_sent_time_local = ros::Time::now();
	while (ros::ok()) {
		// Get current time
		clock_gettime(CLOCK_MONOTONIC, &time);
		timespec_sum(time, period_tms, next_activation);

		UType u_body(UType::Zero()); 

		if (psupervisor->isActive()) {
			// Do something
			psupervisor->Step(dt);

			// Get the desired jerk
			control_cmd = psupervisor->getControls();

			// ... convert the jerk in autopilot commands
			// 1) Convert the jerk in body frame
			u_body = pArg->quat.inverse() * control_cmd;

			// 2) Convert in angular velocity and thrust
			if (thrust > 0.05) {
				control_msg.control.roll = -(u_body(1) / thrust) * 0.032;
				control_msg.control.pitch = (u_body(0) / thrust) * 0.032;
			}
			thrust = std::max(thrust + 0.032 * u_body(2) * dt, 0.0);
			control_msg.control.thrust = thrust;
		} else {
			control_msg.control.thrust = 0.0;
			control_msg.control.roll = 0.0;
			control_msg.control.pitch = 0.0;
			control_msg.control.yaw_dot = 0.0;
		}

		// Performance Message
		ctrl_perf_msg.thrust = thrust;
		for (int i = 0; i < 3; i++) {
			ctrl_perf_msg.jerk_body[i] = u_body(i);
		}
		ctrl_perf_msg.ang_velocity[0] = control_msg.control.roll;
		ctrl_perf_msg.ang_velocity[1] = control_msg.control.pitch;

		ros::Time msg_timestamp = ros::Time::now();
		double dt = (msg_timestamp - last_sent_time_local).toSec();
		//std::cout << dt << std::endl;

		control_msg.header.stamp = msg_timestamp;
		ctrl_perf_msg.header.stamp = msg_timestamp;
		ctrl_pub.publish(control_msg);
		perf_pub.publish(ctrl_perf_msg);

		// Sleep until next activation
		clock_nanosleep(CLOCK_MONOTONIC,
				TIMER_ABSTIME, &next_activation, NULL);
	}
	ROS_INFO("Terminating Thread...\n");
}

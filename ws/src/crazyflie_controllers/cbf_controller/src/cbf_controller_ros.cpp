#include "Eigen/Dense"
#include "math.h"
#include <chrono>

#include "cbf_controller/cbf_controller_ros.hpp"

//#include "utilities/timeutils/timeutils.hpp"
//#include "utilities/custom_conversion/custom_conversion.hpp"
#include <mutex>

void thread_fnc(void* p);
std::mutex mx;

using namespace Eigen;

// =================================================================
// CLASS
//
CBFControllerROS::CBFControllerROS():
	isControlActive_(false),
	setpoint_type_("stop"),
	last_state_time_(-1.0),
	initialized_(false) {}

CBFControllerROS::~CBFControllerROS() {};

bool CBFControllerROS::Initialize(const ros::NodeHandle& n) {
	node_ = n;
	ros::NodeHandle nl(n);

	// Set the node name
	node_name_ = ros::this_node::getName().c_str();

	// Instantiate class
	controller_ = new CBFController();

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
	
	// Setup output publications and services
	SetUpPublications(nl);

	initialized_ = true;

	//periodic_thread_arg_.period = 0.005;
	//periodic_thread_arg_.pcontroller = controller_;
	//periodic_thread_arg_.ctrl_pub = control_pub_;
	//periodic_thread_ = std::thread(thread_fnc, (void*) &periodic_thread_arg_);

	ROS_INFO("[%s] Initialized!", node_name_.c_str());

	return true;
}


bool CBFControllerROS::LoadParameters(const ros::NodeHandle& n) {

	ros::NodeHandle np("~");
	std::string key;

	// Controller and area names
	np.param<std::string>("controller_name", controller_name_, "CBFController");
	np.param<std::string>("area_name", area_name_, "area0");
	// Vehicle name
	np.param<std::string>("vehicle_name", vehicle_name_, "cf2");
	// Vehicle mass 
	np.param<double>("vehicle_mass", Mass_, 1.0);
	controller_->setVehicleMass(Mass_);


	// Vehicle State 
	np.param<std::string>("in_topics/state_topic", state_topic_,
			"/" + vehicle_name_ + "/external_codom");
	// Vehicle Setpoint
	np.param<std::string>("in_topics/setpoint_topic", setpoint_topic_,
			"/" + vehicle_name_ + "/setpoint");
	// Control Cmd 
	np.param<std::string>("out_topics/ctrl_topic", ctrls_topic_ ,
			"/" + area_name_ + "/controller/" + controller_name_ + "/" +
			vehicle_name_ + "/control");

	std::string param_name;
	if (np.searchParam("param/Kpos", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		n.getParam(param_name, Kpos_);
		controller_->setK("Kpos", Kpos_);
	} else {
		ROS_INFO("parameter not found");
	}
	if (np.searchParam("param/Kvel", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		n.getParam(param_name, Kvel_);
		controller_->setK("Kvel", Kvel_);
	} else {
		ROS_INFO("parameter not found");
	}
	if (np.searchParam("param/Kx", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		n.getParam(param_name, Kx_);
		controller_->setK("Kx", Kx_);
	} else {
		ROS_INFO("parameter not found");
	}
	if (np.searchParam("param/Kr", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		n.getParam(param_name, Kr_);
		controller_->setK("Kr", Kr_);
	} else {
		ROS_INFO("parameter not found");
	}
	if (np.searchParam("param/eta", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		n.getParam(param_name, eta_);
		controller_->setK("eta", eta_);
	} else {
		ROS_INFO("parameter not found");
	}
	if (np.searchParam("param/delta_x", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		n.getParam(param_name, delta_x_);
		controller_->setK("delta_x", delta_x_);
	} else {
		ROS_INFO("parameter not found");
	}

	if (np.searchParam("obst_param/gamma", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		double par;
		n.getParam(param_name, par);
		controller_->setK("gamma", par);
	} else {
		ROS_INFO("parameter not found");
	}

	if (np.searchParam("obst_param/beta", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		double par;
		n.getParam(param_name, par);
		controller_->setK("beta", par);
	} else {
		ROS_INFO("parameter not found");
	}

	if (np.searchParam("obst_param/b_safe", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		double par;
		n.getParam(param_name, par);
		controller_->setK("b_safe", par);
	} else {
		ROS_INFO("parameter not found");
	}
	return true;
}


bool CBFControllerROS::SetUpPublications(const ros::NodeHandle& n) {
	ros::NodeHandle nl(n);

	// Output Publications
	// Control Signals (Thrust + Angular Velocities)
	control_pub_ = nl.advertise<testbed_msgs::ControlStamped> (ctrls_topic_.c_str(), 5);

	return true;
}


bool CBFControllerROS::RegisterCallbacks(const ros::NodeHandle& n) {
	ros::NodeHandle nl(n);
	setpoint_sub_ = nl.subscribe(setpoint_topic_.c_str(), 1,
			&CBFControllerROS::onNewSetpoint, this);
	state_sub_ = nl.subscribe(state_topic_.c_str(), 1,
			&CBFControllerROS::onNewState, this);
	return true;
}


// CALLBACKS ----------------------------------------------------------------
void CBFControllerROS::onNewState(
		const testbed_msgs::CustOdometryStamped::ConstPtr& msg) {
	// Update the state of the system
	Vector3d pos(msg->p.x, msg->p.y, msg->p.z);
	Vector3d vel(msg->v.x, msg->v.y, msg->v.z);
	Vector3d acc(msg->a.x, msg->a.y, msg->a.z);
	controller_->setTranslationState(pos, vel, acc);

	Quaterniond quat(Quaterniond::Identity());
	quat.vec() = Eigen::Vector3d (msg->q.x, msg->q.y, msg->q.z);
	quat.w() = msg->q.w;
	quat.normalize();
	Vector3d omega(msg->w.x, msg->w.y, msg->w.z);

	controller_->setAttitudeState(quat, omega);

	// Catch no setpoint.
	if (!received_reference_) return;

	double dt = msg->header.stamp.toSec() - last_state_time_;
	ros::Time ctrl_activation = msg->header.stamp;
	last_state_time_ = ctrl_activation.toSec();
	
	if (isControlActive_) {
		controller_->step(dt);
		Vector3d control_rates = controller_->getControlRates();
		double thrust = controller_->getControlThrust();

		control_msg_.control.roll = control_rates(0);
		control_msg_.control.pitch = control_rates(1);
		control_msg_.control.yaw_dot = control_rates(2);
		control_msg_.control.thrust = thrust / Mass_; // Because the library works with acc
	} else {
		control_msg_.control.thrust = 0.0;
		control_msg_.control.roll = 0.0;
		control_msg_.control.pitch = 0.0;
		control_msg_.control.yaw_dot = 0.0;
	}
	ros::Time msg_timestamp = ros::Time::now();
	control_msg_.header.stamp = msg_timestamp;

	control_pub_.publish(control_msg_);

	return;
}


// Process an incoming setpoint point change.
void CBFControllerROS::onNewSetpoint(
		const testbed_msgs::ControlSetpoint::ConstPtr& msg) {
	if (msg->setpoint_type != "stop") {
		received_reference_ = true;
		isControlActive_ = true;

		Vector3d sp_pos(msg->p.x, msg->p.y, msg->p.z);
		Vector3d sp_vel(msg->v.x, msg->v.y, msg->v.z);
		Vector3d sp_acc(msg->a.x, msg->a.y, msg->a.z);
		controller_->setTranslationRef(sp_pos, sp_vel, sp_acc);
	} else {
		isControlActive_ = false;
	}
}

/*
void thread_fnc(void* p) {
	// Convert the pointer to pass information to the thread.
	Thread_arg* pArg = (Thread_arg*) p;

	double dt = pArg->period;
	CBFController* pcontroller = pArg->pcontroller;
	ros::Publisher ctrl_pub = pArg->ctrl_pub;
	ros::Publisher perf_pub = pArg->perf_pub;

	struct timespec time;
	struct timespec next_activation;

	struct timespec period_tms; 
	create_tspec(period_tms, dt);

	UType control_cmd;
	testbed_msgs::ControlStamped control_msg;

	double thrust = 0.032 * 9.81;

	cis_controller::PerformanceMsg ctrl_perf_msg;

	ros::Time last_sent_time_local = ros::Time::now();
	while (ros::ok()) {
		// Get current time
		clock_gettime(CLOCK_MONOTONIC, &time);
		timespec_sum(time, period_tms, next_activation);

		UType u_body(UType::Zero()); 

		if (pcontroller->isControlActive()) {
			// Do something
			pcontroller->Step(dt);

			// Get the desired jerk
			control_cmd = pcontroller->getControls();

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
			thrust = 0;
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
*/

#include "simulator/simulator.hpp"
#include "testbed_msgs/CustOdometryStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "utilities/timeutils/timeutils.hpp"

#include <time.h>

using namespace std;

void composeVRPN_msg(
		geometry_msgs::PoseStamped& vrpn_msg,
		const vector<double>& x,
		ros::Time t) {
	vrpn_msg.header.stamp = t;
	vrpn_msg.pose.position.x = x[0];
	vrpn_msg.pose.position.y = x[1];
	vrpn_msg.pose.position.z = x[2];

	vrpn_msg.pose.orientation.w = x[6];
	vrpn_msg.pose.orientation.x = x[7];
	vrpn_msg.pose.orientation.y = x[8];
	vrpn_msg.pose.orientation.z = x[9];

}

void composeCodom_msg(
		testbed_msgs::CustOdometryStamped& codom_msg,
		const vector<double>& x,
		const vector<double>& acc,
		ros::Time t) {
	codom_msg.header.stamp = t;
	codom_msg.p.x = x[0];
	codom_msg.p.y = x[1];
	codom_msg.p.z = x[2];
	codom_msg.v.x = x[3];
	codom_msg.v.y = x[4];
	codom_msg.v.z = x[5];
	codom_msg.a.x = acc[0];
	codom_msg.a.y = acc[1];
	codom_msg.a.z = acc[2];
}
void sim_thread_fnc(void* p);


/// =============================================================
XSimulator::XSimulator() {
	srand(time(NULL));
	initialized_ = false;
}

bool XSimulator::Initialize(const ros::NodeHandle& n) {

	ros::NodeHandle nl(n);

	name_ = ros::names::append(n.getNamespace(),
			"drone_sim");

	if (!LoadParameters(n)) {
		ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
		return false;
	}

	if (!RegisterCallbacks(n)) {
		ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
		return false;
	}

	state_pub_ = nl.advertise<testbed_msgs::CustOdometryStamped> (sim_state_topic_.c_str(), 10);

	vrpn_sim_pub_ = nl.advertise<geometry_msgs::PoseStamped> (vrpn_sim_pose_topic_.c_str(), 10);

	old_time_.sec = 0;
	old_time_.nsec = 0; 

	sim_ = new Dynamics_URates(parameters_);

	std::vector<double> x0(10, 0);
	x0[0] = initial_pos_[0];
	x0[1] = initial_pos_[1];
	x0[2] = initial_pos_[2];
	x0[6] = 1.0;
	sim_->set_state(x0);

	initialized_ = true;

	// Start the simulation and the publication of sensor data
	start_simulation(sim_period_);
	start_sensor_pub(sens_period_);

	return true;
}

void XSimulator::start_simulation(double dt) {
	arg_.period = dt;
	arg_.pParam = (void*) &parameters_;
	arg_.pSim = sim_;
	arg_.pub = state_pub_;

	sim_thread = std::thread(sim_thread_fnc, (void*) &arg_);
}

void XSimulator::start_sensor_pub(double dt) {
	pub_thread = std::thread(&XSimulator::pub_thread_fnc, this, dt);

}

bool XSimulator::LoadParameters(const ros::NodeHandle& n) {
	ros::NodeHandle nl("~");

	if (!nl.getParam("frame_name", frame_name_))
		return false;

	nl.param<std::string>("topics/input_control_topic", ctrl_topic_, 
			frame_name_ + "/control");

	nl.param<std::string>("topics/output_sensor_topic", 
			sim_sensor_topic_, "xsim_sensors");

	nl.param<std::string>("topics/output_state_topic", 
			sim_state_topic_, "xsim_state");

	nl.param<std::string>("topics/output_simvrpn_topic", vrpn_sim_pose_topic_,
			"/area0/sensors/optitrack/" + frame_name_ + "/data");

	// Load Model Parameter of the drone
	nl.param<double>("param/drone_mass", Mass_, 1.0);
	nl.param<double>("param/lin_drag_coeff", c_drag_, 0.00001);
	nl.param<double>("param/ang_drag_coeff", a_drag_, 0.00001);
	nl.param<double>("param/sim_period", sim_period_, 0.001);
	nl.param<double>("param/sensor_period", sens_period_, 0.002);

	nl.param<double>("param/sensor_noise", noise_std_, 0.003);


	std::string param_name;
	if (nl.searchParam("param/x0", param_name)) {
		std::vector<double> p_vec;
		n.getParam(param_name, p_vec);
		initial_pos_ = p_vec;

		std::cout << "Simulated Vehicle: Initial Position = ";
		for (auto el : initial_pos_) {
			std::cout << el << " ";
		}
		std::cout << std::endl;
	} else {
		initial_pos_ = std::vector<double>(3,0);
	}


	parameters_.Mass = Mass_;
	parameters_.c_drag = c_drag_;
	parameters_.a_drag = a_drag_;

	return true;
}

bool XSimulator::RegisterCallbacks(const ros::NodeHandle& n) {
	ros::NodeHandle nl;

	// Subscribe to the control topic and link to the 
	// control callback routing.
	ctrl_topic_sub_ = nl.subscribe(
			ctrl_topic_.c_str(),
			2,
			&XSimulator::ControlCallback,
			this);

	return true;
}


/**
 * Callback from the controller
 * - Update the control data in the simulator structure
 */
void XSimulator::ControlCallback(
		const testbed_msgs::ControlStamped::ConstPtr& msg) {

	ros::Time current_time = ros::Time::now();

	if (old_time_.toSec() > 0) {
		double dt = current_time.toSec() - old_time_.toSec();
		old_time_ = current_time;
		vector<double> u_(4);
		u_[0] = msg->control.thrust * parameters_.Mass; // Need to fix the mismatch...
		u_[1] = msg->control.roll;
		u_[2] = msg->control.pitch;
		u_[3] = msg->control.yaw_dot;
		sim_->set_inputs(u_);
	} else {
		old_time_ = current_time;
	}
}

/**
 * In case I wanted to call the function from the ROS interface directly.
 */
void XSimulator::updateState(double dt) {
	sim_->step(dt);
}


/**
 * Simulation thread
 *
 * The ROS class load the Arguments in a structure
 */
void sim_thread_fnc(void* p) {

	// Convert the pointer to pass information to the thread.
	simThread_arg* pArg = (simThread_arg*) p;

	double dt = pArg->period;
	void* pparam = pArg->pParam;
	IDynamics* psim = pArg->pSim;
	ros::Publisher pub = pArg->pub;

	struct timespec time;
	struct timespec next_activation;

	struct timespec period_tms; 
	create_tspec(period_tms, dt);

	std::vector<double> x(10);
	std::vector<double> acc(3);

	while (ros::ok()) {
		// Get current time
		clock_gettime(CLOCK_MONOTONIC, &time);
		timespec_sum(time, period_tms, next_activation);

		psim->step(dt);

		x = psim->get_state();
		acc = psim->get_acceleration();

		ros::Time timestamp = ros::Time::now();
		testbed_msgs::CustOdometryStamped sim_state_msg;
		composeCodom_msg(sim_state_msg, x, acc, timestamp);
		pub.publish(sim_state_msg); 

		clock_gettime(CLOCK_MONOTONIC, &time);
		if (time_diff(next_activation, time) < 0)
			std::cout << "Simulator Deadline Miss" << std::endl;
		clock_nanosleep(CLOCK_MONOTONIC,
				TIMER_ABSTIME, &next_activation, NULL);
	}
	ROS_INFO("Terminating Simulation Thread...\n");

}


double XSimulator::generate_noise() {
	double noise = (rand() % 100) / 100.0;
	noise *= noise_std_;

	return noise;
}


void XSimulator::pub_thread_fnc(double dt) {   
	struct timespec time;
	struct timespec next_activation;

	struct timespec period_tms; 
	create_tspec(period_tms, dt);

	std::vector<double> x(10);
	std::vector<double> acc(3);

	while (ros::ok()) {
		// Get current time
		clock_gettime(CLOCK_MONOTONIC, &time);
		timespec_sum(time, period_tms, next_activation);

		x = sim_->get_state();
		acc = sim_->get_acceleration();

		ros::Time timestamp = ros::Time::now();

		geometry_msgs::PoseStamped vrpn_msg;
		composeVRPN_msg(vrpn_msg, x, timestamp);

		// Add Noise
		vrpn_msg.pose.position.x += generate_noise();
                vrpn_msg.pose.position.y += generate_noise();
                vrpn_msg.pose.position.z += generate_noise();

		testbed_msgs::CustOdometryStamped sim_state_msg;
		composeCodom_msg(sim_state_msg, x, acc, timestamp);

		vrpn_sim_pub_.publish(vrpn_msg);
		//state_pub_.publish(sim_state_msg); 

		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_activation, NULL);
	}
	ROS_INFO("Terminating Simulation Thread...\n");
}


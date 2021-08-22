#include <ros/ros.h>
#include "cis_supervisor/cis_supervisor_ros.hpp"
#include <math.h>
#include <stdio.h>
#include <Eigen/Dense>

#define GRAVITY_MAGNITUDE (9.81f)

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace cis_supervisor {
	CISSupervisorROS::CISSupervisorROS() :
		initialized_(false) {
			ControllerDT_ = 0.05;
		}

	// Initialize.
	bool CISSupervisorROS::Initialize(const ros::NodeHandle& n) {
		std::string namepace_ = n.getNamespace();
		if (!LoadParameters()) {
			ROS_ERROR("CISSupervisor: Failed to load parameters");
			return false;
		}

		// Instatiate the CIS Supervisor class
		cis_supervisor_ = new CISSupervisor(Ad_, Bd_, Ed_, Vector3d::Zero());

		MatrixXd As(6, 9);
		As.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3);
		As.block(3, 0, 3, 3) = -MatrixXd::Identity(3, 3);
		
		MatrixXd Bs(6, 1);
		for (int i = 0; i < 6; i++) {
			Bs(i) = 3;
		}
		cis_supervisor_->AddSafeSet(As, Bs);

		InitPubSubs(n);

		Reset();

		initialized_ = true;
		return true;
	}

	// Load parameters. This may be overridden by derived classes.
	bool CISSupervisorROS::LoadParameters() {
		// Fetch the parameter in the private namespace
		ros::NodeHandle nl("~");

		nl.param<std::string>("vehicle_name", vehicle_name_, "cf1");

		// Controller Parameters
		if (!nl.getParam("param/controllerDT", ControllerDT_)) return false;
		if (!nl.getParam("param/vehicleMass", vehicle_Mass_)) return false;

		XmlRpc::XmlRpcValue XML;
		if (!nl.getParam("param/A", XML)) return false;
		Ad_ = MatrixXd(static_cast<int>(XML["rows"]), static_cast<int>(XML["cols"]));

		// Load the data
		std::vector<double> v;
		if (!nl.getParam("param/A/data", v)) return false;
		int counter = 0;
		for (auto it : v) {
			int row = counter / Ad_.cols();
			int col = counter % Ad_.cols();
			Ad_(row, col) = it;
			counter++;
		}
		std::cout << std::endl;
		std::cout << "Loaded Ad: " << std::endl << Ad_ << std::endl;

		if (!nl.getParam("param/B", XML)) return false;
		Bd_ = MatrixXd(static_cast<int>(XML["rows"]), static_cast<int>(XML["cols"]));
		if (!nl.getParam("param/B/data", v)) return false;
		counter = 0;
		for (auto it : v) {
			int row = counter / Bd_.cols();
			int col = counter % Bd_.cols();
			Bd_(row, col) = it;
			counter++;
		}
		std::cout << "Loaded Bd: " << std::endl << Bd_ << std::endl;

		if (!nl.getParam("param/E", XML)) return false;
		Ed_ = MatrixXd(static_cast<int>(XML["rows"]), static_cast<int>(XML["cols"]));
		if (!nl.getParam("param/E/data", v)) return false;
		counter = 0;
		for (auto it : v) {
			int row = counter / Ed_.cols();
			int col = counter % Ed_.cols();
			Ed_(row, col) = it;
			counter++;
		}
		std::cout << "Loaded Ed: " << std::endl << Ed_ << std::endl;

		// Topics
		if (!nl.getParam("topics/state", state_topic_)) return false;
		if (!nl.getParam("topics/ref_control", control_input_topic_)) return false;
		if (!nl.getParam("topics/obstacles", obstacle_input_topic_)) return false;

		nl.param<std::string>("topics/output_control", control_topic_, namespace_ + vehicle_name_ + "/control");

		return true;
	}

	// Register callbacks.
	bool CISSupervisorROS::InitPubSubs(const ros::NodeHandle& n) {
		ros::NodeHandle nl(n);

		// Subscribe to the topics and associate a callback upon new publications.
		state_sub_ = nl.subscribe(state_topic_.c_str(), 1, &CISSupervisorROS::StateCallback, this);
		ctrl_setpoint_sub_ = nl.subscribe(control_input_topic_.c_str(), 1, &CISSupervisorROS::UpdateControl, this);
		obstacle_sub_ = nl.subscribe(obstacle_input_topic_.c_str(), 1, &CISSupervisorROS::ObstacleCallback, this);

		// Advertise the publication of the control messages
		control_pub_ = nl.advertise<testbed_msgs::ControlStamped>(control_topic_.c_str(), 1, false);

		return true;
	}

	// Reset 
	void CISSupervisorROS::Reset(void) {
		thrust_ = 0.0;
		w_jerk_ = Vector3d::Zero();

		pos_ = Vector3d::Zero();
		vel_ = Vector3d::Zero();
		acc_ = Vector3d::Zero();

		w_q_b_.vec() = Vector3d::Zero();
		w_q_b_.w() = 1;

	}

	// --------------------------------------
	// Callback on new setpoint message
	// -1) Update the internal data storing the current control setpoint
	// -2) Filter the control throught the Supervisor
	void CISSupervisorROS::UpdateControl(const testbed_msgs::ControlStamped::ConstPtr& msg) {
		
		Vector3d b_jerk;
		Vector3d b_omega_ctrl;
		double thrust = msg->control.thrust;
		b_omega_ctrl(0) = msg->control.roll;
		b_omega_ctrl(1) = msg->control.pitch;
		b_omega_ctrl(2) = msg->control.yaw_dot;

		// Compute the jerk in body frame from the omega/thrust actuation command 
		b_jerk(X_COORD) = thrust * b_omega_ctrl(Y_COORD) / vehicle_Mass_;
		b_jerk(Y_COORD) = -thrust * b_omega_ctrl(X_COORD) / vehicle_Mass_;
		b_jerk(Z_COORD) = differentiate(thrust);

		// Compute the jerk in world frame
		w_jerk_ = w_q_b_ * b_jerk;

		// Call the supervisor
		std::cout << "Updating control ... " << std::endl;
		Vector3d w_jerk_filt = cis_supervisor_->UpdateControl(w_jerk_);
		
		// Convert the jerk into body frame
		Vector3d b_jerk_filt_ = w_q_b_.inverse()  * w_jerk_filt;

		Vector3d update_b_omega_ctrl(b_omega_ctrl);
		update_b_omega_ctrl(X_COORD) = -(vehicle_Mass_ / thrust_) * b_jerk_filt_(Y_COORD);
		update_b_omega_ctrl(Y_COORD) =  (vehicle_Mass_ / thrust_) * b_jerk_filt_(X_COORD);
		double thrust_new  = thrust_ + b_jerk_filt_(Z_COORD) * ControllerDT_; 

		testbed_msgs::ControlStamped control_msg;
		if (thrust_new > 0.05) {
			control_msg.control.roll = update_b_omega_ctrl(X_COORD);
			control_msg.control.pitch = update_b_omega_ctrl(Y_COORD);
			control_msg.control.yaw_dot = update_b_omega_ctrl(Z_COORD);
		}
		thrust_ = std::max(thrust_new, 0.0);
		std::clamp(thrust_new, 0.0, 2.0 * vehicle_Mass_ * GRAVITY_MAGNITUDE);

		control_msg.control.thrust = thrust_new / vehicle_Mass_; // Because the library works with acc (XXX Fix this)

		// Set the message timestamp and publish the message.
		ros::Time now = ros::Time::now();
		control_msg.header.stamp = now; 
		control_pub_.publish(control_msg);
	}

	// --------------------------------------
	// Callback on new state data:
	// -1) Fetch data from the ROS message
	// -2) Compute the control input
	// -3) Publish the control message
	void CISSupervisorROS::StateCallback(
			const testbed_msgs::CustOdometryStamped::ConstPtr& msg) {

		// Read the message into the state
		pos_(0) = msg->p.x; pos_(1) = msg->p.y; pos_(2) = msg->p.z;
		vel_(0) = msg->v.x; vel_(1) = msg->v.y; vel_(2) = msg->v.z;
		acc_(0) = msg->a.x; acc_(1) = msg->a.y; acc_(2) = msg->a.z; 

		w_q_b_.vec() = Vector3d (msg->q.x, msg->q.y, msg->q.z);
		w_q_b_.w() = msg->q.w;
		w_q_b_.normalize();
	}


	// --------------------------------------
	// Callback on new obstacle data:
	void CISSupervisorROS::ObstacleCallback(
			const cis_supervisor::ObstacleMsg::ConstPtr& msg) {

		// Read the message into the state
		ObstacleData obst_data;
		obst_data.id = msg->id;
		obst_data.pos(0) = msg->p.x; obst_data.pos(1) = msg->p.y; obst_data.pos(2) = msg->p.z;
		obst_data.vel(0) = msg->v.x; obst_data.vel(1) = msg->v.y; obst_data.vel(2) = msg->v.z;
		
		cis_supervisor_->UpdateObstacle(obst_data);

	}

	double CISSupervisorROS::differentiate(double x) {
		return 1;
	}
}

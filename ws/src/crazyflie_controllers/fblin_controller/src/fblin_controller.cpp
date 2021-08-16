#include <ros/ros.h>
#include "fblin_controller/fblin_controller.hpp"
#include <math.h>
#include <stdio.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <iomanip>

#define GRAVITY_MAGNITUDE (9.81f)

namespace fblin_controller {
	FBLinController::FBLinController() :
		received_setpoint_(false),
		initialized_(false) {
		}

	// Initialize.
	bool FBLinController::Initialize(const ros::NodeHandle& n) {
		std::string namepace_ = n.getNamespace();
		if (!LoadParameters()) {
			ROS_ERROR("FBLinController: Failed to load parameters");
			return false;
		}

		InitPubSubs(n);

		Reset();

		initialized_ = true;
		return true;
	}

	// Load parameters. This may be overridden by derived classes.
	bool FBLinController::LoadParameters() {
		// Fetch the parameter in the private namespace
		ros::NodeHandle nl("~");

		// Topics.
		nl.param<std::string>("vehicle_name", vehicle_name_, "cf1");
		nl.param<std::string>("topics/control", control_topic_, namespace_ + vehicle_name_ + "/control");

		if (!nl.getParam("topics/state", state_topic_)) return false;
		if (!nl.getParam("topics/setpoint", setpoint_topic_)) return false;

		// Controller Parameters
		if (!nl.getParam("param/vehicleMass", vehicleMass_)) return false;

		if (!nl.getParam("param/kp_xy", kp_xy_)) return false;
		if (!nl.getParam("param/kd_xy", kd_xy_)) return false;
		if (!nl.getParam("param/ka_xy", ka_xy_)) return false;

		if (!nl.getParam("param/kp_z", kp_z_)) return false;
		if (!nl.getParam("param/kd_z", kd_z_)) return false;
		if (!nl.getParam("param/ka_z", ka_z_)) return false;

		if (!nl.getParam("param/kR_z", kR_z_)) return false;
		
		return true;
	}

	// Register callbacks.
	bool FBLinController::InitPubSubs(const ros::NodeHandle& n) {
		ros::NodeHandle nl(n);

		// Subscribe to the topics and associate a callback upon new publications.
		state_sub_ = nl.subscribe(state_topic_.c_str(), 1, &FBLinController::StateCallback, this);
		setpoint_sub_ = nl.subscribe(setpoint_topic_.c_str(), 1, &FBLinController::SetpointCallback, this);

		// Declare the publication of the control messages
		control_pub_ = nl.advertise<testbed_msgs::ControlStamped>(control_topic_.c_str(), 1, false);

		return true;
	}

	// Reset 
	void FBLinController::Reset(void) {
		thrust_ = 0.0;
		u_body_ = Vector3d::Zero();

		sp_pos_ = Vector3d::Zero();
		sp_vel_ = Vector3d::Zero();
		sp_acc_ = Vector3d::Zero();
		sp_jrk_ = Vector3d::Zero();

		setpoint_type_ = "stop";

		pos_ = Vector3d::Zero();
		vel_ = Vector3d::Zero();
		acc_ = Vector3d::Zero();

		quat_.vec() = Vector3d::Zero();
		quat_.w() = 1;

		received_setpoint_ = false;
	}

	// --------------------------------------
	// Callback on new setpoint message
	// -1) Update the internal data storing the current setpoint
	void FBLinController::SetpointCallback(
			const testbed_msgs::ControlSetpoint::ConstPtr& msg) {

		setpoint_type_ = msg->setpoint_type; 

		sp_pos_(0) = msg->p.x;
		sp_pos_(1) = msg->p.y;
		sp_pos_(2) = msg->p.z;

		sp_vel_(0) = msg->v.x;
		sp_vel_(1) = msg->v.y;
		sp_vel_(2) = msg->v.z;

		sp_acc_(0) = msg->a.x;
		sp_acc_(1) = msg->a.y;
		sp_acc_(2) = msg->a.z;

		sp_jrk_(0) = msg->j.x;
		sp_jrk_(1) = msg->j.y;
		sp_jrk_(2) = msg->j.z;

		received_setpoint_ = true;
	}

	// --------------------------------------
	// Callback on new state data:
	// -1) Fetch data from the ROS message
	// -2) Compute the control input
	// -3) Publish the control message
	void FBLinController::StateCallback(
			const testbed_msgs::CustOdometryStamped::ConstPtr& msg) {
		// Catch no setpoint.
		if (!received_setpoint_)
			return;

		// Read the message into the state
		pos_(0) = msg->p.x; pos_(1) = msg->p.y; pos_(2) = msg->p.z;
		vel_(0) = msg->v.x; vel_(1) = msg->v.y; vel_(2) = msg->v.z;
		acc_(0) = msg->a.x; acc_(1) = msg->a.y; acc_(2) = msg->a.z; 

		quat_.vec() = Vector3d (msg->q.x, msg->q.y, msg->q.z);
		quat_.w() = msg->q.w;
		quat_.normalize();

		ros::Time now = ros::Time::now();
		double dt = now.toSec() - previous_.toSec();

		if (dt > 0.18) {
			dt = 0.18;
			previous_ = now;

			// Compute error
			Vector3d p_error = sp_pos_ - pos_;
			Vector3d v_error = sp_vel_ - vel_;
			Vector3d a_error = sp_acc_ - acc_;

			// Compute the control action (jerk) in world coordinates
			Vector3d u_world = Vector3d::Zero(); 
			u_world(0) = kp_xy_ * p_error(0) + kd_xy_ * v_error(0) + ka_xy_ * a_error(0);
			u_world(1) = kp_xy_ * p_error(1) + kd_xy_ * v_error(1) + ka_xy_ * a_error(1);
			u_world(2) = kp_z_ * p_error(2) + kd_z_ * v_error(2) + ka_z_ * a_error(2);
			u_world += sp_jrk_;

			// Map the control action from body to world frame
			u_body_ = quat_.inverse() * u_world;

			// Extract the axis of the body frame (expressed in world frame) 
			Matrix3d R = quat_.toRotationMatrix();
			Vector3d x_axis = R.col(0); 
			Vector3d z_axis = R.col(2);

			// Map the jerk into angular velocity and thrust (actual control signals) 
			testbed_msgs::ControlStamped control_msg; // ROS message for the controls
			if (thrust_ > 0.05) {
				control_msg.control.roll = -(u_body_(1) / thrust_) * vehicleMass_;
				control_msg.control.pitch = (u_body_(0) / thrust_) * vehicleMass_;
			}
			thrust_ = std::max(thrust_ + vehicleMass_ * u_body_(2) * dt, 0.0);
			std::clamp(thrust_, 0.0, 2.0 * vehicleMass_ * GRAVITY_MAGNITUDE);

			control_msg.control.thrust = thrust_ / vehicleMass_; // Because the library works with acc (XXX Fix this)

			// Yaw control
			Vector3d yyaw = z_axis.cross(Vector3d::UnitX());
			Vector3d xyaw = yyaw.cross(z_axis);

			Vector3d ni = x_axis.cross(xyaw); 
			double alpha = std::asin(ni.norm());
			ni.normalize();
			// Express the axis in body frame
			Vector3d nb = quat_.inverse() * ni;
			double yaw_ctrl = kR_z_ * alpha;
			control_msg.control.yaw_dot = yaw_ctrl;

			// In case the setpoint type was a stop, put everything to zero.
			if (setpoint_type_ == "stop") {
				control_msg.control.thrust = 0.0;
				control_msg.control.roll = 0.0;
				control_msg.control.pitch = 0.0;
				control_msg.control.yaw_dot = 0.0;
			}

			// Set the message timestamp and publish the message.
			control_msg.header.stamp = now; 
			control_pub_.publish(control_msg);
		}
	}
}

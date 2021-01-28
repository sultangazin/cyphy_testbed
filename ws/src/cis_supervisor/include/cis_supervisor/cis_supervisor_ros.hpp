///////////////////////////////////////////////////////////////////////////////
//
//  CIS Supervisor ROS wrapper
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <thread>

// Messages
#include <testbed_msgs/CustOdometryStamped.h>
#include <testbed_msgs/ControlSetpoint.h>

#include "cis_supervisor/CISSupervisorTune.h"

#include "cis_supervisor/cis_supervisor.hpp"
#include "utilities/network_parser/network_parser.hpp"


struct Thread_arg {
	double period;
	CISSupervisor* psupervisor;
	ros::Publisher ctrl_pub;
	ros::Publisher perf_pub;
	Eigen::Quaterniond quat;
};

// =================================================================
// CLASS
//
class CISSupervisorROS {

	public:
		CISSupervisorROS();
		~CISSupervisorROS();

		// Initialize this class by reading parameters and loading callbacks.
		bool Initialize(const ros::NodeHandle& n);

	private:
		ros::NodeHandle node_;
		std::string node_name_;

		// Initialized flag and name.
		bool received_reference_;
		bool active_;

		bool initialized_;

		std::string setpoint_type_;

		// Name of the area where the controller is located 
		std::string area_name_;
		std::string controller_name_;

		// Name of the vehicle controlled
		std::string vehicle_name_;

		// Load parameters and register callbacks.
		bool LoadParameters(const ros::NodeHandle& n);
		bool RegisterCallbacks(const ros::NodeHandle& n);
		bool SetUpPublications(const ros::NodeHandle& n);

		bool cis_supervisor_tune(
				cis_supervisor::CISSupervisorTune::Request& req,
				cis_supervisor::CISSupervisorTune::Response& res);

		// Remember last time we got a state callback.
		double last_state_time_;

		// Callback on Pose 
		void onNewState(
				const testbed_msgs::CustOdometryStamped::ConstPtr& msg);
		// Callback on Setpoint
		void onNewSetpoint(const testbed_msgs::ControlSetpoint::ConstPtr& msg);

		// Publishers and subscribers.
		// Output publishers:
		ros::Publisher cis_supervisor_ctrl_;
		ros::Publisher performance_pub_;

		// Input Topics names
		std::string state_topic_;
		std::string setpoint_topic_;

		// Output Topics names
		std::string ctrls_topic_;
		std::string performance_topic_;

		ros::Subscriber setpoint_sub_;
		ros::Subscriber state_sub_;

		ros::ServiceServer cis_supervisor_service;

		// DATA -------------------------------------------------------
		std::array<double, CISS_STATESIZE_1D> ctrl_gains_;

		// THREAD OBJECTS 
		Thread_arg periodic_thread_arg_;
		std::thread periodic_thread_;
		std::thread net_disc_thr_;

		// ===========================================================
		// CLASSES
		CISSupervisor* supervisor_;

		Eigen::Quaterniond quat_;

};

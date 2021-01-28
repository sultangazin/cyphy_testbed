#ifndef COMMANDER_INTERFACE_H
#define COMMANDER_INTERFACE_H

#include <mutex>
#include <thread>
#include <string>
#include <unordered_map>

#include "ros/ros.h"

#include "control_router/SelectController.h"
#include "control_router/EnableNWController.h"
#include <testbed_msgs/ControlStamped.h>
#include <crazyflie_driver/PWM.h>
#include "control_router/NetworkStatusMsg.h"

#include "utilities/network_parser/network_parser.hpp"

#define DEFAULT_NUM_ELEMENTS (50)

struct AveragingFilter {
	std::mutex mx;
	std::vector<double> meas;
	int BufferSize;
	unsigned int head;
	unsigned int tail;
	int NumOfElements;

	AveragingFilter() : meas(std::vector<double>(DEFAULT_NUM_ELEMENTS, 0)) {
		BufferSize = DEFAULT_NUM_ELEMENTS;
		head = 0;
		tail = 0;
		NumOfElements = 0;
	}

	AveragingFilter(int n) : meas(std::vector<double>(n, 0)) {
		BufferSize = n;
		head = 0;
		tail = 0;
		NumOfElements = 0;
	}

	AveragingFilter(AveragingFilter&& af) : meas(std::vector<double>(af.BufferSize, 0)) {
		BufferSize = af.BufferSize;
		head = 0;
		tail = 0;
		NumOfElements = 0;
	}

	double getAvg() {
		double output = 0;
		mx.lock();
		if (NumOfElements < 2) {
			mx.unlock();
			return 0;
		}
		output =(meas[head] - meas[tail]) / (NumOfElements - 1); 
		mx.unlock();
		return output;
	}

	double getAvgFreq() {
		double output = 0;
		mx.lock();
		if (NumOfElements < 2) {
			mx.unlock();
			return 0;
		}
		output = (NumOfElements - 1)/(meas[head] - meas[tail]);
		mx.unlock();
		return output;
	}

	void addMeas(double m) {
		mx.lock();
		if (NumOfElements == 0) {
			NumOfElements++;
			meas.at(head) = m;
		} else {
			head = (head  + 1) % BufferSize;
			meas.at(head) = m;
			NumOfElements = (NumOfElements < BufferSize) ? NumOfElements + 1 : BufferSize;
			if (head == tail) {
				tail = (tail + 1) % BufferSize;
			}
		}
		mx.unlock();
	}
};



// =================================================================
// CLASS
//
class ControlRouter {
	public:
		ControlRouter();
		~ControlRouter();

		bool Initialize(const ros::NodeHandle& n);

		// Switch controller
		bool ctrl_select_callback(
				control_router::SelectController::Request  &req,
				control_router::SelectController::Response &res);

		bool enable_network_controllers(
				control_router::EnableNWController::Request& req,
				control_router::EnableNWController::Response& res);

	private:
		std::mutex mx;
		ros::NodeHandle node_;


		std::string area_name_;
		std::string target_name_;

		// Networked Control Enable
		bool enabled_;

		// Current controller
		std::string current_controller_;

		// Current control signal
		testbed_msgs::ControlStamped curr_control_;
		crazyflie_driver::PWM curr_pwm_control_;

		// Load Parameters
		bool LoadParameters(const ros::NodeHandle& n);

		bool RegisterCallbacks();

		int UpdateControlPublishers();

		bool AssociateTopicsToCallbacks(const ros::NodeHandle& n);

		int UpdatePublishers();

		void net_discovery(int ms);

		// Network Parser
		NetworkParser network_parser;

		// Helper
		control_router::NetworkStatusMsg generate_network_status_msg();

		// Service Server
		ros::ServiceServer ctrl_select_srv_;
		ros::ServiceServer ctrl_enable_srv_;

		// Topic subscription

		/**
		 * MAP containing sensor topic information
		 */
		std::unordered_map<std::string, TopicData> outchannels_; 

		/**
		 * MAP for active subscribers
		 */
		std::unordered_map<std::string, ros::Subscriber> active_subscriber; 
		/*
		   ros::Subscriber control_sub_;
		   ros::Subscriber control2_sub_;
		   ros::Subscriber dd_control_sub_;
		   ros::Subscriber dd_control2_sub_;
		   */

		// Topic publication
		ros::Publisher control_pub_;
		ros::Publisher control_pwm_pub_;
		ros::Publisher network_status_pub_;

		// Topics callbacks
		void update_control_callback(
				const boost::shared_ptr<testbed_msgs::ControlStamped const>& msg,
				void* arg);
		void update_pwm_control_callback(
				const boost::shared_ptr<crazyflie_driver::PWM const>& msg,
				void* arg);

		std::vector<AveragingFilter> probes;

		// Names and topics
		std::string name_;
		std::string vehicle_name_;

		std::string output_control_topic_;
		std::string output_control_pwm_topic_;

		std::string network_status_topic_;

		/*
		   std::string input_control_topic_;
		   std::string input_control2_topic_;
		   std::string input_dd_control_topic_;
		   std::string input_dd_control2_topic_;
		   */

		std::thread net_disc_thr;


		bool initialized_;
		bool ready_;
};

#endif

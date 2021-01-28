/** @file commander.cpp
 *  @author l.pannocchi@gmail.com
 *
 */
#include "control_router/control_router.hpp"

// =================================================================
// =================================================================
ControlRouter::ControlRouter() {
	for (int i = 0; i < 3; i++) {
		probes.push_back(AveragingFilter());
	}
	enabled_ = false;
	ready_ = false;
}

ControlRouter::~ControlRouter() {
}


bool ControlRouter::LoadParameters(const ros::NodeHandle& n) {

	ros::NodeHandle np("~");
	np.param<std::string>("param/vehicle_name", vehicle_name_, "cf1");

	// Output
	np.param<std::string>("topics/output_ctrl", output_control_topic_,
			"/" + vehicle_name_ + "/control");
	np.param<std::string>("topics/output_ctrl_pwm", output_control_pwm_topic_,
			"/" + vehicle_name_ + "/cmd_pwm");
	np.param<std::string>("topics/output_network_status", network_status_topic_,
			"/" + vehicle_name_ + "/network_ctrl_status");
	return true;
}

/**
 * Initialization function
 */
bool ControlRouter::Initialize(const ros::NodeHandle& n) {
	// Get the name of the node
	name_ = ros::this_node::getName();
	node_ = n;

	// Load Parameters
	if (!LoadParameters(n)) {
		ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
		return false;
	}


	// Register callback
	if (!RegisterCallbacks()) {
		ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
		return false;
	}


	// Advertise topics/Services
	// I want to advertise service in the node namespace
	ros::NodeHandle nh("~");
	ctrl_select_srv_ = nh.advertiseService(
			"/" + vehicle_name_ + "/nw_ctrl_select",
			&ControlRouter::ctrl_select_callback, this);

	ctrl_enable_srv_ = nh.advertiseService(
			"/" + vehicle_name_ + "/nw_ctrl_enable",
			&ControlRouter::enable_network_controllers, this);

	// Publishers.
	control_pub_ = node_.advertise<testbed_msgs::ControlStamped>(
			output_control_topic_.c_str(), 1, false);

	control_pwm_pub_ = node_.advertise<crazyflie_driver::PWM>(
			output_control_pwm_topic_.c_str(), 1, false);

	network_status_pub_ = node_.advertise<control_router::NetworkStatusMsg>(
			network_status_topic_.c_str(), 1, false);


	net_disc_thr = std::thread(&ControlRouter::net_discovery, this,
			500);

	initialized_ = true; 

	return true;
}


// Update the list of control publications referring to the 
// vehicle in the current area.
bool ControlRouter::RegisterCallbacks() {
	int npub = UpdatePublishers();

	if (npub > 0) {
		AssociateTopicsToCallbacks(node_); 
		ready_ = true;
		if (current_controller_.empty()) {
			current_controller_ = outchannels_.begin()->second.node_name;
		}
	}

	return true;
}


int ControlRouter::UpdatePublishers() {
	std::unordered_map<std::string, ros::master::TopicInfo> sdata;
	network_parser.query_controllers(sdata, area_name_, target_name_);

	for (auto el : sdata) {
		std::string sname = el.first; 
		if (outchannels_.count(sname) == 0) {
			std::cout << "[CONTROL ROUTER] Connecting to Controller: " <<
				sname << std::endl;
			TopicData str;
			str.topic_name = el.second.name;
			str.area_name = area_name_;
			str.node_name = sname;
			str.datatype = el.second.datatype;
			str.frequency = 0.0;
			str.isActive = true;
			str.enabled = true;
			outchannels_.insert(
					std::pair<std::string, TopicData>(
						sname, str)
					);
		}
	}
	return sdata.size();
}

bool ControlRouter::AssociateTopicsToCallbacks(const ros::NodeHandle& n) {
	ros::NodeHandle nl(n);
	for (auto el : outchannels_) { // For every registered channel
		std::string topic_name = el.second.topic_name;
		std::string topic_datatype = el.second.datatype;
		if (el.second.enabled) { // If the channel is not disabled
			if (active_subscriber.count(el.first) == 0) { // Associate the callback
				if (topic_datatype == "testbed_msgs/ControlStamped") {
					active_subscriber.insert(
							std::pair<std::string, ros::Subscriber>(
								el.first,
								nl.subscribe<testbed_msgs::ControlStamped>(
									topic_name.c_str(),
									5, 
									boost::bind(&ControlRouter::update_control_callback, this, _1,
										(void*)&outchannels_[el.first].node_name),
									ros::VoidConstPtr(),
									ros::TransportHints().tcpNoDelay()
									)
								)
							);
				}

				if (topic_datatype == "crazyflie_driver/PWM") {
					active_subscriber.insert(
							std::pair<std::string, ros::Subscriber>(
								el.first,
								nl.subscribe<crazyflie_driver::PWM>(
									topic_name.c_str(),
									5, 
									boost::bind(&ControlRouter::update_pwm_control_callback, this, _1,
										(void*)&outchannels_[el.first].node_name),
									ros::VoidConstPtr(),
									ros::TransportHints().tcpNoDelay()
									)
								)
							);
				}
			}
		} else { // If the chanell is disabled: unsubscribe
			if (active_subscriber.count(el.first) > 0) {
				std::cout << "Unsubscribing from " << el.first << std::endl;
				active_subscriber[el.first].shutdown();
				active_subscriber.erase(el.first);
			}
		}
	}

	return true;
}


// Service Callbacks
bool ControlRouter::ctrl_select_callback(
		control_router::SelectController::Request  &req,
		control_router::SelectController::Response &res) {

	current_controller_ = req.sel_controller;

	res.curr_controller = current_controller_;

	std::cout << "Selected Network Controller: " <<
		current_controller_ << std::endl;

	control_router::NetworkStatusMsg msg = generate_network_status_msg();

	network_status_pub_.publish(msg);

	return true;
}

bool ControlRouter::enable_network_controllers(
		control_router::EnableNWController::Request  &req,
		control_router::EnableNWController::Response &res) {

	enabled_ = req.enable_nwctrl;

	if (enabled_) {
		std::cout << "Enabled Network Controller!" << std::endl;
	} else {
		std::cout << "Disabled Network Controller!" << std::endl;
	}

	res.status = enabled_;

	return enabled_;
}



void ControlRouter::update_control_callback(
		const boost::shared_ptr<testbed_msgs::ControlStamped const>& msg, void* arg) {

	std::string node_name = *(std::string*) arg;

	if (current_controller_ == node_name) {
		curr_control_.control.thrust = msg->control.thrust;
		curr_control_.control.roll = msg->control.roll;  
		curr_control_.control.pitch = msg->control.pitch;
		curr_control_.control.yaw_dot = msg->control.yaw_dot;

		if (enabled_) {
			testbed_msgs::ControlStamped out_msg;
			out_msg = *msg;
			control_pub_.publish(out_msg);
		}
	}
}

void ControlRouter::update_pwm_control_callback(
		const boost::shared_ptr<crazyflie_driver::PWM const>& msg, void* arg) {

	std::string node_name = *(std::string*) arg;

	ros::Time now = ros::Time::now();
	probes.at(0).addMeas(now.toSec());

	if (current_controller_ == node_name) {
		curr_pwm_control_.pwm0 = msg->pwm0;
		curr_pwm_control_.pwm1 = msg->pwm1;
		curr_pwm_control_.pwm2 = msg->pwm2;
		curr_pwm_control_.pwm3 = msg->pwm3;

		if (enabled_) {
			probes.at(2).addMeas(now.toSec());
			crazyflie_driver::PWM out_msg;
			out_msg = *msg;
			control_pwm_pub_.publish(out_msg);
		}
	}

	control_router::NetworkStatusMsg net_msg = generate_network_status_msg();
	network_status_pub_.publish(net_msg);

}

control_router::NetworkStatusMsg ControlRouter::generate_network_status_msg() {
	control_router::NetworkStatusMsg msg;

	msg.header.stamp = ros::Time::now();

	msg.network_ctrl_active = enabled_;
	msg.active_controller_id = current_controller_;

	for (int i = 0; i < probes.size(); i++) {
		msg.msgs_freq.push_back(probes.at(i).getAvgFreq()); 
	}

	return msg;
}

void ControlRouter::net_discovery(int ms) {
	while (ros::ok()) {
		RegisterCallbacks();
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}
}

/** @file commander.cpp
 *  @author l.pannocchi@gmail.com
 *
 */
#include "control_router/control_router.hpp"


// =================================================================
// =================================================================
ControlRouter::ControlRouter() {
        current_controller_ = 1;

        enabled_ = false;
}

ControlRouter::~ControlRouter() {
}


bool ControlRouter::LoadParameters(const ros::NodeHandle& n) {

    ros::NodeHandle np("~");
    np.param<std::string>("param/vehicle_name", vehicle_name_,"cf1");

    // XXX I should make a search online like for the state_aggregator
    // XXX I temporarily hard code things here
    np.param<std::string>("topics/input_ctrl", input_control_topic_, "/area0/controller/geometric_ctrl/" + vehicle_name_ + "/control");
    np.param<std::string>("topics/input_ctrl", input_control2_topic_, "/area0/controller/geometric_ctrl2/" + vehicle_name_ + "/control");

    np.param<std::string>("topics/output_ctrl", output_control_topic_, "/" + vehicle_name_ + "/control");
    return true;
}

/**
 * Initialization function
 */
bool ControlRouter::Initialize(const ros::NodeHandle& n) {
    // Get the name of the node
    name_ = ros::this_node::getName();

    // Load Parameters
    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }

    ros::NodeHandle ng(n);
    control_sub_ = ng.subscribe(
        input_control_topic_.c_str(), 1, &ControlRouter::update_control_callback, this);
    control2_sub_ = ng.subscribe(
        input_control2_topic_.c_str(), 1, &ControlRouter::update_control2_callback, this);


    // Advertise topics/Services
    // I want to advertise service in the node namespace
    ros::NodeHandle nh("~");
    ctrl_select_srv_ = nh.advertiseService("/" + vehicle_name_ + "/nw_ctrl_select",
            &ControlRouter::ctrl_select_callback, this);

    ctrl_enable_srv_ = nh.advertiseService("/" + vehicle_name_ + "/nw_ctrl_enable",
            &ControlRouter::enable_network_controllers, this);

    
    // Publishers.
    control_pub_ = ng.advertise<testbed_msgs::ControlStamped>(
                output_control_topic_.c_str(), 1, false);

    initialized_ = true; 

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

    return enabled_;
}



//XXX There will be a more complex selection...now, I just want to have 
//simple to test the control switching between onboard/offboard
void ControlRouter::update_control_callback(
        const testbed_msgs::ControlStamped::ConstPtr& msg) {

    if (current_controller_ == 1) {
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

void ControlRouter::update_control2_callback(
        const testbed_msgs::ControlStamped::ConstPtr& msg) {

    if (current_controller_ == 2) {
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

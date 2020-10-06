#ifndef COMMANDER_INTERFACE_H
#define COMMANDER_INTERFACE_H

#include <mutex>
#include "ros/ros.h"

#include "control_router/SelectController.h"
#include "control_router/EnableNWController.h"
#include <testbed_msgs/ControlStamped.h>

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

        // Networked Control Enable
        bool enabled_;
        bool stopped_;

        // Current controller
        int current_controller_;

        // Current control signal
        testbed_msgs::ControlStamped curr_control_;

        // Load Parameters
        bool LoadParameters(const ros::NodeHandle& n);
        
        bool RegisterCallbacks(const ros::NodeHandle& n);

        // Service Server
        ros::ServiceServer ctrl_select_srv_;
        ros::ServiceServer ctrl_enable_srv_;

        // Topic subscription
        ros::Subscriber control_sub_;

        // Topic publication
        ros::Publisher control_pub_;
        
        // Topics callbacks
        void update_control_callback(
                const testbed_msgs::ControlStamped::ConstPtr& msg);

        // Names and topics
        std::string name_;
        std::string vehicle_name_;
        std::string output_control_topic_;
        std::string input_control_topic_;
        std::string input_control2_topic_;

        bool initialized_;
};

#endif

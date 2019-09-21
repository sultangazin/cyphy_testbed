/**
 * @file setpoint_ghost_pub.cpp
 *
 * @author l.pannocchi@gmail.com 
 *
 *
 * This ROS node subscribe to the topic regarding the generated 
 * control reference and converts the message into an odometry ROS 
 * message that can be plotted with rviz.
 *
 */

#include <ros/ros.h>
#include "odometry_publisher/odometry_publisher.hpp"

// MAIN ----------------------------------------------
int main(int argc, char** argv) {

	ROS_INFO("Starting Control Setpoint publisher");

	// Initialize the ROS stack
	ros::init(argc, argv, "CtlSetpoint_publisher");
	ros::NodeHandle nh;

    OdometryPublisher op;

    if (!op.Initialize(nh)) {
            ROS_ERROR("%s: Failed to initialize node.", 
                            ros::this_node::getName().c_str());
            return EXIT_FAILURE;
    }

    ros::spin();

    return EXIT_SUCCESS;
}

#include <ros/ros.h>
#include "dd_controller/dd_controller.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "Data_Driven_Estimator");
	ros::NodeHandle nh;

	ROS_INFO("Starting Data driven estimator");
    
    DDController ddcontroller;
		
    if(!ddcontroller.Initialize(nh)) {
        ROS_ERROR("%s: Failed to initialize state_aggregator.",
                ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }

    ROS_INFO("Start Spinning");
    ros::spin();

    return EXIT_SUCCESS;
}

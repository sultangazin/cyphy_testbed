#include <ros/ros.h>
#include "cis_supervisor/cis_supervisor_ros.hpp"


int main(int argc, char** argv) {

        ros::init(argc, argv, "CIS_Supervisor");
        ros::NodeHandle nh;

        ROS_INFO("Starting Data CIS Supervisor Node");

        CISSupervisorROS cissup_node;

        if(!cissup_node.Initialize(nh)) {
                ROS_ERROR("%s: Failed to initialize cis supervisor.",
                                ros::this_node::getName().c_str());
                return EXIT_FAILURE;
        }

        ros::MultiThreadedSpinner spinner(3);
        spinner.spin();

        return EXIT_SUCCESS;
}

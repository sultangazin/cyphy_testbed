#include "control_router/control_router.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "control-router");
    ros::NodeHandle n;
    ROS_INFO("Starting Control Router");

    ControlRouter control_router;


    if (!control_router.Initialize(n)) {
        ROS_ERROR("%s: Failed to initialize!", 
                ros::this_node::getName().c_str());
        return EXIT_FAILURE;
    }

    ros::spin();

    return EXIT_SUCCESS;
}

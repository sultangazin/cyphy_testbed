#include "mn_detector/mn_detector.hpp"
#include "testbed_msgs/MNDetectorPerf.h"


MNDetector::MNDetector() {
    initialized_ = false;
}

bool MNDetector::Initialize(const ros::NodeHandle& n) {

    ros::NodeHandle nl(n);

    name_ = ros::names::append(n.getNamespace(),
            "malicious_node_detector");

    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }

    if (!RegisterCallbacks(n)) {
        ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
        return false;
    }

    mndetector_perf_pub_ = nl.advertise<testbed_msgs::MNDetectorPerf>(
            mnd_perf_topic_.c_str(), 5);

    for (int i = 0; i < 8; i++) {
        anchors_[i] = 0.0;
    }
    
    for (int i = 0; i < 3; i++) {
        rpy_deg_[i] = 0.0;
    }

    initialized_ = true;

    return true;
}

bool MNDetector::LoadParameters(const ros::NodeHandle& n) {
    ros::NodeHandle nl("~");

    if (!nl.getParam("topics/input_control_topic", ctrl_topic_))
        return false;

    if (!nl.getParam("topics/input_sensors", sensor_topic_))
        return false;

    nl.param<std::string>("topics/output_mnd_perf_topic", 
            mnd_perf_topic_, "mnd_perf");

    return true;
}

bool MNDetector::RegisterCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl("~");
    // Subscribe to the sensor topic and link the sensor
    // callback routine.
    sensor_topic_sub_ = nl.subscribe(
            sensor_topic_.c_str(),
            2,
            &MNDetector::SensorsCallback,
            this);

    // Subscribe to the control topic and link to the 
    // control callback routing.
    ctrl_topic_sub_ = nl.subscribe(
        ctrl_topic_.c_str(),
        2,
        &MNDetector::ControlCallback,
        this);

    attitude_topic_sub_ = nl.subscribe(
            "/cf2/external_pose_rpy",
            2,
            &MNDetector::AttitudeCallback,
            this);

    return true;
}

void MNDetector::SensorsCallback(const testbed_msgs::AnchorMeas::ConstPtr& msg) {
    int _id = 0;
    // Fetch the current simulated anchor measurements
    for (auto it = msg->meas.begin(); it != msg->meas.end(); it++) {
        //std::cout << "Anchor[" << _id << "] = " << *it << std::endl;
        anchors_[_id++] = *it;
    }
}

void MNDetector::AttitudeCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    // Take the attitude of the vehicle that can be used to compute the linear 
    // acceleration on the plane.
    rpy_deg_[0] = msg->vector.x;
    rpy_deg_[1] = msg->vector.y;
    rpy_deg_[2] = msg->vector.z;
}

void MNDetector::ControlCallback(
        const testbed_msgs::ControlStamped::ConstPtr& msg) {
    //std::cout << msg->control.thrust << std::endl; 

    testbed_msgs::MNDetectorPerf perf_msg;


    perf_msg.header.stamp = ros::Time::now();
    perf_msg.thrust = msg->control.thrust;

    for (int i = 0; i < 8; i++) {
        perf_msg.meas.push_back(anchors_[i]);
    }


    for (int i = 0; i < 3; i++) {
        perf_msg.rpy[i] = rpy_deg_[i];
    }


    mndetector_perf_pub_.publish(perf_msg);
}

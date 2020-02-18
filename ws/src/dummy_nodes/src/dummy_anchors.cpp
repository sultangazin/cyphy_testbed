#include "dummy_anchors/dummy_anchors.hpp"
#include <testbed_msgs/AnchorMeas.h>
#include <crazyflie_driver/AnchorMeas.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <fstream>
#include <ros/package.h>

DummyAnchors::DummyAnchors() {
    initialized_ = false;
}


bool DummyAnchors::Initialize(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);

    name_ = ros::this_node::getName();

    if (!LoadParameters(nl)) {
        ROS_ERROR("%s: Error loading parameters!\n", name_.c_str());
        return false;
    } else {
        ROS_INFO("%s: Correctly loaded parameters!\n", name_.c_str());
    }

    if (!RegisterCallbacks(nl)) {
        ROS_ERROR("%s: Error registering callbacks!\n", name_.c_str());
        return false;
    } else {
        ROS_INFO("%s: Correcly registered callbacks!\n", name_.c_str());
    }

    sensor_output_pub_ = nl.advertise<crazyflie_driver::AnchorMeas>(
            sensor_output_topic_.c_str(), 5);

    sensor_all_output_pub_ = nl.advertise<testbed_msgs::AnchorMeas>(
            sensor_all_output_topic_.c_str(), 2);

    initialized_ = true;
    enable_distortion_ = false;

    malicious_node_ = 0;
    distortion_value_ = 0.1;

    return true;
}


bool DummyAnchors::LoadParameters(const ros::NodeHandle& n) {
    ros::NodeHandle nl("~");

    std::string config_file;

    nl.param<std::string>("topics/input_feed_topic", 
            input_feed_topic_, "");
    nl.param<std::string>("topics/output_out_topic",
            sensor_output_topic_, "");
    nl.param<std::string>("topics/output_all_out_topic",
            sensor_all_output_topic_, "");

    nl.param<bool>("enable_distortion", enable_distortion_, false);

    nl.param<int>("malicious_node", malicious_node_, 0);

    nl.param<float>("distortion_value", distortion_value_, 0.1);

    nl.param<std::string>("anchors_file", config_file, "anchors.yaml");

    ROS_INFO("%s: Input feed topic = %s", name_.c_str(), input_feed_topic_.c_str());
    ROS_INFO("%s: Ouput output topic = %s", name_.c_str(), sensor_output_topic_.c_str());
    ROS_INFO("%s: Enable Distortion = %d", name_.c_str(), enable_distortion_);
    ROS_INFO("%s: Distortion Value = %3.2f", name_.c_str(), distortion_value_);

    YAML::Node config = YAML::LoadFile(ros::package::getPath("dummy_nodes") + '/' + config_file);

    numberOfAnchors = config["NumOfAnchors"].as<int>();

    for (int i = 0; i < numberOfAnchors; i++) {
        Eigen::Vector3f v;
        v(0) = config[std::to_string(i)]["x"].as<float>();
        v(1) = config[std::to_string(i)]["y"].as<float>();
        v(2) = config[std::to_string(i)]["z"].as<float>();
        anchors.push_back(v);
        ROS_INFO("%s: Loading anchor[%d] = \n", name_.c_str(), i);
        std::cout << v << std::endl;
    }

    if (input_feed_topic_.empty()) {
        ROS_ERROR("%s: Input feed not provided!\n", name_.c_str());
        return false;
    } else {
        return true;  
    }
}


bool DummyAnchors::RegisterCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl("~");
    input_feed_sub_ = nl.subscribe(
            input_feed_topic_.c_str(),
            5,
            &DummyAnchors::onFeedCallback, this,
            ros::TransportHints().tcpNoDelay());

    return true;
}


/**
 * Callback on reception of camera information:
 * It simulates the measurements of the anchors
 */
void DummyAnchors::onFeedCallback(
        const boost::shared_ptr<testbed_msgs::CustOdometryStamped const>& msg) {

    ros::NodeHandle nl("~");
    ros::Time current_time = ros::Time::now();

    Eigen::Vector3f vehicle_p;

    // Fetch the position information from the pose message
    vehicle_p(0) = msg->p.x;
    vehicle_p(1) = msg->p.y;
    vehicle_p(2) = msg->p.z;

    // The testbed anchor meas contains a single vector with all the anchors
    // distances.
    testbed_msgs::AnchorMeas output_all_msg;
    output_all_msg.header.stamp = current_time;

    //std::cout << "Meas = "; 
    for (int i = 0; i < numberOfAnchors; i++) {
        float anchors_meas = (vehicle_p - anchors[i]).norm();   
        output_all_msg.meas.push_back(anchors_meas);
        //std::cout << anchors_meas << "   ";
    }
    //std::cout << " | Vehicle = " << vehicle_p;
    //std::cout << std::endl;
    sensor_all_output_pub_.publish(output_all_msg);

    // The crazyflie_driver message contains only the information about a 
    // single anchor.
    for (unsigned char i = 0; i < numberOfAnchors; i++) {
        crazyflie_driver::AnchorMeas output_msg;

        float anchors_meas = (vehicle_p - anchors[i]).norm();  

        nl.getParam("enable_distortion", enable_distortion_);
        if (enable_distortion_) {
            nl.getParam("malicious_node", malicious_node_);
            nl.getParam("distortion_value", distortion_value_);
            if (i == malicious_node_)
                anchors_meas = anchors_meas + distortion_value_;
        }

        output_msg.dist = anchors_meas;
        output_msg.id = i;

        output_msg.x_anchor = (float)anchors[i](0);
        output_msg.y_anchor = (float)anchors[i](1);
        output_msg.z_anchor = (float)anchors[i](2);

        sensor_output_pub_.publish(output_msg);
        ros::Duration(0.001).sleep();
    }
}

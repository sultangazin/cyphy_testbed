#ifndef DUMMY_ANCHORS_HPP
#define DUMMY_ANCHORS_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include "Eigen/Dense"

class DummyAnchors {
    public:
        DummyAnchors();

        // Initialization methods
        bool Initialize(const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

    private:
        bool initialized_;

        std::string name_;
        std::string input_feed_topic_;
        std::string sensor_output_topic_; 

        ros::Subscriber input_feed_sub_;
        ros::Publisher sensor_output_pub_;

        int numberOfAnchors;
        std::vector<Eigen::Vector3f> anchors;

        // Callbacks
        void onFeedCallback(
                const boost::shared_ptr<geometry_msgs::PoseStamped const>& msg);

};
#endif

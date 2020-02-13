#ifndef MNDETECTOR_HPP
#define MNDETECTOR_HPP

#include <ros/ros.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "testbed_msgs/AnchorMeas.h"
#include "testbed_msgs/ControlStamped.h"

class MNDetector {
    public:

        MNDetector();

        bool Initialize(const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

        // List of Callbacks methods
        void SensorsCallback(
                const testbed_msgs::AnchorMeas::ConstPtr& msg);
        void ControlCallback(
                const testbed_msgs::ControlStamped::ConstPtr& msg);
        void AttitudeCallback(
                const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    private:
        std::string name_;

        bool initialized_;

        ros::Subscriber sensor_topic_sub_;
        ros::Subscriber ctrl_topic_sub_;
        ros::Subscriber attitude_topic_sub_;

        ros::Publisher mndetector_perf_pub_;

        std::string sensor_topic_;
        std::string ctrl_topic_; 
        std::string mnd_perf_topic_;

        std::array<float, 8> anchors_;
        float rpy_deg_[3];
};

#endif

///////////////////////////////////////////////////////////////////////////////
//
// State aggregator, which fuses state information from different sources and 
// compose an overall estimate of the vehicle position. 
// 
//
///////////////////////////////////////////////////////////////////////////////

#ifndef STATE_AGGREGATOR_H
#define STATE_AGGREGATOR_H

#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <thread>
#include "Eigen/Dense"
#include <tf/transform_broadcaster.h>
#include <testbed_msgs/CustOdometryStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <state_aggregator/ControlSensor.h>

#include "filter/polyfilter.hpp"
#include "utilities/network_parser/network_parser.hpp"

struct TopicData {
    std::string topic_name;
    std::string area_name;
    std::string sensor_name;
    std::string datatype;
    double frequency;
    bool isActive;
    bool enabled;
};


using namespace Eigen;

struct kfThread_arg {
    double period;
    PolyFilter* pfilt;
};

// =================================================================
// CLASS
//
class StateAggregator {

    public:
        StateAggregator();
        ~StateAggregator();

        // Initialize this class by reading parameters and loading callbacks.
        bool Initialize(const ros::NodeHandle& n);

        // Maps with the callbacks
        //void (StateAggregator::*) (const boost::shared_ptr<geometry_msgs::PoseStamped const>&, void*) callbacks_pose; 

    /*    std::unordered_map<
            std::string,
            void (StateAggregator::*) (const boost::shared_ptr<geometry_msgs::PointStamped const>&, void*)>
                callbacks_point; 
*/
    private:
        ros::NodeHandle node_;

        std::string area_name_;
        std::string target_name_;

        double _sigmax;
        double _sigmay;

        // Network Parser
        NetworkParser network_parser;

        // Load parameters and register callbacks.
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks();
        bool AssociateTopicsToCallbacks(const ros::NodeHandle& n);
        int UpdateSensorPublishers();
        void net_discovery(int ms);

        // Remember last time we got a state callback.
        double last_state_time_;

        // Callback on Pose 
        void onNewPose(const boost::shared_ptr<geometry_msgs::PoseStamped const>& msg, void* arg);
        void onNewPosition( const boost::shared_ptr<geometry_msgs::PointStamped const>& msg, void* arg);

        // Services
        bool control_sensor(state_aggregator::ControlSensor::Request& req,
                state_aggregator::ControlSensor::Response& res);
        ros::ServiceServer sensor_service;

        // Publishers and subscribers.
        // COMM ------------------------------------------------------------
        // Output publishers and broadcaster:
        ros::Publisher ext_pos_pub_;
        ros::Publisher pose_pub_;
        ros::Publisher pose_rpy_pub_;
        ros::Publisher odometry_pub_;
        ros::Publisher codometry_pub_;
        ros::Publisher rs_pub_;

        /**
         * MAP containing sensor topic information
         */
        std::unordered_map<std::string, TopicData> inchannels_; 

        /**
         * MAP for active subscribers
         */
        std::unordered_map<std::string, ros::Subscriber> active_subscriber; 

        std::string object_name_;
        // Topics names
        std::string vrpn_topic_;
        std::string gtrack_topic_;

        std::string ext_position_topic_;
        std::string ext_pose_topic_;
        std::string ext_pose_rpy_topic_;
        std::string ext_odom_topic_;
        std::string ext_codom_topic_;
        std::string rs_codom_topic_;

        // Initialized flag and name.
        bool received_reference_;
        bool initialized_;
        std::string name_;

        std::string axis_up_;

        // DATA ------------------------------------------------------------
        // Pubblication variables 
        geometry_msgs::PoseStamped ext_pose_msg_;
        geometry_msgs::Vector3Stamped ext_pose_rpy_msg_;
        geometry_msgs::PointStamped ext_position_msg_;
        geometry_msgs::TransformStamped ext_odom_trans_;
        testbed_msgs::CustOdometryStamped ext_codometry_msg_;


        // FILTER
        PolyFilter* _pfilt;
        kfThread_arg arg_;
        std::thread kf_thread;
        std::thread net_disc_thr;

        // ===========================================================
        // Helper variables
        Eigen::Vector3d p_;
        Eigen::Vector3d v_;

        Eigen::Vector3d p_pf_;
        Eigen::Vector3d p_old_;
        Eigen::Vector3d vel_;
        Eigen::Vector3d euler_;
        Eigen::Vector3d w_;

        Eigen::Quaterniond q_;
        Eigen::Quaterniond q_old_;
        Eigen::Quaterniond qd_;
        Eigen::Quaterniond q_pf_;

        double t_delay_; 
        double v_alpha_, qd_alpha_;

}; //\class StateAggregator


#endif

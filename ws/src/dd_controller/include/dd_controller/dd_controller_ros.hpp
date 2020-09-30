///////////////////////////////////////////////////////////////////////////////
//
//  Data Driven controller ROS wrapper
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DD_CONTROLLER_ROS_HPP_ 
#define DD_CONTROLLER_ROS_HPP_

#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <thread>

// Service Includes
#include <dd_controller/DDControllerTune.h>

// Messages
#include <geometry_msgs/PoseStamped.h>
#include <dd_controller/MotorsCtrlStamped.h>

#include "dd_controller/dd_controller.hpp"
#include "dd_controller/dd_estimator.hpp"
#include "dd_controller/dd_estimator_param.hpp"

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



struct Thread_arg {
    double period;
};

// =================================================================
// CLASS
//
class DDControllerROS {

    public:
        DDControllerROS();
        ~DDControllerROS();

        // Initialize this class by reading parameters and loading callbacks.
        bool Initialize(const ros::NodeHandle& n);


    private:
        ros::NodeHandle node_;
        std::string node_name_;

        // Initialized flag and name.
        bool received_reference_;
        bool initialized_;

        // Name of the area where the controller is located 
        std::string area_name_;
        // Name of the vehicle controlled
        std::string target_name_;

        // Network Parser
        NetworkParser network_parser;

        // Load parameters and register callbacks.
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks();
        bool AssociateTopicsToCallbacks(const ros::NodeHandle& n);
        int UpdateSensorPublishers();
        bool SetUpPublications(const ros::NodeHandle& n);
        void net_discovery(int ms);

        // Remember last time we got a state callback.
        double last_state_time_;

        // Callback on Pose 
        void onNewPose(const boost::shared_ptr<geometry_msgs::PoseStamped const>& msg, void* arg);

        // Services for controlling the controller
        bool dd_controller_tune(dd_controller::DDControllerTune::Request& req,
                dd_controller::DDControllerTune::Response& res);
        ros::ServiceServer dd_controller_service;

        // Publishers and subscribers.
        // Output publishers:
        ros::Publisher motor_ctrls_pub_;
        ros::Publisher state_estimate_pub_;
        ros::Publisher param_estimate_pub_;

        // Input Topics names
        std::string pose_meas_topic_;

        // Output Topics names
        std::string motor_ctrls_topic_;
        std::string state_estimate_topic_;
        std::string param_estimate_topic_;

        // Messages 
        dd_controller::MotorsCtrlStamped motor_ctrls_msg_;

        /**
         * MAP containing sensor topic information
         * The map is indicized with the name of the sensor names
         */
        std::unordered_map<std::string, TopicData> inchannels_;

        /**
         * MAP for active subscribers
         * The map is indicized with the name of the sensors
         */
        std::unordered_map<std::string, ros::Subscriber> active_subscriber;


        // DATA ------------------------------------------------------------
        // THREAD OBJECTS 
        Thread_arg periodic_thread_arg_;
        std::thread periodic_thread_;

        std::thread net_disc_thr_;

        // ===========================================================
        // CLASSES
        DDController* pddctrl_;
        DDEstimator* pddest_;
        DDParamEstimator* pddparest_;

}; //\class DDControllerROS


#endif

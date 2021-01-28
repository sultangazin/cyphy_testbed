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
#include <crazyflie_driver/PWM.h>
#include <testbed_msgs/ControlSetpoint.h>


#include "dd_controller/dd_controller.hpp"
#include "dd_controller/dd_estimator.hpp"
#include "dd_controller/dd_estimator_param.hpp"

#include "utilities/network_parser/network_parser.hpp"

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
        bool active_;
        bool estimator_ready_;
        bool controller_ready_;
        int initialization_counter_;

        bool initialized_;

        std::string setpoint_type_;

        // Name of the area where the controller is located 
        std::string area_name_;

        std::string controller_name_;

        // Name of the vehicle controlled
        std::string vehicle_name_;

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
        // Callback on Control
        void onNewControl(const crazyflie_driver::PWM::ConstPtr& msg);
        // Callback on Setpoint
        void onNewSetpoint(const testbed_msgs::ControlSetpoint::ConstPtr& msg);
        
        // Services for controlling the controller
        bool dd_controller_tune(dd_controller::DDControllerTune::Request& req,
                dd_controller::DDControllerTune::Response& res);

        ros::ServiceServer dd_controller_service;

        // Publishers and subscribers.
        // Output publishers:
        ros::Publisher motor_ctrls_pub_;
        ros::Publisher state_estimate_pub_;
        ros::Publisher param_estimate_pub_;
        ros::Publisher performance_pub_;

        // Input Topics names
        std::string pose_meas_topic_;
        std::string setpoint_topic_;

        // Output Topics names
        std::string motor_ctrls_topic_;
        std::string state_estimate_topic_;
        std::string param_estimate_topic_;
        std::string performance_topic_;
        std::string actuated_pwm_topic_;

        Eigen::Matrix<double, DDCTRL_OUTPUTSIZE, 1> pwm_ctrls_;

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

        ros::Subscriber setpoint_sub_;
        ros::Subscriber controller_sub_;


        // DATA -------------------------------------------------------
        std::array<double, 2> gains_x;
        std::array<double, 2> gains_y;
        std::array<double, 4> gains_alpha2d;
        std::array<double, 16> gains_beta2d;

        std::array<double, 2> bbeta_x;
        std::array<double, 2> bbeta_y;
        std::array<double, 16> blbounds;
        std::array<double, 16> bubounds;

        std::array<double, 2> Kxy;
		std::array<double, 2> Kz;
		std::array<double, 2> Katt;
		std::array<double, 2> Kyaw;

        // Drop control packets modulo 'drop_mod_'
        int drop_mod_;

        // Number of control cycles
        int ctrl_counter_;

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

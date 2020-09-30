#include "Eigen/Dense"
#include "math.h"
#include <chrono>

#include "dd_controller/dd_controller_ros.hpp"

// Messages include
#include <dd_controller/StateEstimateStamped.h>
#include <dd_controller/ParamEstimateStamped.h>


#include "utilities/timeutils/timeutils.hpp"
#include "utilities/custom_conversion/custom_conversion.hpp"

void thread_fnc(void* p);

// =================================================================
// CLASS
//
DDControllerROS::DDControllerROS():
    received_reference_(false),
    last_state_time_(-1.0),
    initialized_(false) { 
    };

DDControllerROS::~DDControllerROS() {};

bool DDControllerROS::Initialize(const ros::NodeHandle& n) {

    node_ = n;
    ros::NodeHandle nl(n);

    // Set the node name
    node_name_ = ros::this_node::getName().c_str();

    // Load parameters
    if (!LoadParameters(nl)) {
        ROS_ERROR("%s: Failed to load parameters.", node_name_.c_str());
        return false;
    }

    // Register callbacks
    if (!RegisterCallbacks()) {
        ROS_ERROR("%s: Failed to register callbacks.", node_name_.c_str());
        return false;
    }

    
    // Instantiate classes
    pddctrl_ = new DDController();
    pddest_ = new DDEstimator();
    pddparest_ = new DDParamEstimator();

    // Setup output publications and services
    SetUpPublications(nl);

    /*
    // In case I wanted to do something periodically
    periodic_thread_arg_.period = 0.002;
    periodic_thread_ = std::thread(thread_fnc, (void*) &arg_);
    */

    net_disc_thr_ = std::thread(&DDControllerROS::net_discovery, this,
            500);

    initialized_ = true;

    return true;
}

bool DDControllerROS::LoadParameters(const ros::NodeHandle& n) {

    ros::NodeHandle np("~");
    std::string key;

    np.param<std::string>("param/target_name", target_name_,"cf1");

    // External pose
    np.param<std::string>("topics/in_pose_sensor_topic", pose_meas_topic_,
            "external_pose");
    

    // Output Motors 
    np.param<std::string>("topics/out_motors_topic", motor_ctrls_topic_,
            "motor_ctrls");

    // State Estimate 
    np.param<std::string>("topics/out_state_estimate_topic", state_estimate_topic_,
            "dd_estimate");

    // Parameters Estimation 
    np.param<std::string>("topics/out_param_estimate_topic", param_estimate_topic_,
            "dd_param_estimate");

    /*
    // Params
    if (np.searchParam("sigmax", key)) {
        ROS_INFO("Found parameter %s!", key.c_str());
        n.getParam(key, _sigmax);
        ROS_INFO("Setting parameter %s = %f", 
                "sigmax", _sigmax);
    } else {
        _sigmax = 0.1;
        ROS_INFO("No param 'sigmax' found!"); 
        ROS_INFO("Setting default parameter %s = %f", 
                "sigmax", _sigmax);
    } 
    */

    area_name_ = "area0";

    return true;
}

bool DDControllerROS::SetUpPublications(const ros::NodeHandle& n) {

    ros::NodeHandle nl(n);
    // Output Publications
    motor_ctrls_pub_ = nl.advertise<dd_controller::MotorsCtrlStamped> (motor_ctrls_topic_.c_str(), 10);
    state_estimate_pub_= nl.advertise<dd_controller::StateEstimateStamped> (state_estimate_topic_.c_str(), 10);
    param_estimate_pub_ = nl.advertise<dd_controller::ParamEstimateStamped> (param_estimate_topic_.c_str(), 10);

    // Advertise Services
    dd_controller_service = node_.advertiseService(
            "dd_controller", &DDControllerROS::dd_controller_tune, this);

    return true;
}


int DDControllerROS::UpdateSensorPublishers() {

    // In order to automatize the subscription to sensor topics 
    // I need to fetch information from the network.
    // I will save the information in a data structure indicized
    // with the name of the sensors.
    // The State aggregator knows the name of the vehicle which is tracking. 
    // The State aggregator knows the name of the area in which is tracking.
    // The following query will get the name of the sensors that are in the 
    // current area and that are providing information regarding the target
    // vehicle.
    //
    std::unordered_map<std::string, ros::master::TopicInfo> sdata;
    network_parser.query_sensors(sdata, area_name_, target_name_);

    // For each topic found save the data relative to the sensor in the node
    // 'inchannels' data structure.
    for (auto el : sdata) {
        std::string sname = el.first; 
        if (inchannels_.count(sname) == 0) {
            std::cout << "[" << node_name_ << "] Found Sensor Publication: " <<
                sname << std::endl;

            TopicData tp_data_str;
            tp_data_str.topic_name = el.second.name;
            tp_data_str.area_name = area_name_;
            tp_data_str.sensor_name = sname;
            tp_data_str.datatype = el.second.datatype;
            tp_data_str.frequency = 0.0;
            tp_data_str.isActive = true;
            tp_data_str.enabled = true;

            inchannels_.insert(
                    std::pair<std::string, TopicData>(
                        sname, tp_data_str)
                    );
        }
    }
}


bool DDControllerROS::AssociateTopicsToCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);
    for (auto el : inchannels_) { // For every registered channel
        std::string topic_name = el.second.topic_name;
        std::string topic_datatype = el.second.datatype;

        if (el.second.enabled) { // If the channel is not disabled
            // Associate the callback to the sensor if it is not in the list
            if (active_subscriber.count(el.first) == 0) {  
                // I just take the pose data
                if (topic_datatype == "geometry_msgs/PoseStamped") {
                    // In the MAP 'active_subscriber' there are the subscriber objects
                    // related to active publishers.
                    active_subscriber.insert(
                        std::pair<std::string, ros::Subscriber>(
                            el.first,
                            nl.subscribe<geometry_msgs::PoseStamped>(
                                topic_name.c_str(),
                                5, 
                                boost::bind(&DDControllerROS::onNewPose, this, _1,
                                    (void*)&inchannels_[el.first].sensor_name),
                                ros::VoidConstPtr(),
                                ros::TransportHints().tcpNoDelay()
                                )
                            )
                        );
                }
            }
        } else { // If the chanell is disabled: unsubscribe
                if (active_subscriber.count(el.first) > 0) {
                    std::cout << "Unsubscribing from " << el.first << std::endl;
                    active_subscriber[el.first].shutdown();
                    active_subscriber.erase(el.first);
                }
            }
        }

    return true;
}


bool DDControllerROS::RegisterCallbacks() {
    // Update the list of sensors publications referring to the 
    // vehicle in the current area.
    UpdateSensorPublishers();
    AssociateTopicsToCallbacks(node_); 

    return true;
}


// CALLBACK ---------------------------------------------------------------------
/* 
 * Topic Callback
 * Whenever I receive a new Trajectory message, update the odometry.
 */
void DDControllerROS::onNewPose(const boost::shared_ptr<geometry_msgs::PoseStamped const>& msg, void* arg) {
    double dt;
    static timespec t_old;
    timespec t;

    std::string sensor_name = *(std::string*) arg;

    // Take the time
    ros::Time current_time = ros::Time::now();
    static ros::Time old_time {};

    Eigen::Quaterniond quat;
    Eigen::Vector3d pos;

    pos(0) = msg->pose.position.x;
    pos(1) = msg->pose.position.y;
    pos(2) = msg->pose.position.z;

    quat.x() = msg->pose.orientation.x;
    quat.y() = msg->pose.orientation.y;
    quat.z() = msg->pose.orientation.z;
    quat.w() = msg->pose.orientation.w;
    
    // Read the timestamp of the message
    t.tv_sec = msg->header.stamp.sec;
    t.tv_nsec = msg->header.stamp.nsec;

    double roll =  atan2(2.0 * quat.y() * quat.z() + 2.0 * quat.w() * quat.x(), 
            1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y()));
    double pitch = asin(2.0 * quat.w() * quat.y() - 2.0 * quat.x()*quat.z());
    double yaw =  atan2(2.0 * quat.x() * quat.y() + 2.0 * quat.z() * quat.w(),
            1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
    
    std::array<double, DDEST_NUMOFCHANNELS> meas {
        pos(0),
        pos(1),
        pos(2),
        roll,
        pitch,
        yaw
    };

    // Feed the data into the DD estimator
    pddest_->AddMeas(meas, ros::Time::now().toSec());
    
    // Trigger an estimation step
    // (I should think if it's the case to move this step in a separate thread)
    pddest_->Step();

    // Fetch the state from the estimator
    state_t estim_state;
    pddest_->GetState(&estim_state);

    dd_controller::StateEstimateStamped est_msg;
    est_msg.pos.x = estim_state.position(0);
    est_msg.pos.y = estim_state.position(1);
    est_msg.pos.z = estim_state.position(2);

    est_msg.vel.x = estim_state.velocity(0);
    est_msg.vel.y = estim_state.velocity(1);
    est_msg.vel.z = estim_state.velocity(2);

    est_msg.acc.x = estim_state.acceleration(0);
    est_msg.acc.y = estim_state.acceleration(1);
    est_msg.acc.z = estim_state.acceleration(2);

    est_msg.attitude.x = estim_state.attitude(0);
    est_msg.attitude.y = estim_state.attitude(1);
    est_msg.attitude.z = estim_state.attitude(2);

    est_msg.attitude_d.x = estim_state.attitude_d(0);
    est_msg.attitude_d.y = estim_state.attitude_d(1);
    est_msg.attitude_d.z = estim_state.attitude_d(2);

    est_msg.attitude_dd.x = estim_state.attitude_dd(0);
    est_msg.attitude_dd.y = estim_state.attitude_dd(1);
    est_msg.attitude_dd.z = estim_state.attitude_dd(2);

    // Publish it
    state_estimate_pub_.publish(est_msg);

    return;
}


bool DDControllerROS::dd_controller_tune(
        dd_controller::DDControllerTune::Request& req,
        dd_controller::DDControllerTune::Response& res) {
    std::cout << "[" << node_name_ << "]" << " Changing Settings. " << std::endl;
    
    return true;
}

void DDControllerROS::net_discovery(int ms){
    while (ros::ok()) {
        RegisterCallbacks();
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
}

void thread_fnc(void* p) {
   
    // Convert the pointer to pass information to the thread.
    Thread_arg* pArg = (Thread_arg*) p;

    double dt = pArg->period;

    struct timespec time;
    struct timespec next_activation;
    
    struct timespec period_tms; 
    create_tspec(period_tms, dt);

    while (ros::ok()) {
        // Get current time
        clock_gettime(CLOCK_MONOTONIC, &time);
        timespec_sum(time, period_tms, next_activation);

        // Do something

        clock_nanosleep(CLOCK_MONOTONIC,
                TIMER_ABSTIME, &next_activation, NULL);
    }
    ROS_INFO("Terminating Thread...\n");
}

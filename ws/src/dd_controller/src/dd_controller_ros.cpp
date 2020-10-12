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
    active_(false),
    estimator_ready_(false),
    controller_ready_(false),
    initialization_counter_(4),
    setpoint_type_("stop"),
    last_state_time_(-1.0),
    initialized_(false),
    pwm_ctrls_(Eigen::Matrix<double, DDCTRL_OUTPUTSIZE, 1>::Zero()) { 
        drop_mod_ = 1;
        ctrl_counter_ = 0;
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

    pddparest_->SetGains(gains_x, gains_y, gains_alpha2d,  gains_beta2d);
	pddparest_->SetBounds(bbeta_x, bbeta_y, blbounds, bubounds);
    //pddparest_->SetParams(DDParams& pa);

    // Setup output publications and services
    SetUpPublications(nl);

    // Setup one-time subscription
    setpoint_sub_ = nl.subscribe(setpoint_topic_.c_str(), 1,
            &DDControllerROS::onNewSetpoint, this);

    controller_sub_ = nl.subscribe(actuated_pwm_topic_.c_str(), 1,
            &DDControllerROS::onNewControl, this);

    /*
    // In case I wanted to do something periodically
    periodic_thread_arg_.period = 0.002;
    periodic_thread_ = std::thread(thread_fnc, (void*) &arg_);
    */

    net_disc_thr_ = std::thread(&DDControllerROS::net_discovery, this,
            500);


    initialized_ = true;

    ROS_INFO("[%s] Initialized!", node_name_.c_str());

    return true;
}

bool DDControllerROS::LoadParameters(const ros::NodeHandle& n) {

    ros::NodeHandle np("~");
    std::string key;

    // Controller name
    np.param<std::string>("param/controller_name", controller_name_, "DD_ctrl");

    // Vehicle name
    np.param<std::string>("param/vehicle_name", vehicle_name_,"cf1");

    // Control Setpoint
    np.param<std::string>("topics/in_ctrl_setpoint_topic", setpoint_topic_,
            "/" + vehicle_name_ + "/setpoint");
    
    // Output Motors 
    np.param<std::string>("topics/out_motors_topic", motor_ctrls_topic_,
            "/area0/controller/" +
            controller_name_ + "/" + vehicle_name_ + "/cmd_pwm");

    np.param<std::string>("topics/in_motors_topic", actuated_pwm_topic_,
            "/" + vehicle_name_ + "/cmd_pwm");
            

    // State Estimate 
    np.param<std::string>("topics/out_state_estimate_topic", state_estimate_topic_,
            "/" + controller_name_ + "/" + vehicle_name_ + "/dd_estimate");

    // Parameters Estimation 
    np.param<std::string>("topics/out_param_estimate_topic", param_estimate_topic_,
            "/" + controller_name_ + "/" + vehicle_name_ + "/dd_param_estimate");

    np.param<std::string>("param/area_name", area_name_, "area0");

    np.param<int>("param/drop_mod", drop_mod_, 1);

    // Load the parameter for the dd controller
    // It is a terrible implementation. I should optimize the
    // code replications :-D.
    std::string param_name;
    if (np.searchParam("param/k_par_x", param_name)) {
        std::cout << "Found " << param_name << std::endl; 
        std::vector<double> p_vec;
        n.getParam(param_name, p_vec);
        std::copy_n(p_vec.begin(), 2, gains_x.begin());
        
        std::cout << "gain_x: ";
        for (auto el : gains_x) {
            std::cout << el << " ";
        }
        std::cout << std::endl;
    } else {
        ROS_INFO("No param 'param/k_par_x' found in an upward search");
    }

    if (np.searchParam("param/k_par_y", param_name)) {
        std::cout << "Found " << param_name << std::endl; 
        std::vector<double> p_vec;
        n.getParam(param_name, p_vec);
        std::copy_n(p_vec.begin(), 2, gains_y.begin());
        
        std::cout << "gain_y: ";
        for (auto el : gains_y) {
            std::cout << el << " ";
        }
        std::cout << std::endl;
    } else {
        ROS_INFO("No param 'param/k_par_y' found in an upward search");
    }

    if (np.searchParam("param/k_par_a2", param_name)) {
        std::cout << "Found " << param_name << std::endl; 
        std::vector<double> p_vec;
        n.getParam(param_name, p_vec);
        std::copy_n(p_vec.begin(), gains_alpha2d.size(), gains_alpha2d.begin());

        std::cout << "gain_alpha2d: ";
        for (auto el : gains_alpha2d) {
            std::cout << el << " ";
        }
        std::cout << std::endl;
    } else {
        ROS_INFO("No param 'param/k_par_a2' found in an upward search");
    }

    if (np.searchParam("param/k_par_b2", param_name)) {
        std::cout << "Found " << param_name << std::endl; 
        std::vector<double> p_vec;
        n.getParam(param_name, p_vec);
        std::copy_n(p_vec.begin(), gains_beta2d.size(), gains_beta2d.begin());

        std::cout << "gain_beta2d: ";
        for (auto el : gains_beta2d) {
            std::cout << el << " ";
        }
        std::cout << std::endl;
    } else {
        ROS_INFO("No param 'param/k_par_b2' found in an upward search");
    }

    if (np.searchParam("param/betax_lim", param_name)) {
        std::cout << "Found " << param_name << std::endl; 
        std::vector<double> p_vec;
        n.getParam(param_name, p_vec);
        std::copy_n(p_vec.begin(), bbeta_x.size(), bbeta_x.begin());

        std::cout << "bbeta_x: ";
        for (auto el : bbeta_x) {
            std::cout << el << " ";
        }
        std::cout << std::endl;
    } else {
        ROS_INFO("No param 'param/betax_lim' found in an upward search");
    }

    if (np.searchParam("param/betay_lim", param_name)) {
        std::cout << "Found " << param_name << std::endl; 
        std::vector<double> p_vec;
        n.getParam(param_name, p_vec);
        std::copy_n(p_vec.begin(), bbeta_y.size(), bbeta_y.begin());

        std::cout << "bbeta_y: ";
        for (auto el : bbeta_y) {
            std::cout << el << " ";
        }
        std::cout << std::endl;
    } else {
        ROS_INFO("No param 'param/betay_lim' found in an upward search");
    }

    if (np.searchParam("param/b2d_lbound", param_name)) {
        std::cout << "Found " << param_name << std::endl; 
        std::vector<double> p_vec;
        n.getParam(param_name, p_vec);
        std::copy_n(p_vec.begin(), blbounds.size(), blbounds.begin());

        std::cout << "b2d_lbound: ";
        for (auto el : blbounds) {
            std::cout << el << " ";
        }
        std::cout << std::endl;
    } else {
        ROS_INFO("No param 'param/b2d_lbound' found in an upward search");
    }
    
    if (np.searchParam("param/b2d_ubound", param_name)) {
        std::cout << "Found " << param_name << std::endl; 
        std::vector<double> p_vec;
        n.getParam(param_name, p_vec);
        std::copy_n(p_vec.begin(), bubounds.size(), bubounds.begin());

        std::cout << "b2d_ubound: ";
        for (auto el : bubounds) {
            std::cout << el << " ";
        }
        std::cout << std::endl;
    } else {
        ROS_INFO("No param 'param/b2d_ubound' found in an upward search");
    }

    return true;
}

bool DDControllerROS::SetUpPublications(const ros::NodeHandle& n) {

    ros::NodeHandle nl(n);

    // Output Publications
    motor_ctrls_pub_ = nl.advertise<crazyflie_driver::PWM> (motor_ctrls_topic_.c_str(), 5);
    state_estimate_pub_= nl.advertise<dd_controller::StateEstimateStamped> (state_estimate_topic_.c_str(), 5);
    param_estimate_pub_ = nl.advertise<dd_controller::ParamEstimateStamped> (param_estimate_topic_.c_str(), 5);

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
    // The State aggregator knows the name of the area in which is tracking.
    // The following query will get the name of the sensors that are in the 
    // current area and that are providing information regarding the target
    // vehicle.
    //
    std::unordered_map<std::string, ros::master::TopicInfo> sdata;
    network_parser.query_sensors(sdata, area_name_, vehicle_name_);

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
 * Whenever I receive a new pose message.
 */
void DDControllerROS::onNewPose(const boost::shared_ptr<geometry_msgs::PoseStamped const>& msg, void* arg) {
    double dt;
    static timespec t_old;
    timespec t;

    state_t estim_state;
    DDParams param;

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
    
    // 1) Estimation Step
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
    estimator_ready_ = pddest_->Step();

    if (estimator_ready_) { // If state estimation is ready

        pddest_->GetState(&estim_state);
        // 3) Run the controller
        // If state and parameters are ok we can run the controller.

        double dT = pddest_->GetMeasuresTimeInterval();
        param = pddparest_->GetParams();

        if (active_) {
            pddctrl_->Step(&estim_state,  &param, dT / 3.0);

            pddctrl_->getControls(pwm_ctrls_); 

            // Initialization Procedure
            if (!controller_ready_ && pwm_ctrls_.norm() > 0.1) {
                pwm_ctrls_ << 1.0, 1.0, 1.0, 1.0; 
                if (--initialization_counter_ <= 0) {
                    controller_ready_ = true;
                }
            }
        } else {
            pwm_ctrls_ << 0, 0, 0, 0;
        }
    }

    // Pubblications
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


    dd_controller::ParamEstimateStamped par_msg;
    par_msg.header.stamp = msg->header.stamp;
    par_msg.alpha_x = param.alpha_x;
    par_msg.alpha_y = param.alpha_y;
    par_msg.beta_x = param.beta_x;
    par_msg.beta_y = param.beta_y;

    for (int i = 0; i < DDESTPAR_ALPHA2DSIZE; i++) {
        par_msg.alpha2d[i] = param.alpha2d(i);  
    }

    state_estimate_pub_.publish(est_msg);
    param_estimate_pub_.publish(par_msg);


    // Publish  Controls
    if (setpoint_type_ == "stop") {
        active_ = false;
        for (int i = 0; i < DDCTRL_OUTPUTSIZE; i++) {
            pwm_ctrls_(i) = 0.0;
        }
        crazyflie_driver::PWM pwm_msg;
        pwm_msg.header.stamp = ros::Time::now();

        pwm_msg.pwm0 = 0;
        pwm_msg.pwm1 = 0;
        pwm_msg.pwm2 = 0;
        pwm_msg.pwm3 = 0;

        motor_ctrls_pub_.publish(pwm_msg);
    }

    if (active_) {
        crazyflie_driver::PWM pwm_msg;
        pwm_msg.header.stamp = ros::Time::now();
        pwm_msg.pwm0 = pwm_ctrls_(0) * 60000;
        pwm_msg.pwm1 = pwm_ctrls_(1) * 60000;
        pwm_msg.pwm2 = pwm_ctrls_(2) * 60000;
        pwm_msg.pwm3 = pwm_ctrls_(3) * 60000;

        ctrl_counter_++;
        if (ctrl_counter_ % drop_mod_ == 0) {
            motor_ctrls_pub_.publish(pwm_msg);
        }
    }
    return;
}


void DDControllerROS::onNewControl(const crazyflie_driver::PWM::ConstPtr& msg) {
    //Parameter Estimation
    Eigen::Matrix<double, DDCTRL_OUTPUTSIZE, 1> curr_pwm;
    state_t estim_state;
    
    curr_pwm(0) = msg->pwm0 / 60000.0;
    curr_pwm(1) = msg->pwm1 / 60000.0;
    curr_pwm(2) = msg->pwm2 / 60000.0;
    curr_pwm(3) = msg->pwm3 / 60000.0;   

    if (estimator_ready_) { // If state estimation is ready
        pddest_->GetState(&estim_state);
        if (curr_pwm.norm() > 0.1) {
            double dT = pddest_->GetMeasuresTimeInterval();
            pddparest_->Step(&estim_state, curr_pwm, dT / 3.0);
        }
    }
        return;
}

// Process an incoming setpoint point change.
void DDControllerROS::onNewSetpoint(
        const testbed_msgs::ControlSetpoint::ConstPtr& msg) {

    setpoint_type_ = msg->setpoint_type; 

    setpoint_t ctrl_setpoint;
    ctrl_setpoint.position(0) = msg->p.x;
    ctrl_setpoint.position(1) = msg->p.y;
    ctrl_setpoint.position(2) = msg->p.z;
    ctrl_setpoint.velocity(0) = msg->v.x;
    ctrl_setpoint.velocity(1) = msg->v.y;
    ctrl_setpoint.velocity(2) = msg->v.z;
    ctrl_setpoint.acceleration(0) = msg->a.x;
    ctrl_setpoint.acceleration(1) = msg->a.y;
    ctrl_setpoint.acceleration(2) = msg->a.z;
    ctrl_setpoint.attitude(0) = msg->rpy.x;
    ctrl_setpoint.attitude(1) = msg->rpy.y;
    ctrl_setpoint.attitude(2) = msg->rpy.z;
    ctrl_setpoint.attitude_d(0) = msg->brates.x;
    ctrl_setpoint.attitude_d(1) = msg->brates.y;
    ctrl_setpoint.attitude_d(2) = msg->brates.z;

    // Copy the setpoint structure into the controller class
    pddctrl_->SetSetpoint(&ctrl_setpoint);

    // Set the flag about the setpoint
    active_ = true;
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

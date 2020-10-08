#include "simulator/simulator.hpp"
#include "testbed_msgs/CustOdometryStamped.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;

void sim_thread_fnc(void* p);

extern bool f1(vector<double>& x_,
        const vector<double>& x,
        const vector<double>& u, 
        double dt, void* param);

extern bool g1(std::vector<double>& y,
        const std::vector<double>& x,
        const std::vector<double>& u);

extern bool h1(std::vector<double>& z,
        const std::vector<double>& x,
        const std::vector<double>& u);


XSimulator::XSimulator() {
    initialized_ = false;
}

bool XSimulator::Initialize(const ros::NodeHandle& n) {

    ros::NodeHandle nl(n);

    name_ = ros::names::append(n.getNamespace(),
            "drone_sim");

    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }

    if (!RegisterCallbacks(n)) {
        ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
        return false;
    }

    sensor_pub_ = nl.advertise<testbed_msgs::CustOdometryStamped> (sim_sensor_topic_.c_str(), 10);

    codometry_pub_ = nl.advertise<testbed_msgs::CustOdometryStamped> (sim_state_topic_.c_str(), 10);

    vrpn_sim_pub_ = nl.advertise<geometry_msgs::PoseStamped> (vrpn_sim_pose_topic_.c_str(), 10);

    old_time_.sec = 0;
    old_time_.nsec = 0; 

    sim_ = new SimDyn(10, 4, 1, 0, &f1, &g1, &h1);

    initialized_ = true;

    arg_.period = 0.001;
    arg_.pParam = (void*) &parameters_;
    arg_.pSim = sim_;

    sim_thread = std::thread(sim_thread_fnc, (void*) &arg_);
    return true;
}

bool XSimulator::LoadParameters(const ros::NodeHandle& n) {
    ros::NodeHandle nl("~");

    if (!nl.getParam("frame_name", frame_name_))
        return false;

    nl.param<std::string>("topics/input_control_topic", ctrl_topic_, 
            frame_name_ + "/control");

    nl.param<std::string>("topics/output_sensor_topic", 
            sim_sensor_topic_, "xsim_sensors");

    nl.param<std::string>("topics/output_state_topic", 
            sim_state_topic_, "xsim_state");

    nl.param<std::string>("topics/output_simvrpn_topic", vrpn_sim_pose_topic_,
            "/area0/sensors/optitrack/" + frame_name_ + "/data");

    // Load Model Parameter of the drone
    nl.param<double>("param/drone_mass", Mass_, 1.0);
    nl.param<double>("param/lin_drag_coeff", c_drag_, 0.001);
    nl.param<double>("param/ang_drag_coeff", a_drag_, 0.001);
    nl.param<double>("param/sim_period", sim_period_, 0.005);

    parameters_.Mass = Mass_;
    parameters_.c_drag = c_drag_;
    parameters_.a_drag = a_drag_;

    return true;
}

bool XSimulator::RegisterCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl;

    // Subscribe to the control topic and link to the 
    // control callback routing.
    ctrl_topic_sub_ = nl.subscribe(
        ctrl_topic_.c_str(),
        2,
        &XSimulator::ControlCallback,
        this);

    return true;
}


/**
 * Callback from the controller
 * - Update the control data in the simulator structure
 */
void XSimulator::ControlCallback(
        const testbed_msgs::ControlStamped::ConstPtr& msg) {

    // The implementation of the flat controller returns an acceleration instead
    // of a thrust.
    ros::Time current_time = ros::Time::now();

    std::cout << "HO" << std::endl;

    if (old_time_.toSec() > 0) {
        double dt = current_time.toSec() - old_time_.toSec();
        old_time_ = current_time;
        vector<double> u_(4);
        u_[0] = msg->control.thrust * Mass_;
        u_[1] = msg->control.roll;
        u_[2] = msg->control.pitch;
        u_[3] = msg->control.yaw_dot;
        sim_->set_U(u_);
    } else {
        old_time_ = current_time;
    }
}

/**
 * In case I wanted to call the function from the ROS interface directly.
 */
void XSimulator::updateState(double dt) {
    sim_->sim_step(dt, (void*) &parameters_);
}


/**
 * Simulation thread
 *
 * The ROS class load the Arguments in a structure
 */
void sim_thread_fnc(void* p) {
   
    // Convert the pointer to pass information to the thread.
    simThread_arg* pArg = (simThread_arg*) p;

    double dt = pArg->period;
    void* pparam = pArg->pParam;
    SimDyn* psim = pArg->pSim;

    struct timespec time;
    struct timespec next_activation;
    
    struct timespec period_tms; 
    create_tspec(period_tms, dt);

    std::vector<double> x(10);
    x[6] = 1.0;
    psim->set_X(x);

    while (ros::ok()) {
        // Get current time
        clock_gettime(CLOCK_MONOTONIC, &time);
        timespec_sum(time, period_tms, next_activation);

        psim->sim_step(dt, pparam);
    
        clock_nanosleep(CLOCK_MONOTONIC,
                TIMER_ABSTIME, &next_activation, NULL);
    }
    ROS_INFO("Terminating Simulation Thread...\n");

}

void XSimulator::pub_thread_fnc(double dt) {   
    struct timespec time;
    struct timespec next_activation;
    
    struct timespec period_tms; 
    create_tspec(period_tms, dt);

    std::vector<double> x(10);
    testbed_msgs::CustOdometryStamped ext_codometry_msg;
    geometry_msgs::PoseStamped sim_vrpn_pose_msg;

    int counter = 0;
    while (ros::ok()) {
        // Get current time
        clock_gettime(CLOCK_MONOTONIC, &time);
        timespec_sum(time, period_tms, next_activation);
        //std::cout << next_activation.tv_nsec << std::endl;

        sim_->get_X(x);
    
        if (x[2] > 0)
            std::cout << x[0] << " " << x[1] << " " << x[2] << std::endl;
        //ext_codometry_msg_.header.stamp = msg->header.stamp;
        ext_codometry_msg.header.stamp = ros::Time::now();
        ext_codometry_msg.p.x = x[0];
        ext_codometry_msg.p.y = x[1];
        ext_codometry_msg.p.z = x[2];
        ext_codometry_msg.v.x = x[3];
        ext_codometry_msg.v.y = x[4];
        ext_codometry_msg.v.z = x[5];

        ext_codometry_msg.q.w = x[6];
        ext_codometry_msg.q.x = x[7];
        ext_codometry_msg.q.y = x[8];
        ext_codometry_msg.q.z = x[9];

        ext_codometry_msg.w.x = 0.0;
        ext_codometry_msg.w.y = 0.0;
        ext_codometry_msg.w.z = 0.0;

        if (counter++ % 5 == 0) {
            sim_vrpn_pose_msg.header.stamp = ext_codometry_msg.header.stamp;
            sim_vrpn_pose_msg.pose.position.x = ext_codometry_msg.p.x;
            sim_vrpn_pose_msg.pose.position.y = ext_codometry_msg.p.z;
            sim_vrpn_pose_msg.pose.position.z = -ext_codometry_msg.p.y;

            sim_vrpn_pose_msg.pose.orientation = ext_codometry_msg.q;
            sim_vrpn_pose_msg.pose.orientation.y = ext_codometry_msg.q.z;
            sim_vrpn_pose_msg.pose.orientation.z = -ext_codometry_msg.q.y;

            vrpn_sim_pub_.publish(sim_vrpn_pose_msg);
        }

        codometry_pub_.publish(ext_codometry_msg);
        sensor_pub_.publish(ext_codometry_msg);

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_activation, NULL);
    }
    ROS_INFO("Terminating Simulation Thread...\n");

}


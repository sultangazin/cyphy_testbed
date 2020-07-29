#include "state_aggregator/state_aggregator.hpp"
#include "math.h"


// HELPER FUNCTIONS
static bool healthy_vector(Eigen::Vector3d& v) {
    bool ret = true;
    for (int i = 0; i < 3; i++) {
        if (std::isnan(v(i))) {
            v = Eigen::Vector3d::Zero();
            ret = false;
            break;
        }
    }
    return ret;
}

static double time_diff(timespec t1, timespec t2) {
    long int nsec = (t1.tv_nsec - t2.tv_nsec);
    long int sec = (t1.tv_sec - t2.tv_sec);
    if (nsec < 0) {
        sec -= 1;
        nsec = 1e9 + nsec;
    } 		

    return (double)(sec + (double)nsec/1e9); 
}


static Quaterniond qsum(Quaterniond q1, Quaterniond q2) {
    Quaterniond res(Quaterniond::Identity());

    res.x() = q1.x() + q2.x();
    res.y() = q1.y() + q2.y();
    res.z() = q1.z() + q2.z();
    res.w() = q1.w() + q2.w();

    return res;
}

static Quaterniond qsm(Quaterniond q1, double a) {
    Quaterniond res(Quaterniond::Identity());

    res.x() = q1.x() * a;
    res.y() = q1.y() * a;
    res.z() = q1.z() * a; 
    res.w() = q1.w() * a;

    return res;
}

static Vector3d qd2w(Quaterniond q, Quaterniond qd) {
    Vector3d out;
    Matrix<double, 3, 4> M; 
    Vector4d v;
    v.block<3,1>(0,0) = qd.vec();
    v(3) = qd.w();
    M << -q.x(), q.w(), q.z(), -q.y(),
      -q.y(), -q.z(), q.w(), q.x(),
      -q.z(), q.y(), -q.x(), q.w();

    out = M * v;
    return out;
}


// =================================================================
// CLASS
//
StateAggregator::StateAggregator():
    received_reference_(false),
    last_state_time_(-1.0),
    initialized_(false) { 
    };

StateAggregator::~StateAggregator() {};

bool StateAggregator::Initialize(const ros::NodeHandle& n) {

    ros::NodeHandle nl(n);

    // Compose the name
    name_ = ros::this_node::getName().c_str();

    // Load parameters
    if (!LoadParameters(nl)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }

    // Register callback
    if (!RegisterCallbacks(nl)) {
        ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
        return false;
    }

    Eigen::Vector3d sigma_x(_sigmax, _sigmax, _sigmax);
    Eigen::Vector3d sigma_y(_sigmay, _sigmay, _sigmay);

    _pfilt = new PolyFilter(Eigen::Vector3d::Zero(),
            sigma_x, sigma_y, 0.03);

    // Advertise topics

    // External Orientation
    pose_rpy_pub_ =
        nl.advertise<geometry_msgs::Vector3Stamped> (ext_pose_rpy_topic_.c_str(), 10);

    // External Position
    ext_pos_pub_ = 
        nl.advertise<geometry_msgs::PointStamped> (ext_position_topic_.c_str(), 10);

    // External Pose
    pose_pub_ = 
        nl.advertise<geometry_msgs::PoseStamped> (ext_pose_topic_.c_str(), 10);

    //  
    odometry_pub_ =
        nl.advertise<nav_msgs::Odometry> (ext_odom_topic_.c_str(), 10); 
    codometry_pub_ =
        nl.advertise<testbed_msgs::CustOdometryStamped> (ext_codom_topic_.c_str(), 10);
    rs_pub_ =
        nl.advertise<testbed_msgs::CustOdometryStamped> (rs_codom_topic_.c_str(), 10);

    // Initialize the header refereces of odometry messages
    ext_odom_trans_.header.frame_id = "world";
    ext_odom_trans_.child_frame_id = object_name_;

    // Initialize the header part of the odometry topic message
    ext_odometry_msg_.header.frame_id = "world";
    ext_odometry_msg_.child_frame_id = object_name_;

    initialized_ = true;

    return true;
}

bool StateAggregator::LoadParameters(const ros::NodeHandle& n) {

    ros::NodeHandle np("~");

    // VRPN topic (Set as global)
    np.param<std::string>("topics/in_vrpn_topic", object_name_, "cf1");
    vrpn_topic_ = "/vrpn_client_node/" + object_name_ + "/pose"; 
   
    ROS_INFO("VRPN topic: %s!", vrpn_topic_.c_str());

    std::string key;
    if (np.searchParam("AxisUp", key)) {
        ROS_INFO("Found parameter %s!", key.c_str());
        n.getParam(key, axis_up_);
        ROS_INFO("Setting parameter %s = %s", 
                "AxisUp", axis_up_.c_str());
    } else {
        axis_up_ = "Z";
        ROS_INFO("No param 'AxisUp' found!");
        ROS_INFO("Setting default parameter %s = %s", 
                "AxisUp", axis_up_.c_str());
    }

    // External position (just position)
    np.param<std::string>("topics/out_ext_position_topic", ext_position_topic_, 
            "external_position");
    // External pose
    np.param<std::string>("topics/out_ext_pose_topic", ext_pose_topic_,
            "external_pose");
    // External orientation (rpy)
    np.param<std::string>("topics/out_ext_pose_rpy_topic", ext_pose_rpy_topic_, 
            "external_pose_rpy");
    // External odometry
    np.param<std::string>("topics/out_ext_odom_topic", ext_odom_topic_,
            "external_odom");

    np.param<std::string>("topics/out_ext_codom_topic", ext_codom_topic_,
		    "external_codom");

    np.param<std::string>("topics/out_rs_codom_topic", rs_codom_topic_,
		    "rs_codom");

    //    ROS_INFO("Namespace = %s", );
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
                "valpha", _sigmax);
    } 

    if (np.searchParam("sigmay", key)) {
        ROS_INFO("Found parameter %s!", key.c_str());
        n.getParam(key, _sigmay);
        ROS_INFO("Setting parameter %s = %f", 
                "sigmax", _sigmay);
    } else {
        _sigmay = 0.001;
        ROS_INFO("No param 'sigmax' found!"); 
        ROS_INFO("Setting default parameter %s = %f", 
                "valpha", _sigmay);
    }

    if (np.searchParam("time_delay", key)) {
        ROS_INFO("Found parameter %s!", key.c_str());
        n.getParam(key, t_delay_);
        ROS_INFO("Setting parameter %s = %f", 
                "time_delay", t_delay_);
    } else {
        t_delay_ = 0.0;
        ROS_INFO("No param 'time_delay' found!"); 
        ROS_INFO("Setting default parameter %s = %f", 
                "time_delay", t_delay_);
    }
    return true;
}



bool StateAggregator::RegisterCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);

    // Subscribe to the Optitrack topic
    inchannels["vrpn"] = nl.subscribe(vrpn_topic_.c_str(), 15, 
            &StateAggregator::onNewPose, this, 
            ros::TransportHints().tcpNoDelay());

    // Subscribe to the Gtrack topic
    inchannels["gtrack_nuc1"] = nl.subscribe(gtrack_topic_.c_str(), 5,
            &StateAggregator::onNewPosition, this,
            ros::TransportHints().tcpNoDelay());
    
    return true;
}


// CALLBACK ---------------------------------------------------------------------
/* 
 * Topic Callback
 * Whenever I receive a new Trajectory message, update the odometry.
 */
void StateAggregator::onNewPose(
        const boost::shared_ptr<geometry_msgs::PoseStamped const>& msg) {

    double dt;
    static timespec t_old;
    timespec t;

    // Take the time
    ros::Time current_time = ros::Time::now();

    if (axis_up_ == "Z") {
        p_(0) = msg->pose.position.x;	
        p_(1) = msg->pose.position.y;	
        p_(2) = msg->pose.position.z;	

        q_.x() = msg->pose.orientation.x;
        q_.y() = msg->pose.orientation.y;
        q_.z() = msg->pose.orientation.z;
        q_.w() = msg->pose.orientation.w;
    } else if (axis_up_ == "Y") {
        // Switch the axes to be compatible 
        // with the Aframe/Motive frames
        //  afr --> ros 
        //  R^(ros)_(afr) = Roll(pi/2) = 
        //  [1   0   0
        //   0   0  -1
        //   0   1   0]
        //
        p_(0) = msg->pose.position.x;	
        p_(1) = -msg->pose.position.z;	
        p_(2) = msg->pose.position.y;	

        q_.x() = msg->pose.orientation.x;
        q_.y() = -msg->pose.orientation.z;
        q_.z() = msg->pose.orientation.y;
        q_.w() = msg->pose.orientation.w;
    } else {
       ROS_ERROR("Error selecting the source reference frame"); 
    }
    // Read the timestamp of the message
    t.tv_sec = msg->header.stamp.sec;
    t.tv_nsec = msg->header.stamp.nsec;

    if (!received_reference_) {
        p_old_ = p_;
        q_old_= q_;
        t_old = t; 

        received_reference_ = true;
    } else {
        dt = time_diff(t, t_old); 

        _pfilt->prediction(dt);
        _pfilt->update(p_);

        p_ = _pfilt->getPos();
        v_ = _pfilt->getVel();
        
        if (!healthy_vector(vel_)) {
            ROS_ERROR("Detected NaN!");
        }

        // TODO: Filter the quaternion part
        qd_ = Eigen::Quaterniond::Identity();
        w_ = Eigen::Vector3d::Zero(); 

        p_old_ = p_;
        q_old_ = q_;
        t_old = t;
    }

    // Push forward the position and attitude, considering
    // the delay
    if (t_delay_ > 0.0) {
        p_pf_ = p_ + vel_ * t_delay_;
        //q_pf_ = qsum(q_, qsm(qd_, t_delay_));
        //q_pf_ = q_pf_.normalized();
        q_pf_ = q_;
    } else {
        p_pf_ = p_;
        q_pf_ = q_;
    }

    // Convert to euler
    //euler_ = q_pf_.toRotationMatrix().eulerAngles(0, 1, 2);

    euler_(0) =  atan2(2.0 * q_pf_.y() * q_pf_.z() + 2.0 * q_pf_.w() * q_pf_.x(), 
            1.0 - 2.0 * (q_pf_.x() * q_pf_.x() + q_pf_.y() * q_pf_.y()));
    euler_(1) = asin(2.0 * q_pf_.w()*q_pf_.y() - 2.0 * q_pf_.x()*q_pf_.z());
    euler_(2) =  atan2(2.0 * q_pf_.x() * q_pf_.y() + 2.0 * q_pf_.z() * q_pf_.w(),
            1.0 - 2.0 * (q_pf_.y() * q_pf_.y() + q_pf_.z() * q_pf_.z()));
    

    // Pose: Position + Orientation
    ext_pose_msg_.header.stamp = msg->header.stamp;
    //ext_pose_msg_.header.stamp = current_time;
    ext_pose_msg_.pose.position.x = p_pf_(0);
    ext_pose_msg_.pose.position.y = p_pf_(1);
    ext_pose_msg_.pose.position.z = p_pf_(2);

    ext_pose_msg_.pose.orientation.x = q_pf_.x();
    ext_pose_msg_.pose.orientation.y = q_pf_.y();
    ext_pose_msg_.pose.orientation.z = q_pf_.z();
    ext_pose_msg_.pose.orientation.w = q_pf_.w();

    // Position (directly from the camera)
    ext_position_msg_.header.stamp = msg->header.stamp;
    //ext_position_msg_.header.stamp = current_time;
    ext_position_msg_.point = ext_pose_msg_.pose.position;

    // Orientation RPY
    ext_pose_rpy_msg_.header.stamp = msg->header.stamp;
    //ext_pose_rpy_msg_.header.stamp = current_time;
    ext_pose_rpy_msg_.vector.x = euler_(0) * 180.0 / M_PI;
    ext_pose_rpy_msg_.vector.y = euler_(1) * 180.0 / M_PI;
    ext_pose_rpy_msg_.vector.z = euler_(2) * 180.0 / M_PI;

    // Odometry Topic
    //ext_odometry_msg_.header.stamp = msg->header.stamp;
    ext_odometry_msg_.header.stamp = current_time;
    ext_odometry_msg_.pose.pose.position = ext_pose_msg_.pose.position;
    ext_odometry_msg_.pose.pose.orientation = ext_pose_msg_.pose.orientation;
    ext_odometry_msg_.twist.twist.linear.x = vel_(0);
    ext_odometry_msg_.twist.twist.linear.y = vel_(1);
    ext_odometry_msg_.twist.twist.linear.z = vel_(2);
    ext_odometry_msg_.twist.twist.angular.x = w_(0);
    ext_odometry_msg_.twist.twist.angular.y = w_(1);
    ext_odometry_msg_.twist.twist.angular.z = w_(2);

	// Custom Odometry Topic
    //ext_codometry_msg_.header.stamp = msg->header.stamp;
    ext_codometry_msg_.header.stamp = current_time;
    ext_codometry_msg_.p = ext_pose_msg_.pose.position;
    ext_codometry_msg_.q = ext_pose_msg_.pose.orientation;
    ext_codometry_msg_.v.x = vel_(0);
    ext_codometry_msg_.v.y = vel_(1);
    ext_codometry_msg_.v.z = vel_(2);
    ext_codometry_msg_.w.x = w_(0);
    ext_codometry_msg_.w.y = w_(1);
    ext_codometry_msg_.w.z = w_(2);



    // Update Tranformation Message	
    //ext_odom_trans_.header.stamp = msg->header.stamp;
    ext_odom_trans_.header.stamp = current_time;
    // Position
    ext_odom_trans_.transform.translation.x = p_pf_(0);
    ext_odom_trans_.transform.translation.y = p_pf_(1);
    ext_odom_trans_.transform.translation.z = p_pf_(2);
    // Orientation
    ext_odom_trans_.transform.rotation = ext_pose_msg_.pose.orientation;

    // Send messages and transorm
    ext_odom_broadcaster_.sendTransform(ext_odom_trans_);

    ext_pos_pub_.publish(ext_position_msg_);
    pose_pub_.publish(ext_pose_msg_);
    pose_rpy_pub_.publish(ext_pose_rpy_msg_);
    odometry_pub_.publish(ext_odometry_msg_);
	codometry_pub_.publish(ext_codometry_msg_);

    testbed_msgs::CustOdometryStamped rs_odom_msg;
    rs_odom_msg.header.stamp = current_time;
    rs_odom_msg.p.x = p_(0);
    rs_odom_msg.p.y = p_(1);
    rs_odom_msg.p.z = p_(2);

    rs_odom_msg.v.x = v_(0);
    rs_odom_msg.v.y = v_(1);
    rs_odom_msg.v.z = v_(2);


    rs_pub_.publish(rs_odom_msg);

    return;
}

// Call back to get position messages
void StateAggregator::onNewPosition(
        const boost::shared_ptr<geometry_msgs::PointStamped const>& msg) {

    // Take the time
    ros::Time current_time = ros::Time::now();

    Eigen::Vector3d p;
    Eigen::Vector3d v;
    p(0) = msg->point.x;	
    p(1) = msg->point.y;	
    p(2) = msg->point.z;	

    _pfilt->update(p_);

    p = _pfilt->getPos();
    v = _pfilt->getVel();

    testbed_msgs::CustOdometryStamped rs_odom_msg;
    rs_odom_msg.header.stamp = current_time;
    rs_odom_msg.p.x = p(0);
    rs_odom_msg.p.y = p(1);
    rs_odom_msg.p.z = p(2);

    rs_odom_msg.v.x = v(0);
    rs_odom_msg.v.y = v(1);
    rs_odom_msg.v.z = v(2);


    rs_pub_.publish(rs_odom_msg);

    return;
} 


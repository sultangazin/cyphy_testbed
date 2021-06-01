#include "state_aggregator/state_aggregator.hpp"
#include "math.h"
#include <chrono>

#include "utilities/timeutils/timeutils.hpp"
#include "utilities/custom_conversion/custom_conversion.hpp"

#include "filter/polyfilter.hpp"

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
	node_ = n;
	ros::NodeHandle nl(n);

	// Compose the name
	name_ = ros::this_node::getName().c_str();

	// Load parameters
	if (!LoadParameters(nl)) {
		ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
		return false;
	}

	// Register callback
	if (!RegisterCallbacks()) {
		ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
		return false;
	}

	Eigen::Vector3d sigma_x(_sigmax, _sigmax, _sigmax);
	Eigen::Vector3d sigma_y(_sigmay, _sigmay, _sigmay);
	_pfilt = new PolyFilter(Eigen::Vector3d::Zero(), sigma_x, sigma_y, 0.002);


	// ======================================================================
	// Advertise topics

	// External Orientation
	pose_rpy_pub_ =
		nl.advertise<geometry_msgs::Vector3Stamped> (ext_att_rpy_topic_.c_str(), 10);
	// External Position
	ext_pos_pub_ = 
		nl.advertise<geometry_msgs::PointStamped> (ext_position_topic_.c_str(), 10);
	// External Pose
	pose_pub_ = 
		nl.advertise<geometry_msgs::PoseStamped> (ext_pose_topic_.c_str(), 10);
	odometry_pub_ =
		nl.advertise<testbed_msgs::CustOdometryStamped> (ext_odom_topic_.c_str(), 10);


	// ======================================================================
	// Advertise Services
	sensor_service = node_.advertiseService(
			"control_sensors", &StateAggregator::control_sensor, this);


	// ======================================================================
	// Start discovery thread 
	net_disc_thr = std::thread(&StateAggregator::net_discovery, this,
			500);

	initialized_ = true;
	return true;
}

bool StateAggregator::LoadParameters(const ros::NodeHandle& n) {

	ros::NodeHandle np("~");
	std::string key;

	np.param<std::string>("param/target_name", target_name_,"cf1");

	// External position (just position)
	np.param<std::string>("topics/out_ext_position_topic", ext_position_topic_, 
			"external_position");
	// External pose
	np.param<std::string>("topics/out_ext_pose_topic", ext_pose_topic_,
			"external_pose");
	// External orientation (rpy)
	np.param<std::string>("topics/out_ext_pose_rpy_topic", ext_att_rpy_topic_, 
			"external_pose_rpy");

	np.param<std::string>("topics/out_ext_codom_topic", ext_odom_topic_,
			"external_codom");

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
				"sigmax", _sigmax);
	} 

	if (np.searchParam("sigmay", key)) {
		ROS_INFO("Found parameter %s!", key.c_str());
		n.getParam(key, _sigmay);
		ROS_INFO("Setting parameter %s = %f", 
				"sigmax", _sigmay);
	} else {
		_sigmay = 0.001;
		ROS_INFO("No param 'sigmay' found!"); 
		ROS_INFO("Setting default parameter %s = %f", 
				"sigmay", _sigmay);
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

	area_name_ = "area0";

	return true;
}


int StateAggregator::UpdatePublishers() {

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

	for (auto el : sdata) {
		std::string sname = el.first; 
		if (inchannels_.count(sname) == 0) {
			std::cout << "[STATE AGGREGATOR] Connecting to Sensor: " <<
				sname << std::endl;
			TopicData str;
			str.topic_name = el.second.name;
			str.area_name = area_name_;
			str.node_name = sname;
			str.datatype = el.second.datatype;
			str.frequency = 0.0;
			str.isActive = true;
			str.enabled = true;
			inchannels_.insert(
					std::pair<std::string, TopicData>(
						sname, str)
					);
		}
	}
	return sdata.size();
}


bool StateAggregator::AssociateTopicsToCallbacks(const ros::NodeHandle& n) {
	ros::NodeHandle nl(n);
	for (auto el : inchannels_) { // For every registered channel
		std::string topic_name = el.second.topic_name;
		std::string topic_datatype = el.second.datatype;
		if (el.second.enabled) { // If the channel is not disabled
			if (active_subscriber.count(el.first) == 0) { // Associate the callback
				if (topic_datatype == "geometry_msgs/PoseStamped") {
					active_subscriber.insert(
							std::pair<std::string, ros::Subscriber>(
								el.first,
								nl.subscribe<geometry_msgs::PoseStamped>(
									topic_name.c_str(),
									5, 
									boost::bind(&StateAggregator::onNewPose, this, _1,
										(void*)&inchannels_[el.first].node_name),
									ros::VoidConstPtr(),
									ros::TransportHints().tcpNoDelay()
									)
								)
							);
				}
				if (topic_datatype == "geometry_msgs/PointStamped") {
					active_subscriber.insert(
							std::pair<std::string, ros::Subscriber>(
								el.first,
								nl.subscribe<geometry_msgs::PointStamped>(
									topic_name.c_str(),
									5, 
									boost::bind(&StateAggregator::onNewPosition, this, _1,
										(void*)&inchannels_[el.first].node_name),
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

bool StateAggregator::RegisterCallbacks() {

	// Update the list of sensors publications referring to the 
	// vehicle in the current area.
	UpdatePublishers();
	AssociateTopicsToCallbacks(node_); 

	return true;
}


// CALLBACK -----------------------------------------------------------------
/* 
 * Topic Callback
 * Whenever I receive a new Trajectory message, update the odometry.
 */
void StateAggregator::onNewPose(const boost::shared_ptr<geometry_msgs::PoseStamped const>& msg, void* arg) {
	double dt;
	static timespec t_old;
	timespec t;

	std::string node_name = *(std::string*) arg;

	// Take the time
	ros::Time current_time = msg->header.stamp;
	//ros::Time current_time = ros::Time::now();
	static ros::Time old_time {};

	p_(0) = msg->pose.position.x;	
	p_(1) = msg->pose.position.y;	
	p_(2) = msg->pose.position.z;	

	q_.x() = msg->pose.orientation.x;
	q_.y() = msg->pose.orientation.y;
	q_.z() = msg->pose.orientation.z;
	q_.w() = msg->pose.orientation.w;

	// Read the timestamp of the message
	t.tv_sec = msg->header.stamp.sec;
	t.tv_nsec = msg->header.stamp.nsec;

	if (!received_reference_) {
		_pfilt->resetPosition(p_);
		q_old_= q_;
		t_old = t; 
		received_reference_ = true;
	} else {
		dt = time_diff(t, t_old); 

		_pfilt->prediction(dt);
		_pfilt->update(p_);

		p_ = _pfilt->getPos();
		v_ = _pfilt->getVel();
		a_ = _pfilt->getAcc();

		if (!healthy_vector(v_)) {
			ROS_ERROR("Detected NaN!");
		}

		// TODO: Filter the quaternion part in a better way...
		static Eigen::Vector3d www = Eigen::Vector3d::Zero();
		if (dt > 0.001) {
			Eigen::Quaterniond qd_; 
			for (int i = 0; i < 4; i++) {
				qd_.coeffs()(i) = (q_.coeffs()(i) - q_old_.coeffs()(i)) / dt;
			}
			// Raw angular velocity in body frame from quaternion measurement
			Eigen::Quaterniond tempq = q_.inverse() * qd_;
			// A little piggy filtering...
			www = 0.5 * www + (1 - 0.5) * 2.0 * tempq.vec();
			w_ = www;
		}
		q_old_ = q_;
		t_old = t;
	}
	q_pf_ = q_;

	// Convert to euler
	//euler_ = q_pf_.toRotationMatrix().eulerAngles(0, 1, 2);
	euler_ = q2eul(q_pf_);

	// Pose: Position + Orientation
	//ext_pose_msg_.header.stamp = msg->header.stamp;
	ext_pose_msg_.header.stamp = current_time;
	ext_pose_msg_.pose.position.x = p_(0);
	ext_pose_msg_.pose.position.y = p_(1);
	ext_pose_msg_.pose.position.z = p_(2);

	ext_pose_msg_.pose.orientation.x = q_pf_.x();
	ext_pose_msg_.pose.orientation.y = q_pf_.y();
	ext_pose_msg_.pose.orientation.z = q_pf_.z();
	ext_pose_msg_.pose.orientation.w = q_pf_.w();

	// Position (directly from the camera)
	ext_position_msg_.header.stamp = msg->header.stamp;
	//ext_position_msg_.header.stamp = current_time;
	ext_position_msg_.point = ext_pose_msg_.pose.position;

	// Orientation RPY
	ext_att_rpy_msg_.header.stamp = msg->header.stamp;
	//ext_att_rpy_msg_.header.stamp = current_time;
	ext_att_rpy_msg_.vector.x = euler_(0) * 180.0 / M_PI;
	ext_att_rpy_msg_.vector.y = euler_(1) * 180.0 / M_PI;
	ext_att_rpy_msg_.vector.z = euler_(2) * 180.0 / M_PI;

	// Custom Odometry Topic
	//ext_odometry_msg_.header.stamp = msg->header.stamp;
	ext_odometry_msg_.header.stamp = current_time;

	ext_odometry_msg_.p = ext_pose_msg_.pose.position;

	ext_odometry_msg_.v.x = v_(0);
	ext_odometry_msg_.v.y = v_(1);
	ext_odometry_msg_.v.z = v_(2);

	ext_odometry_msg_.a.x = a_(0);
	ext_odometry_msg_.a.y = a_(1);
	ext_odometry_msg_.a.z = a_(2);

	ext_odometry_msg_.q = ext_pose_msg_.pose.orientation;
	ext_odometry_msg_.w.x = w_(0);
	ext_odometry_msg_.w.y = w_(1);
	ext_odometry_msg_.w.z = w_(2); 

	pose_pub_.publish(ext_pose_msg_);
	pose_rpy_pub_.publish(ext_att_rpy_msg_);
	odometry_pub_.publish(ext_odometry_msg_);
	ext_pos_pub_.publish(ext_position_msg_);

	return;
}


//
// Call back to get position messages
void StateAggregator::onNewPosition(const boost::shared_ptr<geometry_msgs::PointStamped const>& msg, void* arg) {

	// Take the time
	ros::Time current_time = ros::Time::now();
	static timespec told {}; 

	std::string node_name = *(std::string*) arg;

	Eigen::Vector3d p;
	Eigen::Vector3d v;

	p(0) = msg->point.x;	
	p(1) = msg->point.y;	
	p(2) = msg->point.z;	

	_pfilt->update(p);
	// Do something ...

	return;
} 


bool StateAggregator::control_sensor(
		state_aggregator::ControlSensor::Request& req,
		state_aggregator::ControlSensor::Response& res) {
	bool enabling = req.enable;
	std::cout << "Calling the service " << std::endl;
	std::string node_name = req.name;

	if (inchannels_.count(node_name) == 0) {
		std::cout << "No sensor registered with name '" <<
			node_name << "'" << std::endl;
		res.success = false;
		return false;
	}

	if (enabling) {
		std::cout << "[STATE_AGGREGATOR]: Enabling sensor " <<
			node_name << std::endl;
	} else {
		std::cout << "[STATE_AGGREGATOR]: Disabling sensor " <<
			node_name << std::endl;
	}

	inchannels_[node_name].enabled = enabling;
	res.success = true;
	return true;
}


void StateAggregator::net_discovery(int ms){
	while (ros::ok()) {
		RegisterCallbacks();
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}
}

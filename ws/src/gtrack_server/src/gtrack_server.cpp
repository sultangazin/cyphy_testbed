#include "gtrack_server/gtrack_server.hpp"
#include "testbed_msgs/CustPosVel.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>


GTrackServer::GTrackServer() :
    pserver(nullptr) {

        initialized_ = false;
}

GTrackServer::GTrackServer(int port) :
	pserver(new rpc::server(port)) {
	server_port_ = port;

    pserver->bind("new_data", [this](RpcData data) {
            onNewData(data);
            });
}


GTrackServer::~GTrackServer() {}

void GTrackServer::start() {
    if (pserver)
        pserver->async_run(2);
}


bool GTrackServer::Initialize(const ros::NodeHandle& n) {

    ros::NodeHandle nl(n);

    name_ = ros::names::append(n.getNamespace(),
            "gtrack_server");

    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }

    ext_pv_pub_ = nl.advertise<testbed_msgs::CustPosVel>
        (output_state_topic_.c_str(), 10);

    ext_p_pub_ = nl.advertise<geometry_msgs::PointStamped>
        (output_sensor_topic_.c_str(), 10);

    ext_arena_pub_ = nl.advertise<geometry_msgs::PoseStamped>
        (output_arena_topic_.c_str(), 10);


    initialized_ = true;

    pserver = new rpc::server(server_port_);
    pserver->bind("new_data", [this](RpcData data) {
            onNewData(data);
            });

    start();
    return true;
}


bool GTrackServer::LoadParameters(const ros::NodeHandle& n) {
    ros::NodeHandle nl("~");

    nl.param<std::string>("topics/output_state_topic", 
            output_state_topic_, "gtrack_state");

    nl.param<std::string>("topics/output_sensor_topic", 
            output_sensor_topic_, "gtrack_sensors");

    // I don't like this, but until we have a better Arena
    // interface I will play dirty.
    nl.param<std::string>("topics/output_arena_topic", 
            output_arena_topic_, "gtrack_arena/pose");

    nl.param<int>("param/server_port", server_port_, 8080);

    return true;
}


void GTrackServer::onNewData(RpcData data) {
    std::cout << "Received data" << std::endl;
    std::cout << "t = " << data.t << std::endl;
    std::cout << "position = [" << data.x << " " <<
        data.y << " " << data.z << "]" << std::endl;

    testbed_msgs::CustPosVel posvel_msg;
    geometry_msgs::PointStamped pos_msg;
    geometry_msgs::PoseStamped arena_msg;

    ros::Time current_time = ros::Time::now();

    posvel_msg.header.stamp = current_time;
    posvel_msg.p.x = data.x;
    posvel_msg.p.y = data.y;
    posvel_msg.p.z = data.z;

    posvel_msg.v.x = 0.0;
    posvel_msg.v.y = 0.0;
    posvel_msg.v.z = 0.0;

    pos_msg.header.stamp = current_time;
    pos_msg.point = posvel_msg.p;

    arena_msg.pose.position = pos_msg.point;

    ext_pv_pub_.publish(posvel_msg);
    ext_p_pub_.publish(pos_msg);
    ext_arena_pub_.publish(arena_msg);
}

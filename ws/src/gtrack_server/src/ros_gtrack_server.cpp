#include "rpc/this_handler.h"
#include "gtrack_server/ros_gtrack_server.hpp"
#include "testbed_msgs/CustPosVel.h"
#include <geometry_msgs/PointStamped.h>

#define _GTRACK_SERVER_DEBUG 


ROSGTrackServer::ROSGTrackServer() :
    pserver(nullptr) {
        initialized_ = false;
		server_port_ = 8080;
}

ROSGTrackServer::ROSGTrackServer(int port) :
    pserver(nullptr) {
        initialized_ = false;
		server_port_ = port;
}
void ROSGTrackServer::start() {
    if (pserver)
        pserver->async_run(2);
}

ROSGTrackServer::~ROSGTrackServer() {}

bool ROSGTrackServer::Initialize(const ros::NodeHandle& n) {

    ros::NodeHandle nl(n);

    name_ = ros::names::append(n.getNamespace(),
            "gtrack_server");

    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }

    ext_pv_pub_ = nl.advertise<testbed_msgs::CustPosVel>
        (output_state_topic_.c_str(), 10);

    ext_position_pub_ = nl.advertise<geometry_msgs::PointStamped>
        (output_sensor_topic_.c_str(), 10);


    input_feed_sub_ = nl.subscribe(
            "/cf3/external_position",
            5,
            &ROSGTrackServer::onNewPosition, this,
            ros::TransportHints().tcpNoDelay());

    pserver = new rpc::server(server_port_);
    
    /**
     * On the server side, the "send_data" is when the 
     * data arrives from clients.
     * The messages are pushed into the ROS stream with 
     * publications on topics.
     */
	pserver->bind("send_data", [this](RpcPointData data) {
            onNewData(data);
			// Disable the response
			rpc::this_handler().disable_response();
			return 0;
            });

    /**
     * Get data is the request from the client to get information.
     * The parameter "i" is not used.
     */
    pserver->bind("get_data", [this](int i){
            RpcPointData_v outdata;
            std::unordered_map<int, gatlas::TargetData> world_map = ga.getMap();
            for (auto el : world_map) {
                RpcPointData d;
                d.target_id = 3;
                d.xx = el.second.pos(1) + 0.115; 
                d.yy = -el.second.pos(0) + 0.14; 
                d.zz = el.second.pos(2);
                outdata.data.push_back(d);
            }
            //rpc::this_handler().respond(outdata);
            return outdata;
            });

	/**
     * Service to add Tranformation between atlas items.
     */
	pserver->bind("add_atlas_trf_data", [this](RpcGAtlasTrsfData data) {
			onNewTrfData(data);
			rpc::this_handler().disable_response();
			return 0;	
			});

	pserver->bind("get_atlas_trf_data", [this](int src, int dst) {
			bool success; 
			gatlas::TransformData trf {};	
			success = ga.getTransform(src, dst, trf);

			RpcGAtlasTrsfData outdata {};
            outdata.good = false;
			
			if (success) {
                outdata.good = true;
				outdata.origin = src;
				outdata.dest = dst;
				outdata.pos = std::vector<double>(3);
				outdata.quat = std::vector<double>(4);
				outdata.quat[0] = trf.rot.w();
				for (int i = 0; i < 3; i++) {
					outdata.pos[i] = trf.t(i);
					outdata.quat[i + 1] = trf.rot.vec()(i);
				}
			}

			return outdata;
			});

    start();

    initialized_ = true;
    return true;
}


bool ROSGTrackServer::LoadParameters(const ros::NodeHandle& n) {
    ros::NodeHandle nl("~");

    nl.param<std::string>("topics/output_state_topic", 
            output_state_topic_, "gtrack_state");

    nl.param<std::string>("topics/output_sensor_topic", 
            output_sensor_topic_, "/area0/sensors/gtrack/cf3/data");

    nl.param<int>("param/server_port", server_port_, 8080);

    return true;
}


void ROSGTrackServer::onNewData(RpcPointData data) {
    /*
    std::cout << "Received data" << std::endl;
    std::cout << "t = " << data.t << std::endl;
    std::cout << "position = [" << data.xx << " " <<
        data.yy << " " << data.zz << "]" << std::endl;
    */

    testbed_msgs::CustPosVel posvel_msg;
    geometry_msgs::PointStamped point_msg;

    ros::Time current_time = ros::Time::now();

    posvel_msg.header.stamp = current_time;
    posvel_msg.p.x = -data.yy + 0.14;
    posvel_msg.p.y = data.xx - 0.115;
    posvel_msg.p.z = data.zz;

    posvel_msg.v.x = 0.0;
    posvel_msg.v.y = 0.0;
    posvel_msg.v.z = 0.0;

    point_msg.header.stamp = current_time;
    point_msg.point = posvel_msg.p;

    ext_pv_pub_.publish(posvel_msg);
    ext_position_pub_.publish(point_msg);
}

void ROSGTrackServer::onNewPosition(
        const geometry_msgs::PointStampedConstPtr& msg) {

    ros::Time t = ros::Time::now();
    Eigen::Vector3d p(msg->point.x, msg->point.y, msg->point.z);
    ga.update_target_data(3,
            p,
            Eigen::Vector3d::Zero(),
            t.sec + t.nsec * 1e3);
}

void ROSGTrackServer::onNewTrfData(RpcGAtlasTrsfData data) {
	gatlas::TransformData tf;
	tf.rot.w() = data.quat[0];
	for (int i = 0; i < 3; i++) {
		tf.t(i) = data.pos[i];
		tf.rot.vec()(i) = data.quat[i + 1];
	}
	ga.setTransform(data.origin, data.dest, tf);
}

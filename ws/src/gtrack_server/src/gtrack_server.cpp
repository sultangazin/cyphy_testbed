#include "rpc/this_handler.h"
#include "gtrack_server/gtrack_server.hpp"
#include "testbed_msgs/CustPosVel.h"
#include <geometry_msgs/PointStamped.h>

GTrackServer::GTrackServer() :
    pserver(nullptr) {

        initialized_ = false;
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

    ext_position_pub_ = nl.advertise<geometry_msgs::PointStamped>
        (output_sensor_topic_.c_str(), 10);


    input_feed_sub_ = nl.subscribe(
            "/cf3/external_position",
            5,
            &GTrackServer::onNewPosition, this,
            ros::TransportHints().tcpNoDelay());


    pserver = new rpc::server(server_port_);
    
	pserver->bind("new_data", [this](RpcData data) {
            onNewData(data);
			// Disable the response
			rpc::this_handler().disable_response();
			return 0;
            });

    pserver->bind("synch", [this](RpcSynchData d) {
            rpcSynch(d);
            return true;
			});

    pserver->bind("get_data", [this](int i){
            //std::cout << "Answering [" << i << "]" <<
            //std::endl;
            RpcData_v outdata;
            mx.lock();
            for (auto el : world_map_) {
                RpcData d;
                d.id = 3;
                d.xx = el.second.yy + 0.115; 
                d.yy = -el.second.xx + 0.14; 
                d.zz = el.second.zz;
                outdata.data.push_back(d);
            }
            mx.unlock();
            //rpc::this_handler().respond(outdata);
            return outdata;
            });

	// Service to add Tranformation between atlas items
	pserver->bind("add_atlas_trf_data", [this](RpcAtlasTrsfData data) {
			onNewTrfData(data);
			rpc::this_handler().disable_response();
			return 0;	
			});

	pserver->bind("get_atlas_trf_data", [this](int src, int dst) {
			bool success; 
			TransformData trf {};	
			mx.lock();
			success = ga.getTransform(src, dst, trf);
			mx.unlock();

			RpcAtlasTrsfData outdata {};
			
			if (success) {
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


bool GTrackServer::LoadParameters(const ros::NodeHandle& n) {
    ros::NodeHandle nl("~");

    nl.param<std::string>("topics/output_state_topic", 
            output_state_topic_, "gtrack_state");

    nl.param<std::string>("topics/output_sensor_topic", 
            output_sensor_topic_, "/area0/sensors/gtrack/cf3/data");

    nl.param<int>("param/server_port", server_port_, 8080);

    return true;
}


void GTrackServer::rpcSynch(RpcSynchData d) {
	uint64_t sec = d.sec;
    uint64_t nsec = d.nsec;
	ros::Time current_time = ros::Time::now();

	long long int dsec_ns = (current_time.sec - sec) * 1e9;
	long long int dnsec = current_time.nsec - nsec;
	client_time_offset_ns = dsec_ns; 
	client_time_offset_ns += dnsec;
}

void GTrackServer::onNewData(RpcData data) {
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

void GTrackServer::onNewTrfData(RpcAtlasTrsfData data) {
	TransformData tf;
	tf.rot.w() = data.quat[0];
	for (int i = 0; i < 3; i++) {
		tf.t(i) = data.pos[i];
		tf.rot.vec()(i) = data.quat[i + 1]
	}
	ga.insert(data.origin, data.dest, tf);
}

void GTrackServer::onNewPosition(
        const geometry_msgs::PointStampedConstPtr& msg) {
    mx.lock();
    world_map_[3].id = 3;
    world_map_[3].xx = msg->point.x;
    world_map_[3].yy = msg->point.y;
    world_map_[3].zz = msg->point.z;
    mx.unlock();
}

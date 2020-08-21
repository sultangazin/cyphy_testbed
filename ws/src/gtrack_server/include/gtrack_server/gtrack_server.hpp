#include <ros/ros.h>
#include "rpc/server.h"
#include <iostream>
#include <string>
#include <time.h>
#include <mutex>

#include <unordered_map>
#include "gtrack_server_data.hpp"
#include <geometry_msgs/PointStamped.h>

class GTrackServer {
	public: 
        GTrackServer();
		//GTrackServer(int port);
		~GTrackServer();

		void onNewData(RpcData data);
		void rpcSynch(RpcSynchData d);

        void start();
        bool Initialize(const ros::NodeHandle& n);
		rpc::server* pserver;

	private:
        bool initialized_;
		int server_port_;

        std::mutex mx;

        std::unordered_map<int, RpcData> world_map_;

		int64_t client_time_offset_ns;

        ros::Publisher ext_pv_pub_;
        ros::Publisher ext_position_pub_;

        ros::Subscriber input_feed_sub_;

        std::string name_;

        bool LoadParameters(const ros::NodeHandle& n);
        void onNewPosition(const geometry_msgs::PointStampedConstPtr& m);

        std::string output_state_topic_;
        std::string output_sensor_topic_;
};

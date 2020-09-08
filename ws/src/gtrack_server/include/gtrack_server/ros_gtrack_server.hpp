#include <ros/ros.h>
#include "rpc/server.h"
#include <iostream>
#include <string>
#include <time.h>
#include <mutex>

#include <unordered_map>
#include "rpc_data.hpp"
#include "gatlas/gatlas.hpp"
#include <geometry_msgs/PointStamped.h>

class ROSGTrackServer {
	public: 
		ROSGTrackServer();
	    ROSGTrackServer(int port);
		~ROSGTrackServer();

		void start();
		bool Initialize(const ros::NodeHandle& n);

	private:
		bool initialized_;
		int server_port_;

        rpc::server* pserver;

        GAtlas ga;
        std::unordered_map<int, RpcPointData> world_map_;

		ros::Publisher ext_pv_pub_;
		ros::Publisher ext_position_pub_;

		ros::Subscriber input_feed_sub_;

		std::string name_;

		bool LoadParameters(const ros::NodeHandle& n);
		void onNewPosition(const geometry_msgs::PointStampedConstPtr& m);

		/**
		 * Takes new target data from the client
		 */
        void onNewTrfData(RpcGAtlasTrsfData data);
		void onNewData(RpcPointData data);

		std::string output_state_topic_;
		std::string output_sensor_topic_;
};

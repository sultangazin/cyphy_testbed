/**
 * Class NetworkParser
 * \file network_parser.hpp
 *
 * Class to manage ros topic and provide discover and filtering
 * capabilities.
 */

#ifndef NETWORK_PARSER_HPP
#define NETWORK_PARSER_HPP

#include <ros/master.h>
#include <string>
#include <unordered_map>

struct TopicData {
    std::string topic_name;
    std::string area_name;
    std::string node_name;
    std::string datatype;
    double frequency;
    bool isActive;
    bool enabled;
};


class NetworkParser {
    public:
        NetworkParser();
        NetworkParser(const NetworkParser& o);
        ~NetworkParser();

        NetworkParser& operator=(const NetworkParser&);

        int fetchActiveTopics(ros::master::V_TopicInfo& topic_infos);
        void printActiveTopics(); 

        int query_sensors(
                std::unordered_map<std::string, ros::master::TopicInfo>&
                sensors,
                const std::string& area_name = "",
                const std::string& agent_name = "");

		int query_controllers(
				std::unordered_map<std::string,
				ros::master::TopicInfo>& controllers,
				const std::string& area_name,
				const std::string& agent_name);
            
    private: 
        /**
         * Map of active topics
         * Index: Name of the topic
         * Value: Data type of the topic
         */
        std::unordered_map<std::string, std::string> active_topic_map;
};

#endif

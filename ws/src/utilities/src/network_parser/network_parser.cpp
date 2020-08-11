#include <iostream>
#include <regex>
#include "utilities/network_parser/network_parser.hpp"

NetworkParser::NetworkParser() {}

NetworkParser::NetworkParser(const NetworkParser& np) {
    active_topic_map = np.active_topic_map;
}

NetworkParser::~NetworkParser() {}

NetworkParser& NetworkParser::operator=(const NetworkParser& np) {
    active_topic_map = np.active_topic_map;

    return *this;
}

int NetworkParser::fetchActiveTopics(ros::master::V_TopicInfo& topic_infos) {
    // Fetch topic infos from the ROS master
    ros::master::getTopics(topic_infos);

    // Update the internal map
    for (auto el : topic_infos) {
        if (active_topic_map.count(el.name) == 0) {
            active_topic_map.insert(
                    std::pair<std::string, std::string> (
                        el.name, el.datatype)
                    );
        }
    }

    return topic_infos.size();
}

void NetworkParser::printActiveTopics() {
    std::cout << "====== ACTIVE TOPICS ======" << std::endl;
    int counter = 0;
    std::cout << "#\t" << "Name" << "\t" << "Type" << std::endl;
    for (auto el: active_topic_map) {
        std::cout << counter << ":\t"; 
        std::cout << el.first <<  "\t" << el.second << std::endl;
    }
    std::cout << "=========== END ===========" << std::endl;
}

int NetworkParser::query_sensors(
        std::unordered_map<std::string, ros::master::TopicInfo>& sensors,
        const std::string& area_name,
        const std::string& agent_name) {
    int counter = 0;

    std::string req_pattern{"^/"};   

    int index = 1;
    if(!area_name.empty()) {
        req_pattern.append(area_name);
    } else {
        req_pattern.append("(.*?)");
        index++;
    }
    req_pattern.append("/");
    req_pattern.append("sensors");
    req_pattern.append("/");
    req_pattern.append("(.*?)");
    req_pattern.append("/");
    if(!agent_name.empty()) {
        req_pattern.append(agent_name);
    } else {
        req_pattern.append("(.*?)");
    }
    req_pattern.append("/.*");

    //std::cout << "Parsing with pattern: " << req_pattern << std::endl;
    
    std::regex regex_pattern(req_pattern);
    std::smatch matches;

    ros::master::V_TopicInfo topics;
    fetchActiveTopics(topics);

    using namespace ros::master;
    for (auto el : topics) {
        bool res = regex_search(el.name, matches, regex_pattern);
        if (res) {
            sensors.insert(std::pair<std::string, TopicInfo>(
                        matches[index],
                        el 
                        )
                    );
            counter++;
        }
    }

    return counter;
}

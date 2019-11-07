#ifndef COMMANDER_INTERFACE_H
#define COMMANDER_INTERFACE_H

#include "ros/ros.h"

#include "commander_interface/TakeOff.h"
#include "commander_interface/Land.h"
#include "commander_interface/Track.h"
#include "commander_interface/GoTo.h"
#include "commander_interface/Impact.h"
#include "commander_interface/Flip.h"
#include "commander_interface/Stop.h"

// =================================================================
// CLASS
//

class CommanderInterface {
    public:
        CommanderInterface();
        ~CommanderInterface();

        bool Initialize(const ros::NodeHandle& n);

        bool takeoff_callback( 
                commander_interface::TakeOff::Request  &req,
                commander_interface::TakeOff::Response &res);
        bool land_callback(
                commander_interface::Land::Request  &req,
                commander_interface::Land::Response &res);

        bool goto_callback(
                commander_interface::GoTo::Request  &req,
                commander_interface::GoTo::Response &res);

        bool track_callback(
                commander_interface::Track::Request  &req,
                commander_interface::Track::Response &res);

        bool impact_callback(
                commander_interface::Impact::Request  &req,
                commander_interface::Impact::Response &res);

        bool flip_callback(
                commander_interface::Flip::Request &req,
                commander_interface::Flip::Response &res);

        bool stop_callback(
                commander_interface::Stop::Request &req,
                commander_interface::Stop::Response &res);

    private:

        // Load Parameters
        bool LoadParameters(const ros::NodeHandle& n);
        //                bool RegisterCallbacks(const ros::NodeHandle& n);

        // ROS variables
        //
        // Service Server
        ros::ServiceServer takeoff_srv_;
        ros::ServiceServer land_srv_;
        ros::ServiceServer goTo_srv_;
        ros::ServiceServer track_srv_;
        ros::ServiceServer impact_srv_;
        ros::ServiceServer flip_srv_;
        ros::ServiceServer stop_srv_;

        // Service Client
        ros::ServiceClient guidance_clnt_;


        // Callbacks


        // Names and topics
        std::string name_;
        std::string namespace_;

        bool initialized_;
};

#endif

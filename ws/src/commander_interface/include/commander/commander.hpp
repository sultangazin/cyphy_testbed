#ifndef COMMANDER_INTERFACE_H
#define COMMANDER_INTERFACE_H

#include <mutex>
#include "ros/ros.h"

#include <geometry_msgs/PointStamped.h>

#include "commander_interface/TakeOff.h"
#include "commander_interface/Land.h"
#include "commander_interface/Track.h"
#include "commander_interface/GoTo.h"
#include "commander_interface/Impact.h"
#include "commander_interface/Stop.h"
#include "commander_interface/OffBoard.h"

#include <guidance/GuidanceTargetAction.h>
#include <actionlib/client/simple_action_client.h>

enum class AgentState {stop, hovering, moving};
typedef actionlib::SimpleActionClient<guidance::GuidanceTargetAction> ActionClient;
typedef actionlib::SimpleClientGoalState GoalState;

#define AbortState (actionlib::SimpleClientGoalState::ABORTED)
#define PreemptedState (actionlib::SimpleClientGoalState::PREEMPTED)
#define SucceededState (actionlib::SimpleClientGoalState::SUCCEEDED)
#define ActiveState (actionlib::SimpleClientGoalState::ACTIVE)
#define LostState (actionlib::SimpleClientGoalState::LOST)


// =================================================================
// CLASS
//
class CommanderInterface {
    public:
        CommanderInterface();
        ~CommanderInterface();

        bool Initialize(const ros::NodeHandle& n);

        // Request takeoff
        bool takeoff_callback( 
                commander_interface::TakeOff::Request  &req,
                commander_interface::TakeOff::Response &res);

        // Request Landing
        bool land_callback(
                commander_interface::Land::Request  &req,
                commander_interface::Land::Response &res);

        // Request GoTo
        bool goto_callback(
                commander_interface::GoTo::Request  &req,
                commander_interface::GoTo::Response &res);

        // Switch controller
        bool ctrl_offboard_callback(
                commander_interface::OffBoard::Request  &req,
                commander_interface::OffBoard::Response &res);

        /*
           bool impact_callback(
           commander_interface::Impact::Request  &req,
           commander_interface::Impact::Response &res);
        */

        bool stop_callback(
                commander_interface::Stop::Request &req,
                commander_interface::Stop::Response &res);

    private:
        // State
        AgentState agent_state_;

        std::mutex mx;
        boost::array<float, 3> curr_planning;
        boost::array<float, 3> current_position_;

        // Load Parameters
        bool LoadParameters(const ros::NodeHandle& n);
        //                bool RegisterCallbacks(const ros::NodeHandle& n);

        // Service Server
        ros::ServiceServer takeoff_srv_;
        ros::ServiceServer land_srv_;
        ros::ServiceServer goTo_srv_;
        ros::ServiceServer ctrl_offboard_srv_;

        ros::ServiceServer stop_srv_;

        //ros::ServiceServer track_srv_;
        //ros::ServiceServer impact_srv_;
        //ros::ServiceServer flip_srv_;

        // Service Client
        ActionClient* pactc_;

        ros::ServiceClient cf_ros_goto_clnt_;
        ros::ServiceClient cf_ros_takeoff_clnt_;
        ros::ServiceClient cf_ros_land_clnt_;
        ros::ServiceClient cf_ros_stop_clnt_;

        ros::ServiceClient control_router_switch_client_;

        // Topic subscription
        ros::Subscriber vehicle_pos_sub_;
        
        // Topics callbacks
        void VehiclePositionCallback(
                const geometry_msgs::PointStamped::ConstPtr& msg);

        
        // General callback for the action client
        void action_feedback_cb(
                const guidance::GuidanceTargetFeedbackConstPtr& feedback);
        void action_done_cb(
                const actionlib::SimpleClientGoalState& state,
                const guidance::GuidanceTargetResultConstPtr& result);

        // Thread running the action client request
        void action_thread(guidance::GuidanceTargetGoal goal);

        // Names and topics
        std::string name_;
        std::string vehicle_name_;
        std::string vehicle_pos_topic_;
        std::string namespace_;

        bool off_board_controller_;

        bool initialized_;
};

#endif

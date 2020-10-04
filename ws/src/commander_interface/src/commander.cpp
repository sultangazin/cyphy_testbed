/** @file commander.cpp
 *  @author l.pannocchi@gmail.com
 *
 */
#include "commander/commander.hpp"
#include "crazyflie_driver/GoTo.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/Stop.h"
#include "crazyflie_driver/Takeoff.h"

#include <thread>

// =================================================================
// =================================================================
CommanderInterface::CommanderInterface() :
    takeoff_srv_(), land_srv_(), goTo_srv_(){
        agent_state_ = AgentState::stop;

        off_board_controller_ = false;
        initialized_ = false;
    }

CommanderInterface::~CommanderInterface() {
    return;
}


bool CommanderInterface::LoadParameters(const ros::NodeHandle& n) {

    ros::NodeHandle np("~");
    np.param<std::string>("param/vehicle_name", vehicle_name_,"cf1");

    np.param<std::string>("topics/vehicle_pos", vehicle_pos_topic_, "/cf1/external_position");
    return true;
}

/**
 * Initialization function
 */
bool CommanderInterface::Initialize(const ros::NodeHandle& n) {
    // Get the name of the node
    name_ = ros::this_node::getName();
    namespace_ = ros::this_node::getNamespace();

    // Load Parameters
    if (!LoadParameters(n)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }

    // I want to advertise service in the node namespace
    ros::NodeHandle nh("~");
    ros::NodeHandle ng(n);

    vehicle_pos_sub_ = ng.subscribe(vehicle_pos_topic_.c_str(), 1, &CommanderInterface::VehiclePositionCallback, this);

    // Advertise topics/Services
    takeoff_srv_ = nh.advertiseService("takeoff_srv", 
            &CommanderInterface::takeoff_callback, this);

    land_srv_ = nh.advertiseService("land_srv", 
            &CommanderInterface::land_callback, this);

    goTo_srv_ = nh.advertiseService("goTo_srv",
            &CommanderInterface::goto_callback, this);

    ctrl_offboard_srv_ = nh.advertiseService("ctrl_offboard_srv",
            &CommanderInterface::ctrl_offboard_callback, this);

    /*
       track_srv_ = nh.advertiseService("track_srv",
       &CommanderInterface::track_callback, this);

       impact_srv_ = nh.advertiseService("impact_srv",
       &CommanderInterface::impact_callback, this);

       stop_srv_ = nh.advertiseService("stop_srv",
       &CommanderInterface::stop_callback, this);
       */

    // Connect to the service provided by the guidance node.
    // That node will create the guidance (produce reference points) for accomplishing the task

    // Connect to the services of the crazyflie_ros
    cf_ros_goto_clnt_ = ng.serviceClient<crazyflie_driver::GoTo>(
            "go_to");

    cf_ros_takeoff_clnt_ = ng.serviceClient<crazyflie_driver::Takeoff>(
            "takeoff");

    cf_ros_land_clnt_ = ng.serviceClient<crazyflie_driver::Land>(
            "land");

    cf_ros_stop_clnt_ = ng.serviceClient<crazyflie_driver::Stop>(
            "stop");

    std::string guidance_server = "AAAA";
    pactc_ = new ActionClient(guidance_server, true);

    ROS_INFO("%s: Waiting for Guidance Server [%s]...\n", name_.c_str(), guidance_server.c_str());

    pactc_->waitForServer(); 

    ROS_INFO("%s: Guidance Server Replied!\n", name_.c_str());


    initialized_ = true; 

    return true;
}


// Service Callbacks
bool CommanderInterface::takeoff_callback(
        commander_interface::TakeOff::Request  &req,
        commander_interface::TakeOff::Response &res) {

    bool output = false;

    ROS_INFO("%s: Takeoff requested! \n \t Height: %.3f | Duration: %.3f", 
            name_.c_str(), req.height, req.duration);

    if (off_board_controller_) {
        // Prepare the action 
        guidance::GuidanceTargetGoal goal;
        boost::array<float, 3> v{{0.0, 0.0, 0.0}};

        goal.mission_type = "takeoff";
        goal.target_v = v;
        goal.target_a = v;

        v[2] = req.height;
        goal.target_p = v; 

        goal.tg_time = req.duration;

        // Create a thread to call the action and monitor the outcome.
        std::thread run_action(&CommanderInterface::action_thread, this, goal);
        run_action.detach();

        res.ack = "Roger!";
        output = true;
    } else {
        crazyflie_driver::Takeoff srv;
        srv.request.height = req.height;
        srv.request.duration = ros::Duration(req.duration);
        srv.request.groupMask = 0;

        if (cf_ros_takeoff_clnt_.call(srv) == true) {
            res.ack = "Roger!";
            output = true;
        }
        else {
            res.ack = "Fail!";
            output = false;
        }
    }

    return output;
}


bool CommanderInterface::land_callback(
        commander_interface::Land::Request  &req,
        commander_interface::Land::Response &res) {
    bool output = false;

    if (off_board_controller_) {
        guidance::GuidanceTargetGoal goal;

        boost::array<float, 3> v = req.target_p;

        // Relative request to the current point
        goal.mission_type = "land";
        goal.target_p = v; 
        goal.target_a = v;
        goal.tg_time = req.duration;

        // Create a thread to call the action and monitor the outcome.
        std::thread run_action(&CommanderInterface::action_thread, this, goal);
        run_action.detach();

        res.ack = "Roger!";
        output = true;
    } else {
        crazyflie_driver::Land srv;
        srv.request.height = req.target_p[2];
        srv.request.duration = ros::Duration(req.duration);
        srv.request.groupMask = 0;

        if (cf_ros_land_clnt_.call(srv) == true) {
            res.ack = "Roger!";
            output = true;
        }
        else {
            res.ack = "Fail!";
            output = false;
        }
    }
    return true;
}

bool CommanderInterface::goto_callback(
        commander_interface::GoTo::Request  &req,
        commander_interface::GoTo::Response &res) {
    bool output = false;

    if (off_board_controller_) {
        guidance::GuidanceTargetGoal goal;

        boost::array<float, 3> v{{0.0, 0.0, 0.0}};

        goal.mission_type = "goTo";
        goal.target_v = v;
        goal.target_a = v;

        v = req.target_p;
        goal.target_p = v; 

        goal.tg_time = req.duration;

        // Create a thread to call the action and monitor the outcome.
        std::thread run_action(&CommanderInterface::action_thread, this, goal);
        run_action.detach();

        res.ack = "Roger!";
        output = true;
    } else {
        crazyflie_driver::GoTo srv;
        srv.request.goal.x = req.target_p[0];
        srv.request.goal.y = req.target_p[1];
        srv.request.goal.z = req.target_p[2];
        srv.request.yaw = 0;
        srv.request.duration = ros::Duration(req.duration);
        srv.request.relative = false;
        srv.request.groupMask = 0;

        if (cf_ros_goto_clnt_.call(srv) == true) {
            res.ack = "Roger!";
            output = true;
        } else {
            res.ack = "Fail!";
            output = false;
        }
    }
    return output;
}


bool CommanderInterface::ctrl_offboard_callback (
        commander_interface::OffBoard::Request  &req,
        commander_interface::OffBoard::Response &res) {
    
    bool able_to_switch = true;

    // If coming from another state ask the new controller to stay in the last position 
    if ((req.offboard_active != off_board_controller_) && (agent_state_ != AgentState::stop)) {

        if (req.offboard_active) {
            guidance::GuidanceTargetGoal goal;

            goal.mission_type = "goTo";
            boost::array<float, 3> v{{0.0, 0.0, 0.0}};
            goal.target_v = v;
            goal.target_a = v;
            goal.target_p = current_position_; 
            goal.tg_time = 2.0;

            // Create a thread to call the action and monitor the outcome.
            std::thread run_action(&CommanderInterface::action_thread, this, goal);
            run_action.detach();
            able_to_switch = true;
        } else {
            crazyflie_driver::GoTo srv;
            srv.request.goal.x = current_position_[0];
            srv.request.goal.y = current_position_[1];
            srv.request.goal.z = current_position_[2];
            srv.request.yaw = 0;
            srv.request.duration = ros::Duration(2.0);
            srv.request.relative = false;
            srv.request.groupMask = 0;

            if (cf_ros_goto_clnt_.call(srv) == true) {
                able_to_switch = true;
            } else {
                able_to_switch = false;
            }
        }

    }


    if (req.offboard_active && able_to_switch) {
        std::cout << "Running with External Controller" << std::endl;
        off_board_controller_ = true;
    } else {
        std::cout << "Running with Internal Controller" << std::endl;
        off_board_controller_ = false;
    }

    res.status = off_board_controller_;

    return able_to_switch;
}


/*
   bool CommanderInterface::impact_callback(
   commander_interface::Impact::Request  &req,
   commander_interface::Impact::Response &res) {

   guidance::ExeGuidanceTarget srv;

   boost::array<float, 3> v{{0.0, 0.0, 0.0}};

// Relative request to the current point
srv.request.mission_type = "impact";
srv.request.target_p = v; 
srv.request.target_a = v;

srv.request.tg_time = req.duration;

if (guidance_clnt_.call(srv))
res.ack = "Roger!";
else
res.ack = "Fail!";

return true;
}
*/


bool CommanderInterface::stop_callback(
        commander_interface::Stop::Request  &req,
        commander_interface::Stop::Response &res) {
    bool output = false;
   
    crazyflie_driver::Stop srv;
    srv.request.groupMask = 0;

    if (cf_ros_stop_clnt_.call(srv) == true) {
        res.ack = "Roger!";
        output = true;
    } else {
        res.ack = "Fail!";
        output = false;
    }

    return output;
}


void CommanderInterface::VehiclePositionCallback(
        const geometry_msgs::PointStamped::ConstPtr& msg) {
    
    current_position_[0] = msg->point.x;
    current_position_[1] = msg->point.y;
    current_position_[2] = msg->point.z;
}


void CommanderInterface::action_thread(guidance::GuidanceTargetGoal goal){
    guidance::GuidanceTargetResult result;

    std::string goal_type = goal.mission_type;

    mx.lock();
    if (pactc_->getState() == ActiveState) {
        pactc_->cancelAllGoals();
    }

    pactc_->sendGoal(goal,
            boost::bind(&CommanderInterface::action_done_cb, this, _1, _2),
            ActionClient::SimpleActiveCallback(),
            boost::bind(&CommanderInterface::action_feedback_cb, this, _1)
            );
    mx.unlock();

    /*
    std::cout << "[" << std::this_thread::get_id() << "] Waiting for result..." << std::endl;
    pactc_->waitForResult();

    GoalState curr_goal_state = pactc_->getState(); 

    if (curr_goal_state == AbortState || curr_goal_state == PreemptedState) {
        std::cout << "[" << std::this_thread::get_id() << "]" <<
            "[" << goal_type << "] Aborted!" << std::endl;
        return;
    }

    if (curr_goal_state == SucceededState) {
        std::cout << "[" << std::this_thread::get_id() << "]" <<
            "[" << goal_type << "] Success!" << std::endl;
        if (goal.mission_type == "goTo" || goal.mission_type == "takeoff") {
            agent_state_ = AgentState::hovering;
        }
    }
    */

    return;
}

void CommanderInterface::action_feedback_cb(
        const guidance::GuidanceTargetFeedbackConstPtr& feedback) {
    curr_planning = feedback->curr_pos;
    agent_state_ = AgentState::moving;
}

void CommanderInterface::action_done_cb(
        const actionlib::SimpleClientGoalState& state,
        const guidance::GuidanceTargetResultConstPtr& result) {

    ROS_INFO("Trajectory Generation finished in state [%s]", state.toString().c_str());

    std::string goal_type = result->mission_type;
    GoalState curr_goal_state = pactc_->getState(); 

    if (curr_goal_state == AbortState || curr_goal_state == PreemptedState) {
        std::cout << "[" << std::this_thread::get_id() << "]" <<
            "[" << goal_type << "] Aborted!" << std::endl;
        return;
    }

    if (curr_goal_state == SucceededState) {
        std::cout << "[" << std::this_thread::get_id() << "]" << "[" << goal_type << "] Success!" << std::endl;
        if (goal_type == "goTo" || goal_type == "takeoff") {
            agent_state_ = AgentState::hovering;
        }
        
        if (goal_type == "land") {
            agent_state_ = AgentState::stop;
            crazyflie_driver::Stop srv;
            srv.request.groupMask = 0;

            cf_ros_stop_clnt_.call(srv);
        }
    }
}

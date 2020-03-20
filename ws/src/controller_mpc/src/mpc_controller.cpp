#include <ros/ros.h>
#include "mpc_controller.h"
#include <math.h>
#include "math3d.h"
#include <stdio.h>
#include <Eigen/Geometry>

#define GRAVITY_MAGNITUDE (9.81f)

namespace controller_mpc {

    // Initialize.
    bool MPCController::Initialize(const ros::NodeHandle& n) {
        name_ = ros::names::append(n.getNamespace(), "controller_mpc");

        if (!LoadParameters(n)) {
            ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
            return false;
        }

        std::cout << "Loaded Parameters";
        if (!RegisterCallbacks(n)) {
            ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
            return false;
        }
        std::cout << "Registered callbacks" << std::endl;
        // // Load K, x_ref, u_ref from disk.
        // if (!LoadFromDisk()) {
        //   ROS_ERROR("%s: Failed to load K, x_ref, u_ref from disk.", name_.c_str());
        //   return false;
        // }

        // Set up control publisher.
        ros::NodeHandle nl(n);
        //pub_predicted_trajectory_ = nl.advertise<nav_msgs::Path>(topic, 1);
        control_pub_ = nl.advertise<testbed_msgs::ControlStamped>(
                control_topic_.c_str(), 1, false);

        error_pub_ = nl.advertise<testbed_msgs::CtrlPerfStamped>(
                ctrl_perf_topic_.c_str(), 1, false);

        predicted_traj_pub_ = nl.advertise<nav_msgs::Path>(
                predicted_traj_topic_.c_str(), 1, false);

        Reset();

        setNewParams();

        std::cout << "Set new params" << std::endl;

        solve_from_scratch_ = true;

        preparation_thread_ = std::thread(&MpcWrapper<double>::prepare, mpc_wrapper_);

        if(ctrl_mode_ != ControlMode::RATES){
            ROS_ERROR("%s: Please set the control mode in rates.", name_.c_str());
            return false;
        }
        // TODO: Change the hardcoded drone parameter
        trajectory_clnt_ = nl.serviceClient<guidance::MPC_RefWindow>("/cf2/get_pred_window");

        run_thread_ = std::thread(&MPCController::run, this);

        initialized_ = true;

        std::cout << "Initialized" << std::endl;

        return true;
    }

    // Load parameters. This may be overridden by derived classes.
    bool MPCController::LoadParameters(const ros::NodeHandle& n) {
        ros::NodeHandle nl(n);

        // Read state costs
        double Q_pos_xy, Q_pos_z, Q_attitude, Q_velocity, Q_perception;
        if (!nl.getParam("Q_pos_xy", Q_pos_xy)) return false;
        if (!nl.getParam("Q_pos_z", Q_pos_z)) return false;
        if (!nl.getParam("Q_attitude", Q_attitude)) return false;
        if (!nl.getParam("Q_velocity", Q_velocity)) return false;
        if (!nl.getParam("Q_perception", Q_perception)) return false;

        // Check whether all state costs are positive.
        if(Q_pos_xy     <= 0.0 ||
            Q_pos_z      <= 0.0 ||
            Q_attitude   <= 0.0 ||
            Q_velocity   <= 0.0 ||
            Q_perception < 0.0)      // Perception cost can be zero to deactivate.
        {
            ROS_ERROR("MPC: State cost Q has negative enries!");
            return false;
        }

        // Read input costs.
        double R_thrust, R_pitchroll, R_yaw;
        if (!nl.getParam("R_thrust", R_thrust)) return false;
        if (!nl.getParam("R_pitchroll", R_pitchroll)) return false;
        if (!nl.getParam("R_yaw", R_yaw)) return false;

        // Check whether all input costs are positive.
        if(R_thrust  <= 0.0 ||
            R_pitchroll <= 0.0 ||
            R_yaw       <= 0.0)
        {
            ROS_ERROR("MPC: Input cost R has negative enries!");
            return false;
        }

        // Set state and input cost matrices.
        Q_ = (Eigen::Matrix<double, kCostSize, 1>() <<
            Q_pos_xy, Q_pos_xy, Q_pos_z,
            Q_attitude, Q_attitude, Q_attitude, Q_attitude,
            Q_velocity, Q_velocity, Q_velocity,
            Q_perception, Q_perception).finished().asDiagonal();
        R_ = (Eigen::Matrix<double, kInputSize, 1>() <<
            R_thrust, R_pitchroll, R_pitchroll, R_yaw).finished().asDiagonal();

        // Read cost scaling values
        if (!nl.getParam("state_cost_exponential", state_cost_exponential_)) return false;
        if (!nl.getParam("input_cost_exponential", input_cost_exponential_)) return false;

        // Read input limits.
        if (!nl.getParam("max_bodyrate_xy", max_bodyrate_xy_)) return false;
        if (!nl.getParam("max_bodyrate_z", max_bodyrate_z_)) return false;
        if (!nl.getParam("min_thrust", min_thrust_)) return false;
        if (!nl.getParam("max_thrust", max_thrust_)) return false;

         // Check whether all input limits are positive.
        if(max_bodyrate_xy_ <= 0.0 ||
            max_bodyrate_z_  <= 0.0 ||
            min_thrust_      <= 0.0 ||
            max_thrust_      <= 0.0)
        {
            ROS_ERROR("MPC: All limits must be positive non-zero values!");
            return false;
        }

        // Optional parameters
        std::vector<double> p_B_C(3), q_B_C(4);
        if(!nl.getParam("p_B_C", p_B_C))
        {
            ROS_WARN("MPC: Camera extrinsic translation is not set.");
        }
        else
        {
            p_B_C_ = Eigen::Matrix<double, 3, 1>(p_B_C[0], p_B_C[1], p_B_C[2]);
        }
        if(!nl.getParam("q_B_C", q_B_C))
        {
            ROS_WARN("MPC: Camera extrinsic rotation is not set.");
        }
        else
        {
            q_B_C_ = Eigen::Quaternion<double>(q_B_C[0], q_B_C[1], q_B_C[2], q_B_C[3]);
        }

        if (!nl.getParam("print_info", print_info_)) return false;
        if(print_info_) ROS_INFO("MPC: Informative printing enabled.");        

        // Topics.
        if (!nl.getParam("topics/state", state_topic_)) return false;
        std::cout << "State topic" <<std::endl;

        //if (!nl.getParam("topics/setpoint", setpoint_topic_)) return false;
        //if (!nl.getParam("topics/reference", reference_topic_)) return false;
        if (!nl.getParam("topics/control", control_topic_)) return false;
        std::cout << "Control topic" <<std::endl;


        if (!nl.getParam("topics/ctrl_perf", ctrl_perf_topic_)) return false;
        std::cout << "Perf topic" <<std::endl;

        if (!nl.getParam("topics/predicted_traj", predicted_traj_topic_)) return false;
        std::cout << "Perf topic" <<std::endl;

        // Control Mode
        if (!nl.getParam("control_mode", (int&)ctrl_mode_)) return false;
        std::cout << "Control mode topic" <<std::endl;

        return true;
    }

    // Register callbacks.
    bool MPCController::RegisterCallbacks(const ros::NodeHandle& n) {
        ros::NodeHandle nl(n);

        // Subscribers.
        state_sub_ = nl.subscribe(state_topic_.c_str(), 1, &MPCController::StateCallback, this);
        return true;
    }

    // Reset variables.
    void MPCController::Reset(void)
    {
        sp_pos_ = Vector3d::Zero();
        sp_vel_ = Vector3d::Zero();
        sp_acc_ = Vector3d::Zero();
        sp_r_pos_ = Vector3d::Zero();
        sp_r_vel_ = Vector3d::Zero();
        sp_r_acc_ = Vector3d::Zero();
        sp_brates_ = Vector3d::Zero(); 

        setpoint_type_ = "FullTrj";

        pos_ = Vector3d::Zero();
        vel_ = Vector3d::Zero();
        r_pos_ = Vector3d::Zero();
        r_vel_ = Vector3d::Zero();

        quat_.vec() = Vector3d::Zero();
        quat_.w() = 0;

        received_setpoint_ = false;
    }

    // Passes the cost matrices to mpc_wrapper.
    bool MPCController::setNewParams() {
        mpc_wrapper_.setCosts(Q_,R_);
        mpc_wrapper_.setLimits(
            min_thrust_, max_thrust_,
            max_bodyrate_xy_, max_bodyrate_z_);
        // TODO: remove camera stuff
        mpc_wrapper_.setCameraParameters(p_B_C_, q_B_C_);
        changed_ = false;
        return true;
    }

    // Convert from class variables to Eigen format.
    bool MPCController::setStateEstimate(){
        est_state_(kPosX) = pos_(0);
        est_state_(kPosY) = pos_(1);
        est_state_(kPosZ) = pos_(2);
        est_state_(kOriW) = quat_.w();
        est_state_(kOriX) = quat_.x();
        est_state_(kOriY) = quat_.y();
        est_state_(kOriZ) = quat_.z();
        est_state_(kVelX) = vel_(0);
        est_state_(kVelY) = vel_(1);
        est_state_(kVelZ) = vel_(2);

        const bool quaternion_norm_ok = abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
        
        return quaternion_norm_ok;
    }

    // Convert to Eigen format. 
    bool MPCController::setReference(
        const testbed_msgs::TrajectoryMPC& reference_trajectory) {
        reference_states_.setZero();
        reference_inputs_.setZero();

        const double dt = mpc_wrapper_.getTimestep();
        Eigen::Matrix<double, 3, 1> acceleration;
        const Eigen::Matrix<double, 3, 1> gravity(0.0, 0.0, -9.81);
        Eigen::Quaternion<double> q_heading;
        Eigen::Quaternion<double> q_orientation;
        bool quaternion_norm_ok(true);
        if (reference_trajectory.points.size() == 1) {
            q_heading = Eigen::Quaternion<double>(Eigen::AngleAxis<double>(
                reference_trajectory.points.front().heading,
                Eigen::Matrix<double, 3, 1>::UnitZ()));
            
            q_orientation.x() = reference_trajectory.points.front().pose.orientation.x;
            q_orientation.y() = reference_trajectory.points.front().pose.orientation.y;
            q_orientation.z() = reference_trajectory.points.front().pose.orientation.z;
            q_orientation.w() = reference_trajectory.points.front().pose.orientation.w;

            // TODO: Alim thinks it's important.. what a nerd. 
            // Question for Luigi: why are we multiplying orientation by a heading?
            q_orientation = q_orientation * q_heading;

            reference_states_ = (Eigen::Matrix<double, kStateSize, 1>()
                << reference_trajectory.points.front().pose.position.x,
                reference_trajectory.points.front().pose.position.y,
                reference_trajectory.points.front().pose.position.z,
                q_orientation.w(),
                q_orientation.x(),
                q_orientation.y(),
                q_orientation.z(),
                reference_trajectory.points.front().velocity.linear.x,
                reference_trajectory.points.front().velocity.linear.y,
                reference_trajectory.points.front().velocity.linear.z            
            ).finished().replicate(1, kSamples + 1);


            acceleration << reference_trajectory.points.front().acceleration.linear.x,
                            reference_trajectory.points.front().acceleration.linear.y,
                            reference_trajectory.points.front().acceleration.linear.z;

            acceleration -= gravity;
            reference_inputs_ = (Eigen::Matrix<double, kInputSize, 1>() << acceleration.norm(),
                reference_trajectory.points.front().velocity.angular.x,
                reference_trajectory.points.front().velocity.angular.y,
                reference_trajectory.points.front().velocity.angular.z
            ).finished().replicate(1, kSamples + 1);

        } else {
            auto iterator(reference_trajectory.points.begin());
            ros::Duration t_start = reference_trajectory.points.begin()->time_from_start;
            auto last_element = reference_trajectory.points.end();
            last_element = std::prev(last_element);

            for (int i = 0; i < kSamples + 1; i++) {
                while ((iterator->time_from_start - t_start).toSec() <= i * dt &&
                        iterator != last_element) {
                    iterator++;
                }

                q_heading = Eigen::Quaternion<double>(Eigen::AngleAxis<double>(
                    iterator->heading, Eigen::Matrix<double, 3, 1>::UnitZ()));
                q_orientation.x() = iterator->pose.orientation.x;
                q_orientation.y() = iterator->pose.orientation.y;
                q_orientation.z() = iterator->pose.orientation.z;
                q_orientation.w() = iterator->pose.orientation.w;
                            
                q_orientation = q_heading * q_orientation;

                reference_states_.col(i) << iterator->pose.position.x,
                    iterator->pose.position.y,
                    iterator->pose.position.z,
                    q_orientation.w(),
                    q_orientation.x(),
                    q_orientation.y(),
                    q_orientation.z(),
                    iterator->velocity.linear.x,
                    iterator->velocity.linear.y,
                    iterator->velocity.linear.z;


                if (reference_states_.col(i).segment(kOriW, 4).dot(
                    est_state_.segment(kOriW, 4)) < 0.0)
                    reference_states_.block(kOriW, i, 4, 1) =
                        -reference_states_.block(kOriW, i, 4, 1);
                    acceleration << iterator->acceleration.linear.x,
                            iterator->acceleration.linear.y,
                            iterator->acceleration.linear.z;

                    acceleration -= gravity;
                    reference_inputs_.col(i) << acceleration.norm(),
                        iterator->velocity.angular.x,
                        iterator->velocity.angular.y,
                        iterator->velocity.angular.z;

                    quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
            }
        }
        return quaternion_norm_ok;
    }

    // I have restructured run function to account for the fact that we work with callbacks
    void MPCController::run()
    {
        ros::service::waitForService("get_pred_window");
        while(true) {
            ros::Time call_time = ros::Time::now();
            const clock_t start = clock();
            if (changed_) {
                setNewParams();
            }
            if (preparation_thread_.joinable())
                preparation_thread_.join();

            // Convert everything into Eigen format.
            setStateEstimate();

            std::cout << "Debugging state estimate: \n"
                    << "Position: \n"
                    << "x: " << est_state_(kPosX) << std::endl
                    << "y: " << est_state_(kPosY) << std::endl
                    << "z: " << est_state_(kPosZ) << std::endl
                    << "Orientation: \n"
                    << "w: " << est_state_(kOriW) << std::endl
                    << "x: " << est_state_(kOriX) << std::endl
                    << "y: " << est_state_(kOriY) << std::endl
                    << "z: " << est_state_(kOriZ) << std::endl
                    << "Velocity: \n"
                    << "x: " << est_state_(kVelX) << std::endl
                    << "y: " << est_state_(kVelY) << std::endl
                    << "z: " << est_state_(kVelZ) << std::endl;

            // Request the current trajectory
            guidance::MPC_RefWindow srv;
            srv.request.Npoints = ACADO_N;
            srv.request.dt = mpc_wrapper_.getTimestep();

            if(!trajectory_clnt_.call(srv)){
                ROS_ERROR("MPC: The trajectory service is not responding!");
                return;
            }
            testbed_msgs::TrajectoryMPC reference_trajectory = srv.response.trj;
            testbed_msgs::ControlStamped control_msg;
            if (reference_trajectory.active == 1) {
                setReference(reference_trajectory);

                static const bool do_preparation_step(false);

                // Get the feedback from MPC.
                mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
                for (int i = 0; i < kSamples + 1; i++) {
                    std::cout << "Debugging reference states (sample " << i << "): \n" 
                            << "Position:" <<std::endl
                            << "x:" << reference_states_(kPosX, i) << std::endl
                            << "y:" << reference_states_(kPosY, i) << std::endl
                            << "z:" << reference_states_(kPosZ, i) << std::endl
                            << "Orientation:" <<std::endl
                            << "w:" << reference_states_(kOriW, i) << std::endl
                            << "x:" << reference_states_(kOriX, i) << std::endl
                            << "y:" << reference_states_(kOriY, i) << std::endl
                            << "z:" << reference_states_(kOriZ, i) << std::endl
                            << "Velocity:" <<std::endl
                            << "x:" << reference_states_(kVelX, i) << std::endl
                            << "y:" << reference_states_(kVelY, i) << std::endl
                            << "z:" << reference_states_(kVelZ, i) << std::endl;
                    std::cout << "Debugging reference inputs (sample " << i << "): \n" 
                            << "Position:" <<std::endl
                            << "thrust:" << reference_inputs_(kThrust, i) << std::endl
                            << "ratex:" << reference_inputs_(kRateX, i) << std::endl
                            << "ratey:" << reference_inputs_(kRateY, i) << std::endl
                            << "ratez:" << reference_inputs_(kRateZ, i) << std::endl;
                }


                if (solve_from_scratch_) {
                    ROS_INFO("Solving MPC with hover as initial guess.");
                    mpc_wrapper_.solve(est_state_);
                    solve_from_scratch_ = false;
                } else {
                    mpc_wrapper_.update(est_state_, do_preparation_step);
                }

                mpc_wrapper_.getStates(predicted_states_);
                mpc_wrapper_.getInputs(predicted_inputs_);
                
                // for (int i = 0; i < kSamples; i++) {
                //     std::cout << "Debugging (sample " << i << "): \n" 
                //             << "Position:" <<std::endl
                //             << "x:" << predicted_states_(kPosX, i) << std::endl
                //             << "y:" << predicted_states_(kPosY, i) << std::endl
                //             << "z:" << predicted_states_(kPosZ, i) << std::endl
                //             << "Orientation:" <<std::endl
                //             << "w:" << predicted_states_(kOriW, i) << std::endl
                //             << "x:" << predicted_states_(kOriX, i) << std::endl
                //             << "y:" << predicted_states_(kOriY, i) << std::endl
                //             << "z:" << predicted_states_(kOriZ, i) << std::endl;
                // }
                // Publish the predicted trajectory.
                publishPrediction(predicted_states_, predicted_inputs_, call_time);

                // Start a thread to prepare for the next execution.
                preparation_thread_ = std::thread(&MPCController::preparationThread, this);

                // Timing
                const clock_t end = clock();
                timing_feedback_ = 0.9 * timing_feedback_ +
                                    0.1 * double(end - start) / CLOCKS_PER_SEC;
                if (print_info_)
                    ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                                    timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

                // Publish the input control command
                control_msg = updateControlCommand(predicted_states_.col(0),
                                            predicted_inputs_.col(0),
                                            call_time);
            }
            else
            {
                control_msg.control.thrust = 0.0;
                control_msg.control.roll = 0.0;
                control_msg.control.pitch = 0.0;
                control_msg.control.yaw_dot = 0.0;
            }

            control_pub_.publish(control_msg);

        }
        return;
    }


    testbed_msgs::ControlStamped MPCController::updateControlCommand(
        const Eigen::Ref<const Eigen::Matrix<double, kStateSize, 1>> state,
        const Eigen::Ref<const Eigen::Matrix<double, kInputSize, 1>> input,
        ros::Time& time) {
        Eigen::Matrix<double, kInputSize, 1> input_bounded = input;

        // Bound inputs for sanity.
        input_bounded(INPUT::kThrust) = std::max(min_thrust_,
                                                std::min(max_thrust_, input_bounded(INPUT::kThrust)));
        input_bounded(INPUT::kRateX) = std::max(-max_bodyrate_xy_,
                                                std::min(max_bodyrate_xy_, input_bounded(INPUT::kRateX)));
        input_bounded(INPUT::kRateY) = std::max(-max_bodyrate_xy_,
                                                std::min(max_bodyrate_xy_, input_bounded(INPUT::kRateY)));
        input_bounded(INPUT::kRateZ) = std::max(-max_bodyrate_z_,
                                                std::min(max_bodyrate_z_, input_bounded(INPUT::kRateZ)));

        testbed_msgs::ControlStamped command;

        //TODO: Continue ... unclear how we control crazyflie
        command.header.stamp = time;
        //command.armed = true;
        //command.control_mode = quadrotor_common::ControlMode::BODY_RATES;
        //command.expected_execution_time = time;
        double mass = 0.03;
        command.control.thrust = mass * input_bounded(INPUT::kThrust);
        command.control.roll = input_bounded(INPUT::kRateX);
        command.control.pitch = input_bounded(INPUT::kRateY);
        command.control.yaw_dot = input_bounded(INPUT::kRateZ);
        //command.orientation.w() = state(STATE::kOriW);
        //command.orientation.x() = state(STATE::kOriX);
        //command.orientation.y() = state(STATE::kOriY);
        //command.orientation.z() = state(STATE::kOriZ);
        return command;
    }    
/*
    template<typename T>
    bool MpcController<T>::publishPrediction(
        const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
        const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
        ros::Time& time) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = time;
    path_msg.header.frame_id = "world";
    geometry_msgs::PoseStamped pose;
    T dt = mpc_wrapper_.getTimestep();

    for (int i = 0; i < kSamples; i++) {
        pose.header.stamp = time + ros::Duration(i * dt);
        pose.header.seq = i;
        pose.pose.position.x = states(kPosX, i);
        pose.pose.position.y = states(kPosY, i);
        pose.pose.position.z = states(kPosZ, i);
        pose.pose.orientation.w = states(kOriW, i);
        pose.pose.orientation.x = states(kOriX, i);
        pose.pose.orientation.y = states(kOriY, i);
        pose.pose.orientation.z = states(kOriZ, i);
        path_msg.poses.push_back(pose);
    }

    pub_predicted_trajectory_.publish(path_msg);

    return true;
    }
*/

    void MPCController::preparationThread() {
        const clock_t start = clock();

        mpc_wrapper_.prepare();

        // Timing
        const clock_t end = clock();
        timing_preparation_ = 0.9 * timing_preparation_ +
                                0.1 * double(end - start) / CLOCKS_PER_SEC;
    }

    // Process an incoming setpoint point change.
/*    void MPCController::SetpointCallback(
            const testbed_msgs::ControlSetpoint::ConstPtr& msg) {

        setpoint_type_ = msg->setpoint_type; 

        sp_pos_(0) = msg->p.x;
        sp_pos_(1) = msg->p.y;
        sp_pos_(2) = msg->p.z;

        sp_vel_(0) = msg->v.x;
        sp_vel_(1) = msg->v.y;
        sp_vel_(2) = msg->v.z;

        sp_acc_(0) = msg->a.x;
        sp_acc_(1) = msg->a.y;
        sp_acc_(2) = msg->a.z;

        sp_roll_ = msg->rpy.x;
        sp_pitch_ = msg->rpy.y;
        sp_yaw_ = msg->rpy.z;

        sp_brates_(0) = msg->brates.x;
        sp_brates_(1) = msg->brates.y;
        sp_brates_(2) = msg->brates.z;

        received_setpoint_ = true;
    }
*/

    // Process an incoming state measurement.
    void MPCController::StateCallback(
            const testbed_msgs::CustOdometryStamped::ConstPtr& msg) {
        // Catch no setpoint.
        //if (!received_setpoint_)
        //    return;

        //if (last_state_time_ < 0.0)
        //    last_state_time_ = ros::Time::now().toSec();

        // Send the message from the state to run function
        pos_(0) = msg->p.x;
        pos_(1) = msg->p.y;
        pos_(2) = msg->p.z;

        vel_(0) = msg->v.x;
        vel_(1) = msg->v.y;
        vel_(2) = msg->v.z;

        quat_.vec() = Vector3d (msg->q.x, msg->q.y, msg->q.z);
        quat_.w() = msg->q.w;
        quat_.normalize();

        //setStateEstimate();

        // Compute dt
        // float dt = ros::Time::now().toSec() - last_state_time_; // (float)(1.0f/ATTITUDE_RATE);
        // last_state_time_ = ros::Time::now().toSec();
        // std::cout << "dt: " << dt << std::endl;
/*
        testbed_msgs::ControlStamped control_msg;

        if (setpoint_type_ == "FullTrj") { 

            switch (ctrl_mode_) {
                // Control the drone with attitude commands
                case ControlMode::ANGLES: 
                    {
                        // Create "Heading" rotation matrix (x-axis aligned w/ drone but z-axis vertical)
                        ROS_ERROR("%s: Please control with rates.", name_.c_str());
                        break; 
                    }
                    // Control the drone with rate commands
                case ControlMode::RATES:
                    {
                        control_msg = run();
                        break;
                    }
                    // Something is wrong if Default...
                default:
                    ROS_ERROR("%s: Unable to select the control mode.", name_.c_str());
            }

        }
        
        if (setpoint_type_ == "StopCmd") {
            control_msg.control.thrust = 0.0;
            control_msg.control.roll = 0.0;
            control_msg.control.pitch = 0.0;
            control_msg.control.yaw_dot = 0.0;
        }

        control_pub_.publish(control_msg);
        */
        return;
    }

    bool MPCController::publishPrediction(const Eigen::Ref<const Eigen::Matrix<double, kStateSize, kSamples + 1>> states,
        const Eigen::Ref<const Eigen::Matrix<double, kInputSize, kSamples>> inputs,
        ros::Time& time) {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = time;
        path_msg.header.frame_id = "world";
        geometry_msgs::PoseStamped pose;
        double dt = mpc_wrapper_.getTimestep();

        for (int i = 0; i < kSamples; i++) {
            pose.header.stamp = time + ros::Duration(i * dt);
            pose.header.seq = i;
            pose.pose.position.x = states(kPosX, i);
            pose.pose.position.y = states(kPosY, i);
            pose.pose.position.z = states(kPosZ, i);
            pose.pose.orientation.w = states(kOriW, i);
            pose.pose.orientation.x = states(kOriX, i);
            pose.pose.orientation.y = states(kOriY, i);
            pose.pose.orientation.z = states(kOriZ, i);
            std::cout << "Debugging (sample " << i << "): \n" 
                            << "Position:" <<std::endl
                            << "x:" << states(kPosX, i) << std::endl
                            << "y:" << states(kPosY, i) << std::endl
                            << "z:" << states(kPosZ, i) << std::endl
                            << "Orientation:" <<std::endl
                            << "w:" << states(kOriW, i) << std::endl
                            << "x:" << states(kOriX, i) << std::endl
                            << "y:" << states(kOriY, i) << std::endl
                            << "z:" << states(kOriZ, i) << std::endl;
            path_msg.poses.push_back(pose);
        }

        predicted_traj_pub_.publish(path_msg);

        return true;
    }
}

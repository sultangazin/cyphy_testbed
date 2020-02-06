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

        if (!RegisterCallbacks(n)) {
            ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
            return false;
        }

        // // Load K, x_ref, u_ref from disk.
        // if (!LoadFromDisk()) {
        //   ROS_ERROR("%s: Failed to load K, x_ref, u_ref from disk.", name_.c_str());
        //   return false;
        // }

        // Set up control publisher.
        ros::NodeHandle nl(n);
        control_pub_ = nl.advertise<testbed_msgs::ControlStamped>(
                control_topic_.c_str(), 1, false);

        error_pub_ = nl.advertise<testbed_msgs::CtrlPerfStamped>(
                ctrl_perf_topic_.c_str(), 1, false);

        Reset();

        setNewParams();

        solve_from_scratch_ = true;

        preparation_thread_ = std::thread(&MpcWrapper<double>::prepare, mpc_wrapper_);

        initialized_ = true;
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
        Q_ = (Eigen::Matrix<T, kCostSize, 1>() <<
            Q_pos_xy, Q_pos_xy, Q_pos_z,
            Q_attitude, Q_attitude, Q_attitude, Q_attitude,
            Q_velocity, Q_velocity, Q_velocity,
            Q_perception, Q_perception).finished().asDiagonal();
        R_ = (Eigen::Matrix<T, kInputSize, 1>() <<
            R_thrust, R_pitchroll, R_pitchroll, R_yaw).finished().asDiagonal();

        // Read cost scaling values
        if (!nl.getParam("state_cost_exponential", state_cost_exponential_)) return false;
        if (!nl.getParam("input_cost_exponential", input_cost_exponential_)) return false;

        // Read input limits.
        if (!nl.getParam("max_bodyrate_xy", input_cost_exponential_)) return false;
        if (!nl.getParam("max_bodyrate_z", input_cost_exponential_)) return false;
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
        if (!nl.getParam("topics/setpoint", setpoint_topic_)) return false;
        if (!nl.getParam("topics/control", control_topic_)) return false;
        
        if (!nl.getParam("topics/ctrl_perf", ctrl_perf_topic_)) return false;

        // Control Mode
        if (!nl.getParam("control_mode", (int&)ctrl_mode_)) return false;

        return true;
    }

    // Register callbacks.
    bool MPCController::RegisterCallbacks(const ros::NodeHandle& n) {
        ros::NodeHandle nl(n);

        // Subscribers.
        state_sub_ = nl.subscribe(
                state_topic_.c_str(), 1, &MPCController::StateCallback, this);

        setpoint_sub_ = nl.subscribe(
                setpoint_topic_.c_str(), 1, &MPCController::SetpointCallback, this);

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
        const testbed_msgs::TrajectoryMPC::ConstPtr& reference_trajectory) {
        reference_states_.setZero();
        reference_inputs_.setZero();

        const double dt = mpc_wrapper_.getTimestep();
        Eigen::Matrix<double, 3, 1> acceleration;
        const Eigen::Matrix<double, 3, 1> gravity(0.0, 0.0, -9.81);
        Eigen::Quaternion<double> q_heading;
        Eigen::Quaternion<double> q_orientation;
        bool quaternion_norm_ok(true);
        if (reference_trajectory->points.size() == 1) {
            q_heading = Eigen::Quaternion<double>(Eigen::AngleAxis<double>(
                reference_trajectory->points.front().heading,
                Eigen::Matrix<double, 3, 1>::UnitZ()));

            // TODO: Alim thinks it's important.. what a nerd. 
            q_orientation = reference_trajectory->points.front().orientation.template cast<double>() * q_heading;
            reference_states_ = (Eigen::Matrix<double, kStateSize, 1>()
                << reference_trajectory->points.front().position.template cast<double>(),
                q_orientation.w(),
                q_orientation.x(),
                q_orientation.y(),
                q_orientation.z(),
                reference_trajectory->points.front().velocity.template cast<double>()
            ).finished().replicate(1, kSamples + 1);

            acceleration << reference_trajectory->points.front().acceleration.template cast<double>() - gravity;
            reference_inputs_ = (Eigen::Matrix<T, kInputSize, 1>() << acceleration.norm(),
                reference_trajectory->points.front().bodyrates.template cast<double>()
            ).finished().replicate(1, kSamples + 1);
        } else {
            auto iterator(reference_trajectory->points.begin());
            ros::Duration t_start = reference_trajectory->points.begin()->time_from_start;
            auto last_element = reference_trajectory->points.end();
            last_element = std::prev(last_element);

            for (int i = 0; i < kSamples + 1; i++) {
            while ((iterator->time_from_start - t_start).toSec() <= i * dt &&
                    iterator != last_element) {
                iterator++;
            }

            q_heading = Eigen::Quaternion<double>(Eigen::AngleAxis<double>(
                iterator->heading, Eigen::Matrix<double, 3, 1>::UnitZ()));
            q_orientation = q_heading * iterator->orientation.template cast<double>();
            reference_states_.col(i) << iterator->position.template cast<double>(),
                q_orientation.w(),
                q_orientation.x(),
                q_orientation.y(),
                q_orientation.z(),
                iterator->velocity.template cast<double>();
            if (reference_states_.col(i).segment(kOriW, 4).dot(
                est_state_.segment(kOriW, 4)) < 0.0)
                reference_states_.block(kOriW, i, 4, 1) =
                    -reference_states_.block(kOriW, i, 4, 1);
            acceleration << iterator->acceleration.template cast<double>() - gravity;
            reference_inputs_.col(i) << acceleration.norm(),
                iterator->bodyrates.template cast<double>();
            quaternion_norm_ok &= abs(est_state_.segment(kOriW, 4).norm() - 1.0) < 0.1;
            }
        }
        return quaternion_norm_ok;
    }

    // TODO:
    testbed_msgs::ControlStamped MPCController::run(
        const testbed_msgs::CustOdometryStamped::ConstPtr& state_estimate,
        const testbed_msgs::TrajectoryMPC::ConstPtr& reference_trajectory) {
        ros::Time call_time = ros::Time::now();
        const clock_t start = clock();
        if (changed_) {
            setNewParams();
        }

        preparation_thread_.join();

        // Convert everything into Eigen format.
        setStateEstimate();
        setReference(reference_trajectory);

        static const bool do_preparation_step(false);

        // Get the feedback from MPC.
        mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);
        if (solve_from_scratch_) {
            ROS_INFO("Solving MPC with hover as initial guess.");
            mpc_wrapper_.solve(est_state_);
            solve_from_scratch_ = false;
        } else {
            mpc_wrapper_.update(est_state_, do_preparation_step);
        }
        // TODO: CONTINUE..
        mpc_wrapper_.getStates(predicted_states_);
        mpc_wrapper_.getInputs(predicted_inputs_);

        // Publish the predicted trajectory.
        publishPrediction(predicted_states_, predicted_inputs_, call_time);

        // Start a thread to prepare for the next execution.
        preparation_thread_ = std::thread(&MPCController<T>::preparationThread, this);

        // Timing
        const clock_t end = clock();
        timing_feedback_ = 0.9 * timing_feedback_ +
                            0.1 * double(end - start) / CLOCKS_PER_SEC;
        if (params_.print_info_)
            ROS_INFO_THROTTLE(1.0, "MPC Timing: Latency: %1.1f ms  |  Total: %1.1f ms",
                            timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);

        // Return the input control command.
        return updateControlCommand(predicted_states_.col(0),
                                    predicted_inputs_.col(0),
                                    call_time);
    }

    // Process an incoming setpoint point change.
    void MPCController::SetpointCallback(
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

    // Process an incoming state measurement.
    void MPCController::StateCallback(
            const testbed_msgs::CustOdometryStamped::ConstPtr& msg) {
        // Catch no setpoint.
        if (!received_setpoint_)
            return;

        if (last_state_time_ < 0.0)
            last_state_time_ = ros::Time::now().toSec();

        // Read the message into the state
        pos_(0) = msg->p.x;
        pos_(1) = msg->p.y;
        pos_(2) = msg->p.z;

        vel_(0) = msg->v.x;
        vel_(1) = msg->v.y;
        vel_(2) = msg->v.z;

        quat_.vec() = Vector3d (msg->q.x, msg->q.y, msg->q.z);
        quat_.w() = msg->q.w;
        quat_.normalize();

        // Compute dt
        float dt = ros::Time::now().toSec() - last_state_time_; // (float)(1.0f/ATTITUDE_RATE);
        last_state_time_ = ros::Time::now().toSec();
        // std::cout << "dt: " << dt << std::endl;

        testbed_msgs::ControlStamped control_msg;

        if (setpoint_type_ == "FullTrj") { 
            // Position and Velocity error
            Vector3d p_error = sp_pos_ - pos_;
            Vector3d v_error = sp_vel_ - vel_;
            // std::cout << "p_error: " << p_error << std::endl;
            // std::cout << "v_error: " << v_error << std::endl;

            testbed_msgs::CtrlPerfStamped ctrl_perf_msg;
            ctrl_perf_msg.ep.x = p_error(0);
            ctrl_perf_msg.ep.y = p_error(1);
            ctrl_perf_msg.ep.z = p_error(2);
            
            ctrl_perf_msg.ev.x = v_error(0);
            ctrl_perf_msg.ev.y = v_error(1);
            ctrl_perf_msg.ev.z = v_error(2);


            // Integral Error
            i_error_x += p_error(0) * dt;
            i_error_x = std::max(std::min(p_error(0), i_range_xy), -i_range_xy);

            i_error_y += p_error(1) * dt;
            i_error_y = std::max(std::min(p_error(1), i_range_xy), -i_range_xy);

            i_error_z += p_error(2) * dt;
            i_error_z = std::max(std::min(p_error(2), i_range_z), -i_range_z);

            // Desired thrust [F_des]
            Vector3d target_thrust = Vector3d::Zero();
            Vector3d fb_thrust = Vector3d::Zero();

            fb_thrust(0) = kp_xy * p_error(0) + kd_xy * v_error(0) + ki_xy * i_error_x;
            fb_thrust(1) = kp_xy * p_error(1) + kd_xy * v_error(1) + ki_xy * i_error_y;
            fb_thrust(2) = kp_z  * p_error(2) + kd_z  * v_error(2) + ki_z  * i_error_z;

            target_thrust(0) = sp_acc_(0);
            target_thrust(1) = sp_acc_(1);
            target_thrust(2) = (sp_acc_(2) + GRAVITY_MAGNITUDE);
                        
            ctrl_perf_msg.fb_t.x = fb_thrust(0);
            ctrl_perf_msg.fb_t.y = fb_thrust(1);
            ctrl_perf_msg.fb_t.z = fb_thrust(2);
        
            ctrl_perf_msg.ff_t.x = target_thrust(0);
            ctrl_perf_msg.ff_t.y = target_thrust(1);
            ctrl_perf_msg.ff_t.z = target_thrust(2) - GRAVITY_MAGNITUDE;

            target_thrust = target_thrust + fb_thrust;  
            // std::cout << "target_thrust: " << target_thrust << std::endl;

            error_pub_.publish(ctrl_perf_msg);

            // Move YAW angle setpoint
            double yaw_rate = 0;
            double yaw_des = sp_yaw_;

            // Z-Axis [zB]
            Matrix3d R = quat_.toRotationMatrix();
            Vector3d z_axis = R.col(2);

            // Current thrust [F]
            double current_thrust = target_thrust.dot(z_axis);

            // Calculate axis [zB_des]
            Vector3d z_axis_desired = target_thrust.normalized();

            // [xC_des]
            // x_axis_desired = z_axis_desired x [sin(yaw), cos(yaw), 0]^T
            Vector3d x_c_des;
            x_c_des(0) = cosf(radians(yaw_des));
            x_c_des(1) = sinf(radians(yaw_des));
            x_c_des(2) = 0;

            // [yB_des]
            // Vector3d y_axis_desired = (z_axis_desired.cross(x_c_des)).normalized();
            Vector3d y_axis_desired = (z_axis_desired.cross(R.col(0))).normalized();

            // [xB_des]
            Vector3d x_axis_desired = (y_axis_desired.cross(z_axis_desired)).normalized();

            Matrix3d Rdes;
            Rdes.col(0) = x_axis_desired;
            Rdes.col(1) = y_axis_desired;
            Rdes.col(2) = z_axis_desired;


            switch (ctrl_mode_) {
                // Control the drone with attitude commands
                case ControlMode::ANGLES: 
                    {
                        // Create "Heading" rotation matrix (x-axis aligned w/ drone but z-axis vertical)
                        Matrix3d Rhdg;
                        Vector3d x_c(R(0,0) ,R(1,0), 0);
                        x_c.normalize();
                        Vector3d z_c(0, 0, 1);
                        Vector3d y_c = z_c.cross(x_c);
                        Rhdg.col(0) = x_c;
                        Rhdg.col(1) = y_c;
                        Rhdg.col(2) = z_c;

                        Matrix3d Rout = Rhdg.transpose() * Rdes;

                        Matrix3d Rerr = 0.5 * (Rdes.transpose() * Rhdg - Rhdg.transpose() * Rdes);
                        Vector3d Verr(-Rerr(1,2),Rerr(0,2),-Rerr(0,1));

                        // std::cout << "Rout: " << Rout << std::endl;

                        control_msg.header.stamp = ros::Time::now();

                        control_msg.control.roll = std::atan2(Rout(2,1),Rout(2,2));
                        control_msg.control.pitch = -std::asin(Rout(2,0));
                        control_msg.control.yaw_dot = -10*std::atan2(R(1,0),R(0,0)); //std::atan2(Rdes(1,0),Rdes(0,0));
                        control_msg.control.thrust = current_thrust;
                        break; 
                    }
                    // Control the drone with rate commands
                case ControlMode::RATES:
                    {
                        // Compute the rotation error between the desired z_ and the current one in Inertial frame
                        Vector3d ni = z_axis.cross(z_axis_desired);
                        double alpha = std::acos(ni.norm());
                        ni.normalize();

                        // Express the axis in body frame
                        Vector3d nb = quat_.inverse() * ni;
                        Quaterniond q_pq(Eigen::AngleAxisd(alpha, nb));

                        control_msg.control.roll = (q_pq.w() > 0) ? (2.0 * kpq_rates_ * q_pq.x()) : (-2.0 * kpq_rates_ * q_pq.x());
                        control_msg.control.pitch = (q_pq.w() > 0) ? (2.0 * kpq_rates_ * q_pq.y()) : (-2.0 * kpq_rates_ * q_pq.y());

                        Quaterniond q_r = q_pq.inverse() * quat_.inverse() * Quaterniond(Rdes);
                        control_msg.control.yaw_dot = (q_r.w() > 0) ? (2.0 * kr_rates_ * q_r.z()) : (-2.0 * kr_rates_ * q_r.z());
                        control_msg.control.yaw_dot = -1.0 * control_msg.control.yaw_dot;

                        break;
                    }
                    // Something is wrong if Default...
                default:
                    ROS_ERROR("%s: Unable to select the control mode.", name_.c_str());
            }

        }

        if (setpoint_type_ == "AttTrj") {
            
            Quaterniond qt_i;
            qt_i = Eigen::AngleAxisd(sp_yaw_, Vector3d::UnitZ()) * 
                Eigen::AngleAxisd(sp_pitch_, Vector3d::UnitY()) *
                Eigen::AngleAxisd(sp_roll_, Vector3d::UnitX());

           // // Z target in inertial frame
           // double x = cos(sp_roll_) * sin(sp_pitch) * cos(sp_yaw_) + sin(sp_roll_) * sin(sp_yaw_);
           // double y = cos(sp_roll_) * sin(sp_pitch) * sin(sp_yaw_) - sin(sp_roll_) * cos(sp_yaw_);
           // double z = cos(sp_yaw_) * cos(sp_roll_);
           // Vector3d zt_i(x, y, z); 

           // // Z target in body frame
           // Vector3d zt_b = quat_.inverse() * zt_i;

           Quaterniond qt_b = quat_.inverse() * qt_i; // Rotation Body to Target

           double sinr_cosp = 2.0 * (qt_b.w() * qt_b.x() + qt_b.y() * qt_b.z());
           double cosr_cosp = 1.0 - 2.0 * (qt_b.x() * qt_b.x() + qt_b.y() * qt_b.y());
           double roll = atan2(sinr_cosp, cosr_cosp);
            
           double sinp = 2.0 * (qt_b.w() * qt_b.y() - qt_b.z() * qt_b.x());

           double pitch = asin(sinp);

            
           // I need to keep the thrust over a limit 
           // in order to control angles, otherwise
           // everything will be set to 0.
           control_msg.control.thrust = 0.25;
           control_msg.control.roll = roll;
           control_msg.control.pitch = pitch;
           control_msg.control.yaw_dot = 0.0;

        }

        if (setpoint_type_ == "StopCmd") {
            control_msg.control.thrust = 0.0;
            control_msg.control.roll = 0.0;
            control_msg.control.pitch = 0.0;
            control_msg.control.yaw_dot = 0.0;
        }

        control_pub_.publish(control_msg);
    }

}

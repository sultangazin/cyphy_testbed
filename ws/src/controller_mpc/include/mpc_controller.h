/*
 * Copyright (c) 2017, The Regents of the University of California (Regents)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Linear feedback controller that reads in control/references from files.
// Controllers for specific state spaces will be derived from this class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <thread>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

#include <guidance/MPC_RefWindow.h>

#include "types.h"
#include "angles.h"
#include <testbed_msgs/ControlSetpoint.h>
#include <testbed_msgs/Control.h>
#include <testbed_msgs/ControlStamped.h>
#include <testbed_msgs/FullStateStamped.h>
#include <testbed_msgs/CustOdometryStamped.h>
#include <testbed_msgs/CtrlPerfStamped.h>
#include <testbed_msgs/TrajectoryMPC.h>
#include <testbed_msgs/TrajectoryPointMPC.h>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>
#include <fstream>

#include "mpc_wrapper.h"

namespace controller_mpc {

  enum class ControlMode {
    ANGLES, RATES, NUM_MODES
  };

  enum STATE {
    kPosX = 0,
    kPosY = 1,
    kPosZ = 2,
    kOriW = 3,
    kOriX = 4,
    kOriY = 5,
    kOriZ = 6,
    kVelX = 7,
    kVelY = 8,
    kVelZ = 9
  };

  enum INPUT {
    kThrust = 0,
    kRateX = 1,
    kRateY = 2,
    kRateZ = 3
  };
    
  class MPCController {
    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      static_assert(kStateSize == 10,
                    "MpcController: Wrong model size. Number of states does not match.");
      static_assert(kInputSize == 4,
                    "MpcController: Wrong model size. Number of inputs does not match.");


      // Initialize this class by reading parameters and loading callbacks.
      bool Initialize(const ros::NodeHandle& n);

      // Compute control given the current state.
      Vector3d Control(const VectorXd& x) const;
      
      MPCController():
        received_setpoint_(false),
        last_state_time_(-1.0),
        initialized_(false),
        mpc_wrapper_(MpcWrapper<double>()),
        timing_feedback_(1e-3),
        timing_preparation_(1e-3),
        est_state_((Eigen::Matrix<double, kStateSize, 1>() <<
                                                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0).finished()),
        reference_states_(Eigen::Matrix<double, kStateSize, kSamples + 1>::Zero()),
        reference_inputs_(Eigen::Matrix<double, kInputSize, kSamples + 1>::Zero()),
        predicted_states_(Eigen::Matrix<double, kStateSize, kSamples + 1>::Zero()),
        predicted_inputs_(Eigen::Matrix<double, kInputSize, kSamples>::Zero()),
        point_of_interest_(Eigen::Matrix<double, 3, 1>::Zero()),
        
        changed_(false),
        print_info_(false),
        state_cost_exponential_(0.0),
        input_cost_exponential_(0.0),
        max_bodyrate_xy_(0.0),
        max_bodyrate_z_(0.0),
        min_thrust_(0.0),
        max_thrust_(0.0),
        p_B_C_(Eigen::Matrix<double, 3, 1>::Zero()),
        q_B_C_(Eigen::Quaternion<double>(1.0, 0.0, 0.0, 0.0)),
        Q_(Eigen::Matrix<double, kCostSize, kCostSize>::Zero()),
        R_(Eigen::Matrix<double, kInputSize, kInputSize>::Zero()) {}

      // Load parameters and register callbacks. These may/must be overridden
      // by derived classes.
      bool LoadParameters(const ros::NodeHandle& n);
      bool RegisterCallbacks(const ros::NodeHandle& n);

      void Reset(void);

      // MPC functions below

      bool setNewParams(void);

      bool setStateEstimate(void);

      bool setReference(const testbed_msgs::TrajectoryMPC& reference_trajectory);

      void run();

      testbed_msgs::ControlStamped updateControlCommand(
        const Eigen::Ref<const Eigen::Matrix<double, kStateSize, 1>> state,
        const Eigen::Ref<const Eigen::Matrix<double, kInputSize, 1>> input,
        ros::Time& time);

      // Process an incoming state measurement.
      void StateCallback(const testbed_msgs::CustOdometryStamped::ConstPtr& msg);

      // Preparation thread function 
      void preparationThread(void);

      bool publishPrediction(const Eigen::Ref<const Eigen::Matrix<double, kStateSize, kSamples + 1>> states,
      const Eigen::Ref<const Eigen::Matrix<double, kInputSize, kSamples>> inputs,
      ros::Time& time);

      ControlMode ctrl_mode_;

      // MPC Parameters
      bool changed_;
      bool print_info_;
      bool solve_from_scratch_;
      std::thread preparation_thread_;

        // MPC
      MpcWrapper<double> mpc_wrapper_;

      // Variables
      double timing_feedback_, timing_preparation_;
      Eigen::Matrix<double, kStateSize, 1> est_state_;
      Eigen::Matrix<double, kStateSize, kSamples + 1> reference_states_;
      Eigen::Matrix<double, kInputSize, kSamples + 1> reference_inputs_;
      Eigen::Matrix<double, kStateSize, kSamples + 1> predicted_states_;
      Eigen::Matrix<double, kInputSize, kSamples> predicted_inputs_;
      Eigen::Matrix<double, 3, 1> point_of_interest_;

      double state_cost_exponential_;
      double input_cost_exponential_;

      double max_bodyrate_xy_;
      double max_bodyrate_z_;
      double min_thrust_;
      double max_thrust_;

      Eigen::Matrix<double, 3, 1> p_B_C_;
      Eigen::Quaternion<double> q_B_C_;

      Eigen::Matrix<double, kCostSize, kCostSize> Q_;
      Eigen::Matrix<double, kInputSize, kInputSize> R_;

      double prev_omega_roll;
      double prev_omega_pitch;
      double prev_setpoint_omega_roll;
      double prev_setpoint_omega_pitch;

      Vector3d sp_pos_, sp_vel_, sp_acc_, sp_r_pos_, sp_r_vel_, sp_r_acc_;
      Vector3d sp_brates_;
      Vector3d pos_, vel_, r_pos_, r_vel_;
      Eigen::Quaterniond quat_;

      std::string setpoint_type_;

      double sp_roll_,sp_pitch_,sp_yaw_;

      // Logging variables
      Vector3d z_axis_desired;

      // Dimensions of control and state spaces.
      size_t x_dim_;
      size_t u_dim_;

        // Integral of position error.
      Vector3d x_int_;
      Vector3d x_int_thresh_;
      Vector3d integrator_k_;

      // Remember last time we got a state callback.
      double last_state_time_;

      // Publishers and subscribers.
      ros::Subscriber state_sub_;
      ros::Subscriber setpoint_sub_;
      //ros::Subscriber reference_sub_;
      ros::Publisher control_pub_;
      ros::Publisher error_pub_;
      ros::Publisher predicted_traj_pub_;

      std::string state_topic_;
      std::string setpoint_topic_;
      //std::string reference_topic_;
      std::string control_topic_;
      std::string ctrl_perf_topic_;
      std::string predicted_traj_topic_;


      // Initialized flag and name.
      bool received_setpoint_;
      bool initialized_;
      std::string name_;

      ros::ServiceClient trajectory_clnt_;

      std::thread run_thread_;

      // // Load K, x_ref, u_ref from disk.
      // bool LoadFromDisk();

  }; //\class MPCController

} 

#endif

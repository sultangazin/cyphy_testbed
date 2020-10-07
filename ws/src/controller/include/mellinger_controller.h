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

#ifndef MELLINGER_CONTROLLER_H
#define MELLINGER_CONTROLLER_H

#include "types.h"
#include "angles.h"
#include <testbed_msgs/ControlSetpoint.h>
#include <testbed_msgs/Control.h>
#include <testbed_msgs/ControlStamped.h>
#include <testbed_msgs/FullStateStamped.h>
#include <testbed_msgs/CustOdometryStamped.h>
#include <testbed_msgs/CtrlPerfStamped.h>
#include <Eigen/Geometry>

#include <crazyflie_driver/PWM.h>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <fstream>
#include <unordered_map>

namespace controller {

enum class ControlMode {
    ANGLES, RATES, PWM, NUMCTRLMODES
};
 
class MellingerController {
  public:
  MellingerController();

  // Initialize this class by reading parameters and loading callbacks.
  bool Initialize(const ros::NodeHandle& n);

  // Compute control given the current state.
  Vector3d Control(const VectorXd& x) const;
  

  // Load parameters and register callbacks. These may/must be overridden
  // by derived classes.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  void Reset(void);

  // Process an incoming setpoint point.
  void SetpointCallback(
    const testbed_msgs::ControlSetpoint::ConstPtr& msg);

  // Process an incoming state measurement.
  void StateCallback(
    const testbed_msgs::CustOdometryStamped::ConstPtr& msg);

  std::string ctrl_mode_;

  double g_vehicleMass; // TODO: should be CF global for other modules
  double massThrust;

  // XY Position PID
  double kp_xy;       // P
  double kd_xy;       // D
  double ki_xy;      // I
  double i_range_xy;

  // Z Position
  double kp_z;       // P
  double kd_z;        // D
  double ki_z;       // I
  double i_range_z;

  // Attitude
  double kR_xy; // P
  double kw_xy; // D
  double ki_m_xy; // I
  double i_range_m_xy;

  // Yaw
  double kR_z; // P
  double kw_z; // D
  double ki_m_z; // I
  double i_range_m_z;

  // roll and pitch angular velocity
  double kd_omega_rp; // D  

  // Gain for the rate controller
  double kpq_rates_; // Gain for roll/pitch
  double kr_rates_; // Gain for yaw

  // Helper variables
  double i_error_x;
  double i_error_y;
  double i_error_z;

  double prev_omega_roll;
  double prev_omega_pitch;
  double prev_setpoint_omega_roll;
  double prev_setpoint_omega_pitch;

  double i_error_m_x;
  double i_error_m_y;
  double i_error_m_z;

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
  ros::Publisher control_pub_;
  ros::Publisher control_pwm_pub_;
  ros::Publisher error_pub_;

  std::string state_topic_;
  std::string setpoint_topic_;
  std::string control_topic_;
  std::string control_pwm_topic_;
  std::string ctrl_perf_topic_;

  // Initialized flag and name.
  bool received_setpoint_;
  bool initialized_;
  std::string name_;

  std::string vehicle_name_;

  std::map<std::string, ControlMode> controlMode_;

 
  // // Load K, x_ref, u_ref from disk.
  // bool LoadFromDisk();

}; //\class MellingerController

} 

#endif

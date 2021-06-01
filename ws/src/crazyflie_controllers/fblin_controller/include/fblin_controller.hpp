/*
 * Copyright (c) 2021, The Regents of the University of California (Regents).
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
 * Authors: Luigi Pannocchi ( lpannocchi@g.ucla.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// The Flatoutput Controller node.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FBLIN_CONTROLLER_H 
#define FBLIN_CONTROLLER_H 

#include <testbed_msgs/ControlSetpoint.h>
#include <testbed_msgs/ControlStamped.h>
#include <testbed_msgs/CustOdometryStamped.h>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <math.h>

#include "types.h"

namespace fblin_controller {

	class FBLinController {
		public:
			FBLinController();

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


		private:
			double vehicleMass_;

			// XY Position PID
			double kp_xy_;	// P
			double kd_xy_;	// D
			double ka_xy_;	// A

			// Z Position
			double kp_z_;	// P
			double kd_z_;	// D
			double ka_z_;	// A

			// Yaw
			double kR_z_; // P
			double kw_z_; // D

			Vector3d sp_pos_, sp_vel_, sp_acc_, sp_jrk_; 
			Vector3d pos_, vel_, acc_;
			Eigen::Quaterniond quat_;

			std::string setpoint_type_;

			double thrust_;

			Vector3d u_body_;


			// Logging variables
			Vector3d z_axis_desired;

			// Publishers and subscribers.
			ros::Subscriber state_sub_;
			ros::Subscriber setpoint_sub_;

			ros::Publisher control_pub_;

			std::string state_topic_;
			std::string setpoint_topic_;
			std::string control_topic_;

			// Initialized flag and name.
			bool received_setpoint_;
			bool initialized_;

			std::string vehicle_name_;
			std::string name_;

			ros::Time previous_;

	}; //\class FBLinController
} 
#endif

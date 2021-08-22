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

#pragma once

#include <testbed_msgs/ControlStamped.h>
#include <testbed_msgs/CustOdometryStamped.h>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <math.h>

#include "cis_supervisor/cis_supervisor.hpp"
#include <cis_supervisor/ObstacleMsg.h>

namespace cis_supervisor {
	const int X_COORD = 0;
	const int Y_COORD = 1;
	const int Z_COORD = 2;

	class CISSupervisorROS {
		public:
			// Default constructor
			CISSupervisorROS();

			// Initialization function
			bool Initialize(const ros::NodeHandle& n);

			// Reset Function
			void Reset(void);

		private:

			double ControllerDT_;

			/**
			 * Reference to the CIS Supervisor Class
			 */
			CISSupervisor* cis_supervisor_;

			double vehicle_Mass_;

			Eigen::MatrixXd Ad_;
			Eigen::MatrixXd Bd_;
			Eigen::MatrixXd Ed_;

			Eigen::Vector3d pos_, vel_, acc_;

			Eigen::Quaterniond w_q_b_;

			/**
			 * Control 
			 */
			double thrust_;
			Eigen::Vector3d w_jerk_;

			// Publishers and subscribers.
			ros::Subscriber state_sub_;
			ros::Subscriber ctrl_setpoint_sub_;
			ros::Subscriber obstacle_sub_;
			ros::Publisher control_pub_;

			std::string state_topic_;
			std::string control_input_topic_;
			std::string control_topic_;
			std::string obstacle_input_topic_;

			// Initialized flag and name.
			bool initialized_;

			std::string vehicle_name_;
			std::string namespace_;

			ros::Time previous_;

			// Load parameters and Initialize Pub/Sub.
			bool LoadParameters();
			bool InitPubSubs(const ros::NodeHandle& n);

			/**
			 * Process an incoming control messages.
			 */
			void UpdateControl(const testbed_msgs::ControlStamped::ConstPtr&);

			// Process an incoming state measurement.
			void StateCallback(
					const testbed_msgs::CustOdometryStamped::ConstPtr& msg);

			void ObstacleCallback(
					const cis_supervisor::ObstacleMsg::ConstPtr& msg);

			double differentiate(double x);
	}; //\class CISSupervisorROS
} 

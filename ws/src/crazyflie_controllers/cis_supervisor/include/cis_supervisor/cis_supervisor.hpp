#pragma once

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <cis2m/hpolyhedron.hpp>
#include <cis2m/cis_generator.hpp>

#include <drake/systems/framework/framework_common.h>
#include <drake/geometry/optimization/hpolyhedron.h>

namespace cis_supervisor {
	struct ObstacleData {
		unsigned char id;
		Eigen::Vector3d pos;
		Eigen::Vector3d vel;
	};

	class CISSupervisor {
		public:
			/**
			 * \brief Constructor for systems with disturbance   
			 * x_new = Ad * x + Bd * u + Ed * w
			 * \param[in]	Ad
			 * \param[in]	Bd
			 * \param[in]	Ed
			 */
			CISSupervisor(const Eigen::MatrixXd& Ad,
					const Eigen::MatrixXd& Bd,
					const Eigen::MatrixXd& Ed,
					const Eigen::Vector3d& x0);

			/**
			 * \brief Constructor for systems without disturbance   
			 * x_new = Ad * x + Bd * u
			 * \param[in]	Ad
			 * \param[in]	Bd
			 */
			CISSupervisor(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::Vector3d& x0);

			/**
			 * \brief Add information about the safe set
			 * \param[in]	Ss_A
			 * \param[in]	Ss_b
			 */
			void AddSafeSet(const Eigen::MatrixXd& Ss_A, const Eigen::VectorXd& Ss_b);

			/**
			 * \brief Update the current control command
			 * \param[in]	w_jerk	Jerk control in world reference frame
			 *
			 * return	Control filtered by the supervisor
			 */
			Eigen::Vector3d UpdateControl(const Eigen::Vector3d w_jerk);

			/**
			 * \brief Update the state of the constrolled agent 
			 * 
			 * \param[in]	p	Position
			 * \param[in]	v	Velocity
			 * \param[in]	a	Acceleration
			 *
			 * return	void
			 */
			void UpdateState(
					const Eigen::Vector3d& p, const Eigen::Vector3d& v,
					const Eigen::Vector3d& a);


			/**
			 * \brief Update the map of the obstacles
			 * \param[in]	obst	Obstacle data
			 *
			 * return	void
			 */
			void UpdateObstacle(const ObstacleData& obst);

			
		private:
			/**
			 * Dynamic Model: A matrix
			 */
			Eigen::MatrixXd Ad_;

			/** 
			 * Dynamic Model: B matrix
			 */
			Eigen::MatrixXd Bd_;

			/**
			 * Dynamic Model: E matrix
			 */
			Eigen::MatrixXd Ed_;

			/**
			 * State vector of the system
			 */
			Eigen::VectorXd translational_state_;

			/**
			 * Jerk command in world frame
			 */
			Eigen::Vector3d w_jerk_; 


			/**
			 * CIS Generator Class Reference
			 */
			cis2m::CISGenerator* cis_gen_;

			/**
			 * Computed CIS
			 */
			cis2m::HPolyhedron CIS_;

			/**
			 * SafeSet
			 */
			cis2m::HPolyhedron SafeSet_;


			/**
			 * Obstacle Map
			 */
			std::unordered_map<int, ObstacleData> obst_map_;

			/**
			 * Domain of the problem
			 */
			drake::geometry::optimization::HPolyhedron* domain_;


			/**
			 * \brief Compute a prediction step of the model
			 */
			Eigen::VectorXd Predict_step(const Eigen::Vector3d& w_jerk_);

			/**
			 * \brief Solve the optimiation problem to select the best control command that satisfy the constraints
			 */
			Eigen::Vector3d SolveOptimizationProblem(
					const Eigen::Vector3d& j, const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

			/**
			 * \brief Compute the free space given the current position and having a map of the obstacles
			 */
			cis2m::HPolyhedron ComputeFreeSpace(const Eigen::VectorXd& p);
	};
}

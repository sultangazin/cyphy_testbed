#pragma once

#include <Eigen/Dense>
#include <vector>
#include "cis_supervisor/hpolyhedron.hpp"
#include "cis_supervisor/cis_generator.hpp"

namespace cis_supervisor {
	typedef int ObstacleData;
	class CISSupervisor {
		public:
			CISSupervisor(const Eigen::MatrixXd& Ad,
					const Eigen::MatrixXd& Bd,
					const Eigen::MatrixXd& Ed);
			CISSupervisor(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

			/**
			 * Update the current control command
			 */
			Eigen::Vector3d UpdateControl(const Eigen::Vector3d w_jerk);

			/**
			 * Update the state of the constrolled agent 
			 */
			void UpdateState(
					const Eigen::Vector3d&, const Eigen::Vector3d&,
					const Eigen::Vector3d&);


			/**
			 * Update the map of the obstacles
			 */
			void UpdateObstacle(const std::vector<ObstacleData>& obsts);

			
		private:
			/**
			 * Dynamic Model
			 */
			Eigen::MatrixXd Ad_;
			Eigen::MatrixXd Bd_;
			Eigen::MatrixXd Ed_;

			Eigen::VectorXd translational_state_;

			/**
			 * Jerk command in world frame
			 */
			Eigen::Vector3d w_jerk_; 


			/**
			 * CIS Generator Class Reference
			 */
			cis2m::CISGenerator* cis_gen_;

			cis2m::HPolyhedron CIS_;


			Eigen::VectorXd predict_step(const Eigen::VectorXd&translational_state_, const Eigen::Vector3d& w_jerk_);

			Eigen::Vector3d SolveOptimizationProblem(
					const Eigen::Vector3d& j, const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

			cis2m::HPolyhedron ComputeFreeSpace(
					const Eigen::VectorXd& p, std::vector<ObstacleData>& obst);
	};
}

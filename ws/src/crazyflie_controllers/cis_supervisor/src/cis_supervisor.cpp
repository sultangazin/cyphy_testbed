#include "cis_supervisor/cis_supervisor.hpp"
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
// choose exact integral type
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

#include "drake/geometry/optimization/iris.h"
#include "drake/geometry/optimization/vpolytope.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;


// program and solution types
typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

namespace cis_supervisor {
	CISSupervisor::CISSupervisor(const MatrixXd& Ad, const MatrixXd& Bd, const MatrixXd& Ed, const Vector3d& x0) : Ad_(Ad), Bd_(Bd), Ed_(Ed) {
		cis_gen_ = new cis2m::CISGenerator(Ad_, Bd_, Ed_);
		translational_state_ = VectorXd::Zero(Ad_.rows());
		translational_state_.head(3) = x0;
		w_jerk_ = Vector3d::Zero();
	}


	CISSupervisor::CISSupervisor(const MatrixXd& Ad, const VectorXd& Bd, const Vector3d& x0) : Ad_(Ad), Bd_(Bd) {
		cis_gen_->AddDisturbanceSet(cis2m::HPolyhedron(Ad, Bd));
		translational_state_ = VectorXd::Zero(Ad_.rows());
		translational_state_.head(3) = x0;
		w_jerk_ = Vector3d::Zero();
	}

	void CISSupervisor::AddSafeSet(const MatrixXd& Ss_A, const VectorXd& Ss_b) {
		domain_ = new drake::geometry::optimization::HPolyhedron(Ss_A, Ss_b);
		SafeSet_ = cis2m::HPolyhedron(domain_->A(), domain_->b());
		cis_gen_->computeCIS(SafeSet_, 6, 0);
	}


	Vector3d CISSupervisor::UpdateControl(const Vector3d w_jerk) {
		w_jerk_ = w_jerk;
		VectorXd pred_state = Predict_step(w_jerk_);

		std::cout << "Cazzo 0" << std::endl;
		if (CIS_.isValid()) {
			if (CIS_.Contains(pred_state)) {
				// Don't do anything, you are going to stay in the CIS
				std::cout << "I am ok!" << std::endl;
				return w_jerk_;
			} else {
				// Compute the U
				std::cout << "Correcting..." << std::endl;	
				w_jerk_ = SolveOptimizationProblem(w_jerk_, CIS_.Ai(), CIS_.bi());
			}
		}

		
		return w_jerk_;
	}


	Vector3d CISSupervisor::SolveOptimizationProblem(
			const Vector3d& j, const MatrixXd& A, const VectorXd& b) {
		const int X = 0;
		const int Y = 1;
		const int Z = 2;
		MatrixXd A_prime = A * Bd_;
		VectorXd b_prime = b - A * Ad_ * translational_state_;
		// by default, we have a nonnegative QP with Ax <= b
		Program qp (CGAL::SMALLER, true, 0, false, 0);
		// now set the non-default entries:
		for (int i = 0; i < A_prime.rows(); i++) {
			qp.set_b(i, b_prime(i));
			for (int j = 0; j < A_prime.cols(); j++) {
				qp.set_a(j, i,  A_prime(i, j)); 
			}
		}
		// cost: <u, u> -2*<u, u0> + <u0, u0>
		//qp.set_u(Y, true, 100);                                   		//       y <= 4
		qp.set_d(X, X, 2); qp.set_d (Y, Y, 2); qp.set_d(Z, Z, 2);   		// !!specify 2D!!    x^2 + 4 y^2
		qp.set_c(X, -2*j(X)); qp.set_c(Y, -2*j(Y)); qp.set_c(Z, -2*j(Z));       // -32y
		qp.set_c0(j.norm());                                          		// +64
		// solve the program, using ET as the exact type
		Solution s = CGAL::solve_quadratic_program(qp, ET());

		Vector3d output;
		return output;
	}


	void CISSupervisor::UpdateState(
			const Vector3d& w_pos, const Vector3d& w_vel, const Vector3d& w_acc) {
		// Fill the current translational state vector
		std::cout << "Cazzo 1 " << std::endl;
		translational_state_.head(3) = w_pos;
		translational_state_.segment(3, 3) = w_vel;
		translational_state_.tail(3) = w_acc;
		std::cout << "CAzzo 2 " << std::endl;
	}


	void CISSupervisor::UpdateObstacle(const ObstacleData& ob) {
		// Update the obstacle map
		if (obst_map_.count(ob.id) > 0) {
			obst_map_[ob.id].pos = ob.pos;
			obst_map_[ob.id].vel = ob.vel;
		} else {
			obst_map_.insert(std::pair<int, ObstacleData>(ob.id, ob));
		}

		// Compute the free space
		SafeSet_ = ComputeFreeSpace(translational_state_);

		// Computing the CIS given a Safe set
		cis_gen_->computeCIS(SafeSet_, 6, 0);
		CIS_ = cis_gen_->Fetch_CIS();
	}


	VectorXd CISSupervisor::Predict_step(const Vector3d& u) {
		VectorXd output = Ad_ * translational_state_ + Bd_ * u; 
		return output;
	}

	cis2m::HPolyhedron CISSupervisor::ComputeFreeSpace(const VectorXd& x) {

		drake::geometry::optimization::ConvexSets obstacles;

		/// XXX Should use the mpa
		obstacles.emplace_back(
				drake::geometry::optimization::VPolytope::MakeBox(
					Eigen::Vector3d(-0.5, -0.5, 0.0), Eigen::Vector3d(-0.3, -0.3, 0.5)
					)
				);

		drake::geometry::optimization::HPolyhedron SafeSet_drake = drake::geometry::optimization::Iris(obstacles, translational_state_.head(3), *domain_);

		cis2m::HPolyhedron out(SafeSet_drake.A(), SafeSet_drake.b());
		return out;
	}
}

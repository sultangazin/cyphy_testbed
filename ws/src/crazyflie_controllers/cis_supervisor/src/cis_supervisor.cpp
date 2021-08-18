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

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;


// program and solution types
typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

namespace cis_supervisor {
	CISSupervisor::CISSupervisor(const MatrixXd& Ad, const MatrixXd& Bd, const MatrixXd& Ed) : Ad_(Ad), Bd_(Bd), Ed_(Ed) {
		//cis_gen_ = new cis2m::CISGenerator(Ad_, Bd_, Ed_);
	}


	CISSupervisor::CISSupervisor(const MatrixXd& A, const VectorXd& b) {
		//cis_gen_->AddDisturbanceSet(cis2m::HPolyhedron(A, b));
	}


	Vector3d CISSupervisor::UpdateControl(const Vector3d w_jerk) {
		
		w_jerk_ = w_jerk;
		VectorXd pred_state = predict_step(translational_state_, w_jerk_);

		if (CIS_.contains(pred_state)) {
			// Don't do anything, you are going to stay in the CIS
			return w_jerk_;
		} else {
			// Compute the U
			w_jerk_ = SolveOptimizationProblem(w_jerk_, CIS_.Ai(), CIS_.bi());
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
		translational_state_.block(0, 0, 3, 1) = w_pos;
		translational_state_.block(3, 0, 3, 1) = w_vel;
		translational_state_.block(6, 0, 3, 1) = w_acc;
	}


	void CISSupervisor::UpdateObstacle(const std::vector<ObstacleData>& obsts) {
		// Compute the free space
		//SafeSet_ = ComputeFreeSpace(translational_state_, obsts);

		// Computing the CIS given a Safe set
		//cis_gen_->computeCIS(SafeSet_, 6, 0);
		//CIS_ = cis_gen_->Fetch_CIS();
	}

	cis2m::HPolyhedron CISSupervisor::ComputeFreeSpace(const VectorXd& x, std::vector<ObstacleData>& obst) {
		cis2m::HPolyhedron out;
		return out;
	}
}

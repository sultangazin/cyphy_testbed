#include "cis_supervisor/cis_supervisor.hpp"
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <iomanip>

#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;

#include "drake/geometry/optimization/iris.h"
#include "drake/geometry/optimization/vpolytope.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::seq;


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
		cis_gen_ = new cis2m::CISGenerator(Ad_, Bd_);
		translational_state_ = VectorXd::Zero(Ad_.rows());
		translational_state_.head(3) = x0;
		w_jerk_ = Vector3d::Zero();
	}


	void CISSupervisor::AddSafeSet(const MatrixXd& Ss_A, const VectorXd& Ss_b) {
		std::cout << "Adding Safe Set" << std::endl;
		MatrixXd A_pos = Ss_A.leftCols(3);
		domain_ = new drake::geometry::optimization::HPolyhedron(A_pos, Ss_b);
		SafeSet_ = cis2m::HPolyhedron(Ss_A, Ss_b);
		cis_gen_->computeCIS(SafeSet_, 6, 0);
		CIS_ = cis_gen_->Fetch_CIS();
		std::cout << "CIS Initialized!" << std::endl;
	}


	void CISSupervisor::AddDisturbanceSet(const MatrixXd& Ss_A, const VectorXd& Ss_b) {
		std::cout << "Adding Disturbance Set" << std::endl;
		cis2m::HPolyhedron DistSet = cis2m::HPolyhedron(Ss_A, Ss_b);
		cis_gen_->AddDisturbanceSet(DistSet);
	}


	Vector3d CISSupervisor::UpdateControl(const Vector3d w_jerk) {
		w_jerk_ = w_jerk;

		// Predict the next state using the nominal control
		VectorXd pred_state = Predict_step(w_jerk_);

		if (CIS_.isValid()) {
			// Check if the next state would be in the CIS
			if (Contains(translational_state_, w_jerk)) {
				std::cout << "IN THE CIS (predicted)" << std::endl;
				std::cout << "I am @: " << translational_state_.transpose() << std::endl;
				std::cout << "Nom U: " << w_jerk_.transpose() << std::endl;
				std::cout << "I will be @: " << pred_state.transpose() << std::endl;
			} else {
				std::cout << "OUT THE CIS (predicted)" << std::endl;
				std::cout << "I am @: " << translational_state_.transpose() << std::endl;
				std::cout << "I will be @: " << pred_state.transpose() << std::endl;

				cis2m::HPolyhedron Acis = cis_gen_->Fetch_CIS();
				// Solve the optimization problem to stay in the CIS
				w_jerk_ = SolveOptimizationProblem(w_jerk_, Acis.Ai(), Acis.bi());
				
				// Double check that with the supervised input I stay inside the safe set
				VectorXd test_x = Predict_step(w_jerk_);

				std::cout << "After Corr. I should be @: " << test_x.transpose() << std::endl;
				std::cout << "Corr. U: " << w_jerk_.transpose() << std::endl;
				std::cout << "Acis: " << std::endl << Acis.Ai() << std::endl;
				std::cout << "Bcis: " << std::endl << Acis.bi().transpose() << std::endl;
				if (!Contains(translational_state_, w_jerk_)) {
					std::cout << " PROBLEM! The corrected input is not keeping me in the CIS..." << std::endl;
				}
			}
			std::cout << std::endl << std::endl;
		} else {
			std::cout << "CIS is not valid" << std::endl;
		}

		return w_jerk_;
	}


	// Solve the supervision problem (Find the nearest control input to the nominal one that keeps me in the CIS)
	Vector3d CISSupervisor::SolveOptimizationProblem(const Vector3d& jerk, const MatrixXd& A, const VectorXd& b) {
		const int X = 0; const int Y = 1; const int Z = 2;

		// Transform the jerk into the Brunovsky coordinates
		Vector3d nu = cis_gen_->TransformU2B(jerk, translational_state_);

		// Take the A referring to the  inputs
		// 			       |                          |
		// A is partitioned as [A_state, A_current_u, A_periodic_u]
		MatrixXd A_prime = A.rightCols(cis_gen_->GetExtendedDim());

		// Move the state part on the rhs of the equation Ax + Au < b ==> Au < b - Ax
		VectorXd b_prime = b - A.leftCols(cis_gen_->GetStateDim()) * translational_state_;

		// Prepare the optimization problem (Fill the A matrix)
		Program qp (CGAL::SMALLER, true, -100.0, true, 100.0);
		for (int i = 0; i < A_prime.rows(); i++) {
			qp.set_b(i, b_prime(i));
			for (int j = 0; j < A_prime.cols(); j++) {
				qp.set_a(j, i,  A_prime(i, j)); 
			}
		}

		// cost: x' D x + c' x + c0  (The optimization variable in our case is the u...)
		// In practice:
		// cost: <Hu, Hu> -2*<Hu, u0> + <u0, u0>
		int L = cis_gen_->GetLevel();
		qp.set_d(0, 0, 2); qp.set_d (L, L, 2); qp.set_d(2 * L, 2 * L, 2);// !!specify 2D!!
		qp.set_c(0, -2 * nu(X)); qp.set_c(L, -2 * nu(Y)); qp.set_c(2 * L, -2 * nu(Z));
		qp.set_c0(nu.norm());
		// solve the program, using ET as the exact type
		Solution s = CGAL::solve_quadratic_program(qp, ET());

		// Fetch the solution from the optimization variable
		Vector3d nu_opt(Vector3d::Zero());
		Vector3d output(Vector3d::Zero());
		if (s.is_optimal()) {
			int counter = 0;
			auto it = s.variable_values_begin();
			nu_opt(0) = CGAL::to_double(*it);
			nu_opt(1) = CGAL::to_double(*(it + 6));
			nu_opt(2) = CGAL::to_double(*(it + 12));
			
			// Map back to the original space (there is the brunovksy stuff in between)
			output = cis_gen_->TransformU2O(nu_opt, translational_state_);
		} else {
			std::cout << "Optimization failed!" << std::endl;
			std::cout << s << std::endl;
		}
		return output;
	}


	// Just update the current state...
	void CISSupervisor::UpdateState(
			const Vector3d& w_pos, const Vector3d& w_vel, const Vector3d& w_acc) {
		// Fill the current translational state vector
		translational_state_.head(3) = w_pos;
		translational_state_.segment(3, 3) = w_vel;
		translational_state_.tail(3) = w_acc;
	}


	// Update the maps of the obstacles 
	void CISSupervisor::UpdateObstacle(const ObstacleData& ob) {
		// Update the obstacle map
		if (obst_map_.count(ob.id) > 0) {
			obst_map_[ob.id].pos = ob.pos;
			obst_map_[ob.id].vel = ob.vel;
		} else {
			obst_map_.insert(std::pair<int, ObstacleData>(ob.id, ob));
		}

		// Compute the free space
		SafeSet_ = ComputeFreeSpace();

		// Computing the CIS given a Safe set
		cis_gen_->computeCIS(SafeSet_, 6, 0);
		CIS_ = cis_gen_->Fetch_CIS();
	}


	// Compute if the system is in the CIS
	bool CISSupervisor::Contains(const VectorXd& x, const VectorXd& u) {
		int stateDim = cis_gen_->GetStateDim();
		int extendedDim = cis_gen_->GetExtendedDim();
		int L = cis_gen_->GetLevel();

		std::vector<int> u0_indexes;
		for (int i = stateDim; i < stateDim + extendedDim; i = i + L) {
			u0_indexes.push_back(i);
		}

		// Select the part of the system which is known
		MatrixXd Ahd_U2B = CIS_.Ai()(Eigen::all, u0_indexes);
		MatrixXd Ahd_A = CIS_.Ai().leftCols(stateDim);

		// Select the remaining columns
		std::vector<int> rem_indexes;
		for (int i = 0; i < extendedDim; i++) {
			if (i % L != 0) {
				rem_indexes.push_back(i + stateDim);
			}
		}

		Vector3d u_br = cis_gen_->TransformU2B(u, x);
		MatrixXd A = CIS_.Ai()(Eigen::all, rem_indexes);
		VectorXd b = CIS_.bi() - Ahd_U2B * u_br - Ahd_A * x;

		// Optimization problem
		Program lp (CGAL::SMALLER, true, -100, true, 100);

		lp.set_c0(1.0);
		for (int r = 0; r < A.rows(); r++) {
			lp.set_b(r, b(r));
			for (int c = 0; c < A.cols(); c++) {
				lp.set_a(c, r, A(r, c));
			}
		}

		// solve the program, using ET as the exact type
		Solution s = CGAL::solve_linear_program(lp, ET());
		
		VectorXd output(rem_indexes.size());

		auto it = s.variable_values_begin();
		for (int i = 0; i < rem_indexes.size(); i++) {
			output(i) = (CGAL::to_double(*(it++)));
		}

		std::cout << std::setprecision(3) << (A * output).transpose() << std::endl;
		std::cout << std::setprecision(3) << b.transpose() << std::endl;

		return s.is_optimal();
	}


	// Prediction step assuming linear, discrete time model
	VectorXd CISSupervisor::Predict_step(const Vector3d& u) {
		VectorXd output = Ad_ * translational_state_ + Bd_ * u; 
		return output;
	}

	// Update the CIS given the obstacles and the drone positions
	cis2m::HPolyhedron CISSupervisor::ComputeFreeSpace() {

		drake::geometry::optimization::ConvexSets obstacles;

		/// XXX Should use the map with the obstacles obtained from the System (obst_map_)
		// For testing purposes I am putting a static object...
		obstacles.emplace_back(
				drake::geometry::optimization::VPolytope::MakeBox(
					Eigen::Vector3d(-0.5, -0.5, 0.0), Eigen::Vector3d(-0.3, -0.3, 0.5)
					)
				);

		// CALL IRIS to compute the SafeSet
		drake::geometry::optimization::HPolyhedron SafeSet_drake = drake::geometry::optimization::Iris(obstacles, translational_state_.head(3), *domain_);

		// Copy back to our format...
		MatrixXd A_out = MatrixXd::Zero(SafeSet_drake.A().rows(), Ad_.cols());
		A_out.leftCols(SafeSet_drake.A().cols()) = SafeSet_drake.A();

		// Return the Polyhedron representing the largest Safeset
		cis2m::HPolyhedron out(A_out, SafeSet_drake.b());
		return out;
	}
}

/**
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <math.h>
#include <iostream>
#include <string>
#include <fstream>

#include <ros/package.h>

#include "cis_supervisor/cis_supervisor.hpp"
#include "cis_supervisor/gurobiRoutines.hpp"

using namespace std;
using namespace Eigen;

// Read matrix from CSV to Eigen:
template<typename M>
M load_csv (const string& path) {
	ifstream indata;
	indata.open(path);
	string line;
	vector<double> values;
	uint rows = 0;
	while (getline(indata, line)) {
		stringstream lineStream(line);
		string cell;
		while (getline(lineStream, cell, ',')) {
			values.push_back(stod(cell));
		}
		++rows;
	}
	return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
}

bool check_ineq(const XType& x0, const Eigen::MatrixXd& Aineq, const Eigen::VectorXd& bineq) {
	bool guard = true;
	int k = 0;
	while (guard && k < Aineq.rows()) {
		double lhs = Aineq.row(k) * x0;
		if (lhs > bineq(k))
			guard = false;
		k++;
	}
	return guard;
}


CISSupervisor::CISSupervisor() : ctrl_outputs_(UType::Zero()),
	K_{58, 43, 10}, active_{false} {}

	CISSupervisor::~CISSupervisor() {}

	void CISSupervisor::SetInitialState(const XType& x0) {
		x0_ = x0;	
	}


void CISSupervisor::SetK(const std::array<double, CISS_STATESIZE_1D>& k) {
	std::cout << "CIS Supervisor - setting K: [ ";
	for (int i = 0; i < CISS_STATESIZE_1D; i++) {
		K_[i] = k[i];
		std::cout << K_[i] << " ";
	}
	std::cout << "]" << std::endl;
}

void CISSupervisor::LoadModel() {
	Eigen::MatrixXd Ad, Bd;
	// Load from file:
	string path = ros::package::getPath("cis_supervisor");
	Ad = load_csv<Eigen::MatrixXd>(path + "/config/data/Ad.csv");
	Bd = load_csv<Eigen::MatrixXd>(path + "/config/data/Bd.csv");
	mdl_ = new model(Ad.cols(), Bd.cols(), Ad, Bd);
}

void CISSupervisor::LoadCISs() {
	Eigen::MatrixXd cisA, cisb;
	string path = ros::package::getPath("cis_supervisor");
	for (int i=0; i < 9; i++) {
		string filename = path + "/config/data/cis" + to_string(i);
		cisA = load_csv<Eigen::MatrixXd>(filename + "_A.csv");
		cisb = load_csv<Eigen::MatrixXd>(filename + "_b.csv");
		polytope CIS(cisA, cisb);
		CISs_.add_poly(CIS);
	}

	// Input constraints:
	Eigen::MatrixXd Gu, Fu;
	Gu = load_csv<Eigen::MatrixXd>(path + "/config/data/inputA.csv");
	Fu = load_csv<Eigen::MatrixXd>(path + "/config/data/inputb.csv");
	inputPol_ = new polytope(Gu, Fu);
}


UType CISSupervisor::ComputeNominalU(const XType& err) {
	UType U(UType::Zero()); 
	for (int i = 0; i < CISS_OUTPUTSIZE; i++) {
		for (int j = 0; j < CISS_STATESIZE_1D; j++) {
			U[i] += K_[j] * err[j * CISS_STATESIZE_1D + i];
		}
	}

	return U;
}

void CISSupervisor::Step(double deltaT) {
	XType x_curr_ = x_;

	// Compute the error
	XType err = x_ref_ - x_curr_;
	UType u_des = ComputeNominalU(err);

	// Compute the nominal next state
	XType x_next_des = mdl_->Ad * x_curr_ + mdl_->Bd * u_des;

	bool cond = isContained(x_curr_);

	if (!cond) {
		cout << "I am outside the CIS! " << endl;
	}

	cond = isContained(x_next_des);
	if (cond) {
		ctrl_outputs_ = u_des;
		cout << "u_des: " << u_des.transpose() << endl;
		cout << "curr_in: " << x_curr_.transpose() << endl;
		cout << "next_in: " << x_next_des.transpose() << endl;
		cout << endl;
		x_curr_ = x_next_des;
	} else {
		// Check in which CIS the initial state is located
		vector<int> active_ciss = findCIS(x_curr_);
		int Ncis = active_ciss.size();

		vector<UType> u_cand(Ncis);
		vector<double> f_cand(Ncis);

		//cout << "It would go out!" << endl;
		for (int k = 0; k < Ncis; k++) {
			int cindex = active_ciss[k];
			cout << "Correcting in [" << cindex << "].." << endl;

			polytope ptope(CISs_.get_polA(cindex), CISs_.get_polb(cindex));
			std::pair<double, UType> res = callSupervisor(
					x_curr_, u_des, mdl_, ptope, inputPol_, 0);

			f_cand[k] = res.first;
			u_cand[k] = res.second;
		}

		// Sanity check:
		vector<double>::iterator min_el_it = std::min_element(
				f_cand.begin(), f_cand.end());

		if (min_el_it == f_cand.end() || *min_el_it == __DBL_MAX__) {
			cout << "All the optimizations failed.." << endl;
			cout << "[fail] curr_corr: " << x_curr_.transpose() << std::endl;
			cout << "[fail] next_corr: " << x_next_des.transpose() << std::endl;
			cout << "[fail] costs: [ ";
			for (auto& el : f_cand) {
				cout << el << " "; 
			}
			cout << "]" << endl;
			cout << endl;
			return;
		}
		double fval = *min_el_it;

		// Find minimum cost over all CISs:
		UType u_corrected = u_cand[min_el_it - f_cand.begin()];

		cout << u_corrected.transpose() << endl;


		// Make sure that next state is indeed in the next cis:
		XType x_next = mdl_->Ad * x_curr_ + mdl_->Bd * u_corrected;
		if (!isContained(x_next)){
			cout << "Next state not in a CIS!" << endl;
			cout << "Next: " << x_next.transpose() << std::endl;
		} else {
			cout << "Corrected!" << endl;
			cout << "curr_corr: " << x_curr_.transpose() << std::endl;
			cout << "next_corr: " << x_next.transpose() << std::endl;
			cout << endl;
		}

		ctrl_outputs_ = u_corrected;
	}
}

void CISSupervisor::getControls(UType& ctrls) const {
	ctrls = ctrl_outputs_;
}

const UType CISSupervisor::getControls() const {
	return ctrl_outputs_;
}

void CISSupervisor::SetSetpoint(const XType& xref) {
	x_ref_ = xref;
}

void CISSupervisor::SetState(const XType& x) {
	x_ = x;
}

std::pair<double, UType> CISSupervisor::callSupervisor(
		XType x_curr,
		UType u_des,
		const model* mdl,
		const polytope& CIS,
		const polytope* inputConstr,
		int method) {

	double f_cand;
	VectorXd u_cand(u_des.rows());

	// CIS inequalitites:
	MatrixXd cisA = CIS.A;
	VectorXd cisb = CIS.b;

	// wrt to u: cisA*Bd* u < cisb - cisA*Ad*x_curr
	// Simplify to box constraints:
	polytope box = simplify2box(x_curr, CIS, mdl, inputConstr);
	MatrixXd Aineq = box.A;
	VectorXd bineq = box.b.col(0);

	if (method == 0) {
		// Approach 1: Analytical solution:
		UType lb(u_des.rows());
		UType ub(u_des.rows());

		lb(0) = -bineq(0);
		lb(1) = -bineq(1);
		lb(2) = -bineq(2);

		ub(0) = bineq(3);
		ub(1) = bineq(4);
		ub(2) = bineq(5);

		if ((ub-lb).minCoeff() < 0) {  // infeasible
			cout << "Infeasible " << endl;
			f_cand = __DBL_MAX__;
		} else {
			//           lb(j), if x0(j) < lb(j),
			// x*(k) =   ub(j), if x0(j) > ub(j),
			//           x0(j), otherwise.
			for (int j=0; j < ub.rows(); j++){
				if (u_des(j) < lb(j))
					u_cand(j) = lb(j);
				else if (u_des(j) > ub(j))
					u_cand(j) = ub(j);
				else
					u_cand(j) = u_des(j);
			}
			f_cand = (u_cand - u_des).squaredNorm();
		}
	} else {
		// Approach 2: Use Gurobi:
		// Original cost: || u - udes ||^2.
		MatrixXd H = MatrixXd::Identity(mdl->Nu,mdl->Nu);
		VectorXd c = -2 * u_des;
		
		opt_result res = solveGurobi(H, c, Aineq, bineq, 0);
		if (res.solved){
		    u_cand = res.sol;
		    f_cand = res.objVal;
		}
		else{
		    f_cand = __DBL_MAX__;
		}

	}

	std::pair<double, UType> res(f_cand, u_cand);
	return res;
}


vector<int> CISSupervisor::findCIS(const XType& x0) {
	Eigen::MatrixXd Aineq;
	Eigen::VectorXd bineq;

	vector<int> output;

	for (int i = 0; i < CISs_.length(); i++) {
		Aineq = CISs_.get_polA(i);
		bineq = CISs_.get_polb(i);

		bool isCont = check_ineq(x0, Aineq, bineq);
		if (isCont) {
			output.push_back(i);
		}
	}

	return output;
}

bool CISSupervisor::isContained(const XType& x0) {

	vector<int> valid_reg = findCIS(x0);
	if (valid_reg.empty())
		return false;
	else
		return true;
}


void CISSupervisor::SetActive(bool active) {
	active_ = active;
}

bool CISSupervisor::isActive() {
	return active_;
}


// Remove redundant inequalities of box constraints.
polytope CISSupervisor::simplify2box(
		XType x0,
		const polytope& CIS,
		const model* mdl,
		const polytope* inputConstr) {
	const double abs_tol = 0.0000001;   // absolute tolerance is used by MPT to check containments (I think...).
	// Get variables:
	MatrixXd Ad = mdl->Ad;
	MatrixXd Bd = mdl->Bd;
	MatrixXd cisA = CIS.A;
	VectorXd cisb = CIS.b;
	MatrixXd Gu = inputConstr->A;
	MatrixXd Fu = inputConstr->b;
	int ulen = Gu.cols();

	// Construct linear inequality constraints:
	MatrixXd Aineq(cisA.rows()+Gu.rows(), mdl->Bd.cols());
	Aineq << cisA * mdl->Bd, Gu;
	VectorXd bineq(cisb.rows() + Fu.rows());
	bineq << cisb - cisA * mdl->Ad * x0, Fu.col(0);

	// Normalize by the non-zero value of each row:
	bool guard = false;
	for (int j=0; j<Aineq.rows(); j++){
		int sum = 0;
		int idx;
		double val;
		for (int i=0; i<Aineq.cols(); i++){
			if (Aineq(j,i)!=0){
				val = Aineq(j,i);
				sum++;
				idx = i;
			}
		}
		if (sum>1){
			cout << "More than one non-zero value at row: "<< j << ", polytope not a hyper-rectangle" << endl;
			exit(-1);
		}
		else if (sum==1){
			val = abs(Aineq(j,idx));
			Aineq(j,idx) = Aineq(j,idx)/val;
			bineq(j) = bineq(j)/val;
		}
		else    // Everything is zero, we check feasibility:
			if (bineq(j)<0 && abs(bineq(j)) > abs_tol) // infeasible.
				guard = true;
	}

	// Extract the box:
	VectorXd ub = __DBL_MAX__*VectorXd::Ones(Aineq.cols(),1);
	VectorXd lb = -__DBL_MAX__*VectorXd::Ones(Aineq.cols(),1);
	if (guard){
		ub = -VectorXd::Ones(Aineq.cols(),1);
		lb = VectorXd::Ones(Aineq.cols(),1);
	}
	else{
		for (int i=0; i<Aineq.cols(); i++){
			for (int j=0; j<Aineq.rows(); j++){
				if (Aineq(j,i)>0)
					ub(i) = min(ub(i),bineq(j));
				if (Aineq(j,i)<0)
					lb(i) = max(lb(i),-bineq(j));
			}
		}
	}

	// Construct box polytope:
	MatrixXd boxA(ulen*2, ulen);
	boxA << -MatrixXd::Identity(ulen,ulen), MatrixXd::Identity(ulen,ulen);
	MatrixXd boxb(ulen*2,1);
	boxb << -lb, ub;
	polytope box(boxA,boxb);
	return box;
}
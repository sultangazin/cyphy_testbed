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

bool check_ineq(XType& x0, const Eigen::MatrixXd& Aineq, const Eigen::VectorXd& bineq) {
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

	bool cond = isContained(x_next_des);

	if (cond || true) {
		x_curr_ = x_next_des;
		ctrl_outputs_ = u_des;
	} else {
		// Check in which CIS the initial state is located
		vector<int> active_ciss = findCIS(x_curr_);
		int Ncis = active_ciss.size();

		vector<UType> u_cand(Ncis);
		vector<double> f_cand(Ncis);

		for (int k = 0; k < Ncis; k++){
			int cis_index = active_ciss[k];

			// CIS inequalitites:
			//
			Eigen::MatrixXd cisA = CISs_.get_polA(cis_index);
			Eigen::VectorXd cisb = CISs_.get_polb(cis_index).col(0);

			cout << "Correcting.." << endl;

			// wrt to u:
			// cisA*Bd* u < cisb - cisA*Ad*x_curr
			Eigen::MatrixXd Aineq(cisA.rows() + inputPol_->A.rows(), mdl_->Bd.cols());
			Aineq << cisA * mdl_->Bd, inputPol_->A;
			Eigen::VectorXd bineq(cisb.rows() + inputPol_->b.rows());
			bineq << cisb - cisA*mdl_->Ad * x_curr_, inputPol_->b.col(0);
			// Original cost: || u - udes ||^2.
			Eigen::MatrixXd H = Eigen::MatrixXd::Identity(mdl_->Nu,mdl_->Nu);
			Eigen::VectorXd c = -2 * u_des;

			// Call solver:
			opt_result res = solveGurobi(H, c, Aineq, bineq, 1);

			if (res.solved){
				u_cand[k] = res.sol;
				f_cand[k] = res.objVal;
			}
			else{
				f_cand[k] = __DBL_MAX__;
			}
		}

		// Sanity check:
		vector<double>::iterator it = std::min_element(f_cand.begin(), f_cand.end());
		
		if (it == f_cand.end() || *it == __DBL_MAX__){
			cout << "All the optimizations returned NaN.." << endl;
			return;
		}

		// Find minimum cost over all CISs:
		vector<double>::iterator min_el_it = std::min_element(f_cand.begin(), f_cand.end());
		double fval = *min_el_it;
		UType u_corrected = u_cand[min_el_it - f_cand.begin()];

		// Make sure that next state is indeed in the next cis:
		XType x_next = mdl_->Ad * x_curr_ + mdl_->Bd * u_corrected;
		if (!isContained(x_next)){
			cout << "Next state not in a CIS!" << endl;
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

vector<int> CISSupervisor::findCIS(XType& x0) {
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

bool CISSupervisor::isContained(XType& x0) {

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

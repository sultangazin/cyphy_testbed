/*  Copyright (C) 2020, Tzanis Anevlavis.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.  */

#ifndef _GUROBIROUTINES_H
#define _GUROBIROUTINES_H
 
#include <vector>
#include <gurobi_c++.h>
#include <Eigen/Dense>

struct opt_result{
    bool solved;
    Eigen::VectorXd sol;
    double objVal;
    int status;
    // Constructor:
    opt_result(bool b=false, double val=0.0, Eigen::VectorXd vec = Eigen::VectorXd::Zero(0), int s=0) : solved(b), objVal(val), sol(vec), status(s) {};
};

void addVarsGRB(GRBModel& model, std::vector<GRBVar>& u, double lb, double ub, int size);

void addConstrsGRB(GRBModel& model, Eigen::MatrixXd A, Eigen::VectorXd b, std::vector<GRBVar> u);

void addObjectiveGRB(GRBModel& model, Eigen::MatrixXd H, Eigen::VectorXd c, std::vector<GRBVar> u);

opt_result solveGurobi(Eigen::MatrixXd H,
		Eigen::VectorXd c,
		Eigen::MatrixXd Aineq,
		Eigen::VectorXd bineq, int verbose);

#endif

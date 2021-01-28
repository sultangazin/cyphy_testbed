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
 
#include <iostream>
#include "cis_supervisor/gurobiRoutines.hpp"

using namespace std;
using namespace Eigen;

void addVarsGRB(GRBModel& model, vector<GRBVar>& u, double lb, double ub, int size){
    GRBVar tmp_u;
    string str = "u";
    for (int i = 0; i < size; i++) {
        string tmp_s = str + to_string(i);
        tmp_u = model.addVar(lb, ub, 0.0, GRB_CONTINUOUS, tmp_s);
        u.push_back(tmp_u);
    }
}

void addConstrsGRB(GRBModel& model, MatrixXd A, VectorXd b, vector<GRBVar> u){
    string str = "c";
    for (int i = 0; i < A.rows(); i++){
        string tmp_s = str + to_string(i);
        model.addConstr(A(i,0)*u[0] + A(i,1)*u[1] + A(i,2)*u[2] <= b(i), tmp_s);
    }
}

void addObjectiveGRB(GRBModel& model, MatrixXd H, VectorXd c, vector<GRBVar> u){
    GRBQuadExpr obj = 0.0;
    for (int i=0; i < H.rows(); i++)
        for (int j=0; j < H.cols(); j++)
            obj = obj + u[i]*H(i,j)*u[j];
    
    for (int i=0; i<c.rows(); i++)
        obj = obj + c(i)*u[i];
    
    model.setObjective(obj);
}


opt_result solveGurobi(MatrixXd H, VectorXd c, MatrixXd Aineq, VectorXd bineq, int verbose){
    try {
        // Create environment and model:
        GRBEnv env = GRBEnv();
        // Deactivate presolve:
        env.set(GRB_IntParam_Presolve,0);
        if (verbose<0 || verbose>1)
            exit(-1);
        env.set(GRB_IntParam_OutputFlag,verbose);
        GRBModel model = GRBModel(env);

        // Set model parameters:
	/*
        model.set(GRB_DoubleParam_FeasibilityTol, 0.000000001);
        model.set(GRB_DoubleParam_OptimalityTol, 0.000000001);
        model.set(GRB_IntParam_Method,-1); // primal simplex.
	*/

        // Create variables:
        vector<GRBVar> u;
        //addVarsGRB(model, u, __DBL_MIN__, __DBL_MAX__, 3);
	addVarsGRB(model, u, -60.0, 60.0, 3);


        // Set objective:
        addObjectiveGRB(model, H, c, u);
        // Add constraints:
        addConstrsGRB(model, Aineq, bineq, u);
        
        cout << Aineq << endl;
        cout << bineq << endl;

        // Optimize model
        model.optimize();
        cout << u[0].get(GRB_StringAttr_VarName) << " "
             << u[0].get(GRB_DoubleAttr_X) << endl;
        cout << u[1].get(GRB_StringAttr_VarName) << " "
             << u[1].get(GRB_DoubleAttr_X) << endl;
        cout << u[2].get(GRB_StringAttr_VarName) << " "
             << u[2].get(GRB_DoubleAttr_X) << endl;
        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
        VectorXd sol(u.size());

	model.update();
	model.write("debug.qp");

        for (int i=0; i<u.size(); i++)
            sol[i] = u[i].get(GRB_DoubleAttr_X);
        
        int status = model.get(GRB_IntAttr_Status);
        
        opt_result res((status==2), 0.0, sol, status);
        
        if (status==2)  // solved to optimality.
            res.objVal = model.get(GRB_DoubleAttr_ObjVal);
        else
            res.objVal = __DBL_MAX__;
        
        return res;
    }
    catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    catch(...) {
        cout << "Exception during optimization" << endl;
    }
    
    return {};
}

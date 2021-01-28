/*  Copyright (C) 2020, Tzanis Anevlavis, Luigi Pannocchi.

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

#pragma once

#include <vector>
#include <Eigen/Dense>

struct model {
    int Nx;
    int Nu;
    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
    // Constructor:
    model(int n, int m, Eigen::MatrixXd A, Eigen::MatrixXd B) : Nx(n), Nu(m), Ad(A), Bd(B) {};
};

struct polytope {
	Eigen::MatrixXd A;
	Eigen::MatrixXd b;
	// Constructor:
	polytope(Eigen::MatrixXd A, Eigen::MatrixXd b) : A(A), b(b) {};
};

class unionPoly {
	public:
		// Constructor:
		unionPoly() : sets() {}
		// Add a polytope:
		void add_poly(polytope set){ sets.push_back(set); };
		// Access polytopes:
		polytope get_poly(int idx){ return sets[idx]; };
		Eigen::MatrixXd get_polA(int idx){ return sets[idx].A; };
		//    VectorXd get_polb(int idx){ return sets[idx].b; };
		Eigen::MatrixXd get_polb(int idx){ return sets[idx].b; };
		// Get size of union:
		int length(){ return sets.size(); };
	private:
		std::vector<polytope> sets;
};

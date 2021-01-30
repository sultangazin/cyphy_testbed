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

#include <Eigen/Dense>
#include <vector>
#include <utility> 
#include "cis_supervisor/cis_supervisor_data.hpp"

#define CISS_OUTPUTSIZE (3)
#define CISS_STATESIZE_1D (3)
#define CISS_NUMDIMS (3)
#define CISS_STATESIZE (CISS_STATESIZE_1D * CISS_NUMDIMS)

typedef Eigen::Matrix<double, CISS_OUTPUTSIZE, 1> UType;
typedef Eigen::Matrix<double, CISS_STATESIZE, 1> XType;

class CISSupervisor {
	public:
		CISSupervisor();
		~CISSupervisor(); 

		void SetActive(bool active);
		bool isActive();


		void SetInitialState(const XType& x0);
		void SetSetpoint(const XType& xref);
		void SetState(const XType& x);

		void SetK(const std::array<double, CISS_STATESIZE_1D>& k);

		void LoadModel();
		void LoadCISs();

		void Step(double T); 
		void getControls(UType& ctrls) const;
		const UType getControls() const;

	private:
		bool active_;

		XType x0_;

		XType x_ref_;
		XType x_;

		std::array<double, CISS_STATESIZE_1D> K_;

		UType ctrl_outputs_;

		// CIS Polytopes
		unionPoly CISs_;

		// Control Constratints Polytope
		polytope* inputPol_;

		// Model
		model* mdl_;

		bool isContained(XType& x);
		std::vector<int> findCIS(XType& x);

		UType ComputeNominalU(const XType& err);

		std::pair<double, UType> callSupervisor(
				XType x_curr,
				UType u_des,
				const model* mdl,
				const polytope& CIS,
				const polytope* inputConstr);

		polytope simplify2box(
				XType x0,
				const polytope& CIS,
				const model* mdl,
				const polytope* inputConstr);
};

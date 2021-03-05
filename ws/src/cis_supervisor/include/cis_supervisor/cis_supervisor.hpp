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
		bool isControlActive();
		bool isSupervisionActive();

		void SetInitialState(const XType& x0);
		void SetSetpoint(const XType& xref);
		void SetCtrlFF(const UType& ff);
		void SetState(const XType& x);
		void SetQuat(const Eigen::Quaterniond& q);

		void SetK(const std::array<double, CISS_STATESIZE_1D>& k);
		void SetKyaw(double d);

		void LoadModel();
		void LoadCISs();

		bool Step(double T); 
		void getControls(UType& ctrls) const;
		const UType getControls() const;
		double getYawCtrl();

		const XType getNextState() const;

	private:
		bool active_;
		bool supervision_active_;

		XType x0_;

		XType state_ref_;
		XType state_;
		XType state_next_des_;

		UType ctrl_ff_;

		Eigen::Quaterniond quat_;

		std::array<double, CISS_STATESIZE_1D> K_;
		double kr_rates_;

		UType ctrl_outputs_;
		double ctrl_yaw_;

		// CIS Polytopes
		unionPoly CISs_;
		unionPoly RCISs_;

		// Control Constratints Polytope
		polytope* inputSet_;

		// Model
		model* mdl_;

		bool isContained(const XType& x);
		std::vector<int> findCIS(const XType& x);

		UType ComputeNominalU(const XType& err);
		double ComputeYawCtrl(const Eigen::Vector3d& acc_d,
				const Eigen::Vector3d& acc);

		std::pair<double, UType> callSupervisor(
				XType x_curr,
				UType u_des,
				const model* mdl,
				const polytope& CIS,
				const polytope* inputSet,
				int method);

		polytope simplify2box(
				XType x0,
				const polytope& CIS,
				const model* mdl,
				const polytope* inputSet);
};

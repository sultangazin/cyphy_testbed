/**
 * @author Luigi Pannocchi
 * @file flatofilter.hpp
 */

#pragma once

#include <Eigen/Dense>
#include <mutex>

const int STATE_DIM(9);
const int INPUT_DIM(3);
const int MEAS_DIM(3);

// Redefinition of types to reduce the cluttering...
typedef Eigen::Matrix<double, STATE_DIM, 1> XMat;
typedef Eigen::Matrix<double, INPUT_DIM, 1> UMat;
typedef Eigen::Matrix<double, MEAS_DIM, 1> YMat;

typedef Eigen::Matrix<double, STATE_DIM, MEAS_DIM> KMat;

class  Filter {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		virtual ~Filter() {};

		virtual void setGain(const KMat& k) = 0;

		virtual void reset(const XMat& x0) = 0;
		virtual void resetPosition(const Eigen::Vector3d& p0) = 0;
		virtual void prediction(double dt) = 0;
		virtual void setU(const UMat& u) = 0;
		virtual void update(const YMat&) = 0;

		// Fetchers
		virtual const Eigen::Vector3d getPos() const = 0;
		virtual const Eigen::Vector3d getVel() const = 0;
		virtual const Eigen::Vector3d getAcc() const = 0;
};

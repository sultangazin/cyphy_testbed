/**
 * @author Luigi Pannocchi
 * @file lbfilter.hpp
 */

#pragma once

#include <Eigen/Dense>
#include <mutex>

#include "filter/filter.hpp"

class LBFilter : public Filter {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		LBFilter(const Eigen::Vector3d& p0, double dt);
		~LBFilter();

		void setGain(const KMat& k);

		void reset(const XMat& x0);
		void resetPosition(const Eigen::Vector3d& p0);
		void prediction(double dt);
		void setU(const UMat& u);
		void update(const YMat&);

		// Fetchers
		virtual const Eigen::Vector3d getPos() const;
		virtual const Eigen::Vector3d getVel() const;
		virtual const Eigen::Vector3d getAcc() const;

	private:
		double Ts_;

		mutable std::mutex mx_;

		XMat state_;
		UMat input_;
		YMat meas_;

		KMat FilterGain_;

		void setPos(const Eigen::Vector3d& p);
		void setVel(const Eigen::Vector3d& v);
		void setAcc(const Eigen::Vector3d& a);

		void setPosNoBlk(const Eigen::Vector3d& p);
		void setVelNoBlk(const Eigen::Vector3d& v);
		void setAccNoBlk(const Eigen::Vector3d& a);

		/**
		 * Fetcher Private
		 */
		const Eigen::Vector3d Pos() const;
		const Eigen::Vector3d Vel() const;
		const Eigen::Vector3d Acc() const;

};

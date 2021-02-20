/**
 * @author Luigi Pannocchi
 * @file ddfilter.hpp
 *
 * Data Driven filter:
 * It consider the target to have a dynamics of 
 * a chain of integrator driven by an unkown input.
 *
 * The state of the filter is composed by:
 * Position (3) + Velocity (3) + Acceleration (3) + Jerk(3)
 * The measurement consist in the target Position (3)
 */

#ifndef _DDFILTER_HPP_
#define _DDFILTER_HPP_

#include <Eigen/Dense>
#include <queue>
#include <mutex>

#define DDFILTER_STATE_DIM (9)
#define DDFILTER_INPUT_DIM (3)
#define DDFILTER_MEAS_DIM (3)

// Redefinition of types to reduce the cluttering...
typedef Eigen::Matrix<double, DDFILTER_STATE_DIM, 1> DDXMat;
typedef Eigen::Matrix<double, DDFILTER_INPUT_DIM, 1> DDUMat;
typedef Eigen::Matrix<double, DDFILTER_MEAS_DIM, 1> DDYMat;

typedef Eigen::Matrix<double, DDFILTER_STATE_DIM, DDFILTER_STATE_DIM> DDDynMat;
typedef Eigen::Matrix<double, DDFILTER_MEAS_DIM, DDFILTER_STATE_DIM> DDCMat;


class DDFilter {
	public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		DDFilter();

		DDFilter(const Eigen::Vector3d& p0, int Nsteps,	double dt);
		~DDFilter();

		void resetPosition(const Eigen::Vector3d& p0);

		void prediction(double dt);

		void setU(const DDUMat& u);

		void setSteps(int n);

		void update(const DDYMat&, double timestamp);

		// Fetchers
		DDXMat getState() const;
        const Eigen::Vector3d getPos() const;
        const Eigen::Vector3d getVel() const;
        const Eigen::Vector3d getAcc() const;

    private:
        mutable std::mutex _mx;

		int _Nsteps;
		std::queue<DDYMat> _YQueue;
        std::queue<double> _tQueue;

		DDXMat _x;
		DDUMat _u;
		DDYMat _y;

		double _T;
		
		DDDynMat _A;
		DDCMat _C;

		void updateSampleTime(double dt);

		void setPos(const Eigen::Vector3d& p);
        void setVel(const Eigen::Vector3d& v);
        void setAcc(const Eigen::Vector3d& a);



		/**
		 * Fetcher Private
		 */
		const Eigen::Vector3d ExtractPos(const DDXMat& x) const;
        const Eigen::Vector3d ExtractVel(const DDXMat& x) const;
        const Eigen::Vector3d ExtractAcc(const DDXMat& x) const;

        /**
         * Compute the matrix of the dynamcs.
         */
        DDDynMat computeA(double dt);
		DDCMat computeC();

};
#endif

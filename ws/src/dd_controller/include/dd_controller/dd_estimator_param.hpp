/*
 *
 * Copyright (c) 2020 Luigi Pannocchi 
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
 * estimatorDD_data.h - DataDriven Estimator Data Structures 
 *
 */
#ifndef __ESTIMATOR_DD_PARAM_H__
#define __ESTIMATOR_DD_PARAM_H__

#include <Eigen/Dense>
#include "dd_controller/dd_estimator.hpp"

#define DDESTPAR_STATE1DSIZE (3)

#define DDESTPAR_NVAR2D (4)
#define DDESTPAR_INPUT2DSIZE (4)
#define DDESTPAR_ALPHA2DSIZE (DDESTPAR_INPUT2DSIZE)
#define DDESTPAR_BETA2DSIZE (DDESTPAR_INPUT2DSIZE * DDESTPAR_INPUT2DSIZE)
#define DDESTPAR_GAINS2DSIZE (DDESTPAR_ALPHA2DSIZE + DDESTPAR_BETA2DSIZE)

typedef Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, 1>  alpha2d_t;
typedef Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, DDESTPAR_INPUT2DSIZE>  beta2d_t; 


/**
 * Data structure for a better export 
 * of the paramters
 */
typedef struct DDParams_s DDParams;
struct DDParams_s {
	bool valid;
	
	double alpha_x;
	double alpha_y;
	
	double beta_x;
	double beta_y;

	alpha2d_t alpha2d;
	beta2d_t beta2d;
};


/**
 * Data structures for the parameters 
 */
class DDParamEstimator1D {
	public:
		DDParamEstimator1D();

		// Estimation Step
		void Step(double state_acc, double input, double T);

		void SetGains(const std::array<double, 2>& gains);

		void SetBetaBounds(const std::array<double, 2>& bbounds);

		void SetParams(double alpha, double beta);

		void GetParams(double* alpha, double* beta);

		void Reset();

	private:
		double alpha_;
		double beta_;

		// Estimator Gains
        std::array<double, 2> est_gains_;

		// Bounds on the beta parameter
        std::array<double, 2> beta_bounds_;
};




/**
 * Data structure for the 2D parameter estimator 
 */
class DDParamEstimator2D {
		public:
				/**
				 * Initialization and reset (Not changing the gains)
				 */
				DDParamEstimator2D();
				~DDParamEstimator2D();

				/**
				 * Reset Step
				 */
				void Reset();

				/**
				 * Estimation step
				 */
				void Step(
                        const Eigen::Matrix<double, DDESTPAR_NVAR2D, 1>& est_acc,
                        const Eigen::Matrix<double, DDESTPAR_INPUT2DSIZE, 1>& input,
                        double deltaT);
				/**
				 * Set estimator gains
				 */
				void SetGains(
                        const std::array<double, DDESTPAR_ALPHA2DSIZE>& alpha_gains,
                        const std::array<double, DDESTPAR_BETA2DSIZE>& beta_gains);

				/**
				 * Get estimated params
				 */
				void GetParams(alpha2d_t& a, beta2d_t& b);

				/**
				 * Set params
				 */
				void SetParams(const alpha2d_t& a, const beta2d_t& b);

				/**
				 * Set Bounds
				 */
				void SetBetaBounds(
								const std::array<double, DDESTPAR_BETA2DSIZE>& blbounds,
								const std::array<double, DDESTPAR_BETA2DSIZE>& bubounds);


		private:
				alpha2d_t alpha_;
				beta2d_t beta_;

				Eigen::Array<double, DDESTPAR_ALPHA2DSIZE, 1>  kest_alpha;
				Eigen::Array<double, DDESTPAR_ALPHA2DSIZE, DDESTPAR_INPUT2DSIZE> kest_beta;

				std::array<double, DDESTPAR_BETA2DSIZE> beta_lbounds_;
				std::array<double, DDESTPAR_BETA2DSIZE> beta_ubounds_;

				void SetBetaLBounds(const std::array<double, DDESTPAR_BETA2DSIZE>& bounds);
				void SetBetaUBounds(const std::array<double, DDESTPAR_BETA2DSIZE>& bounds);

};


/**
 * Parameter Estimator FULL 
 * I am using std::arrays for parameters and gains to facilitate the usage in a ROS environment.
 */
class DDParamEstimator {
		public:
				/**
				 * Reset and init function of the parameter estimator
				 */
				DDParamEstimator();
				~DDParamEstimator();

				void Reset();

				/**
				 * Perform a estimation step
				 */
				void Step(state_t* ps,
								const Eigen::Matrix<double, DDESTPAR_INPUT2DSIZE, 1>& input,
								double deltaT);

				/**
				 * Set the parameter estimators gains
				 */
				void SetGains(const std::array<double, 2>& gains_x, const std::array<double, 2>& gains_y,
								const std::array<double, DDESTPAR_ALPHA2DSIZE>& gains_alpha2d,
								const std::array<double, DDESTPAR_BETA2DSIZE>& gains_beta2d);

				/**
				 * Set the data structure of the parameters from the estimator
				 */
				void SetParams(const DDParams& pa);

				/**
				 * Set the bound for the beta parameters
				 */
				void SetBounds(const std::array<double, 2>& beta_x, const std::array<double, 2>& beta_y,
								const std::array<double, DDESTPAR_BETA2DSIZE>& blbounds,
								const std::array<double, DDESTPAR_BETA2DSIZE>& bubounds);

                /**
				 * Get the data structure of the parameters from the estimator
				 */
				DDParams GetParams();


		private:
				// Measurement Data
				DDParamEstimator1D paramest1D[2];
				DDParamEstimator2D paramest2D;

				DDParams params_;

				bool initialized;
};

#endif //__ESTIMATOR_DD_DATA_H__

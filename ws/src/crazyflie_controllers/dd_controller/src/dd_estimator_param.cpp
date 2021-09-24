#include "dd_controller/dd_estimator_param.hpp"
#include <algorithm>
#include <iostream>


// ======================================================
//			DATA BUFFERS
// ======================================================


// ======================================================

//
// DDESTIMATOR1D
//
DDParamEstimator1D::DDParamEstimator1D() {
		alpha_ = 0.0;
		beta_ = 0.0;
}

void DDParamEstimator1D::Step(
				double state_acc,
				double input, double T) {
		// Get the current value from the global variables
		double alpha = alpha_;
		double beta = beta_;

		// Local variables
		double alpha_new = 0.0f;
		double beta_new = 0.0f;

		double sqrt_T = sqrtf(T);

		//Compute Updates
		alpha_new = alpha +
				est_gains_[0] * sqrt_T * (state_acc - (alpha + beta * input));
		beta_new = beta +
				est_gains_[1] * sqrt_T * input * (
								state_acc - (alpha + beta * input));

		beta_new = std::clamp(beta_new, beta_bounds_[0], beta_bounds_[1]);

		//Update global variables
		alpha_ = alpha_new;
		beta_ = beta_new;
}

void DDParamEstimator1D::SetGains(const std::array<double, 2>& gains) {
    est_gains_ = gains;
}

void DDParamEstimator1D::SetBetaBounds(const std::array<double, 2>& bbounds) {
	beta_bounds_ = bbounds;
}


void DDParamEstimator1D::GetParams(double* alpha, double* beta) {
		*alpha = alpha_;
		*beta = beta_;
} 

void DDParamEstimator1D::SetParams(double alpha, double beta) {
		alpha_ = alpha;
		beta_ = beta;
}


void DDParamEstimator1D::Reset() {
		alpha_ = 0;
		beta_ = 0;
}


//
// DDESTIMATOR2D
//
DDParamEstimator2D::DDParamEstimator2D() {
}

DDParamEstimator2D::~DDParamEstimator2D() {
}

void DDParamEstimator2D::Reset() {
	alpha_ = Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, 1>::Zero();	
	beta_ = Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, DDESTPAR_INPUT2DSIZE>::Zero();
}

void DDParamEstimator2D::Step(
				const Eigen::Matrix<double, DDESTPAR_NVAR2D, 1>& est_acc,
				const Eigen::Matrix<double, DDESTPAR_INPUT2DSIZE, 1>& input,
				double deltaT) {

		// Compute the estimation error of the acceleration
		// using the current estimated parameter alpha and beta.
		Eigen::Matrix<double, DDESTPAR_NVAR2D, 1>  acc_est_err = 
				est_acc - (alpha_ + beta_ * input);

		// Update the parameters 
		// (I make use of the array() method because operation
		// are element-wise)

		double sqrt_T = sqrtf(deltaT);
		// Alphas
		// a_ = a + K * sqrt(T) * (est_error)
		// (On the right I have element-wise operations)
		alpha_ = alpha_.array() + kest_alpha * sqrt_T * acc_est_err.array();

		// Betas
		// On the rows:
		// z; roll; pitch; yaw
		for (int i = 0; i < DDESTPAR_ALPHA2DSIZE; i++) {
				// Update 'i' row of the Beta matrix 
				double e_i = acc_est_err(i) * sqrt_T;
				beta_.block<1, DDESTPAR_INPUT2DSIZE>(i, 0) = 
					beta_.block<1, DDESTPAR_INPUT2DSIZE>(i, 0).array() +
					kest_beta.block<1, DDESTPAR_INPUT2DSIZE>(i, 0).array() * input.transpose().array() * e_i;
		}

		// Check bound of Betas
		for (int i = 0; i < DDESTPAR_BETA2DSIZE; i++) {
			beta_(i) = std::clamp(beta_(i), beta_lbounds_[i], beta_ubounds_[i]);
		}
}


void DDParamEstimator2D::SetGains(
				const std::array<double, DDESTPAR_ALPHA2DSIZE>& alpha_gains,
				const std::array<double, DDESTPAR_BETA2DSIZE>& beta_gains) {
		// Set Alpha Gains
		for (int i = 0; i < DDESTPAR_ALPHA2DSIZE; i++) {
				kest_alpha(i) = alpha_gains[i];
		}
		// Set Beta Gains
		for (int i = 0; i < DDESTPAR_BETA2DSIZE; i++) {
				kest_beta(i) = beta_gains[i];
		}
}

void DDParamEstimator2D::SetBetaLBounds(
				const std::array<double, DDESTPAR_BETA2DSIZE>& bbounds) {
				beta_lbounds_ = bbounds;
}

void DDParamEstimator2D::SetBetaUBounds(
				const std::array<double, DDESTPAR_BETA2DSIZE>& bbounds) {
				beta_ubounds_ = bbounds;
}

void DDParamEstimator2D::SetBetaBounds(
				const std::array<double, DDESTPAR_BETA2DSIZE>& blbounds,
				const std::array<double, DDESTPAR_BETA2DSIZE>& bubounds) {
		SetBetaLBounds(blbounds);
		SetBetaUBounds(bubounds);
}
/*
void DDParamEstimator2D::GetParams(
        Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, 1>& a,
        Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, DDESTPAR_INPUT2DSIZE>& b) {
		a = alpha_;
		b = beta_;
}
*/

Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, 1> DDParamEstimator2D::GetAlphaParams() {
    return alpha_;
}

Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, DDESTPAR_INPUT2DSIZE> DDParamEstimator2D::GetBetaParams() {
    return beta_;
}


void DDParamEstimator2D::SetParams(const Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, 1>& a, const Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, DDESTPAR_INPUT2DSIZE>& b) {
		alpha_ = a;
		beta_ = b;
}




// 
// DDPARAMESTIMATOR
//
DDParamEstimator::DDParamEstimator() {
		SetGains(
						{0.005, 0.1},
						{0.005, 0.1},
						{0.002, 0.002, 0.002, 0.002},
						{ 0.0001, 0.0005, 0.0001, 0.0001,
						0.0001, 0.0001, 0.0001, 0.0001,
						0.0001, 0.0001, 0.0001, 0.0001,
						0.0001, 0.0001, 0.0001, 0.0001}
				);

		DDParams pa;
		pa.valid = false;
		pa.alpha_x = 0;
		pa.alpha_y = 0;
	
		pa.beta_x = 4;
		pa.beta_y = -4;

		pa.alpha2d = Eigen::Matrix<double, DDESTPAR_ALPHA2DSIZE, 1>(-18.0, 0, 0, 0);
		pa.beta2d << 
				5, 5, 5, 5,
				-50, -50, 50, 50,
				-50, 50, 50, -50,
				50, -50, 50, -50;

		SetParams(pa);


		SetBounds(
						{1.0, 18.0},
						{-18.0, -1.0},
						{1,1,1,1,
						-100, -100, 1, 1,
						-100, 1, 1, -100,
						1, -100, 1, -100},
						{ 10,10,10,10,
						-1, -1, 100, 100,
						-1, 100, 100, -1,
						100, -1, 100, -1}
				 );

}

void DDParamEstimator::Reset() {
	paramest1D[0].Reset();
	paramest1D[1].Reset();
	paramest2D.Reset();
}


/** 
 * Estimation step
 */
void DDParamEstimator::Step(state_t* ps,
				const Eigen::Matrix<double, DDESTPAR_INPUT2DSIZE, 1>& input,
				double deltaT) {

		// Estimate the paremeters on X
		double state_accx = ps->acceleration(0);
		double input_x = ps->attitude(1); 
		paramest1D[0].Step(state_accx,
						input_x, deltaT);

		// Estimate the paremeters on Y
		double state_accy = ps->acceleration(1);
		double input_y = ps->attitude(0);
		paramest1D[1].Step(state_accy,
						input_y, deltaT);

		// Estimate the paremeters on the rest
		Eigen::Matrix<double, DDESTPAR_NVAR2D, 1> state_acczatt(
						ps->acceleration(2), ps->attitude_dd(0),
						ps->attitude_dd(1), ps->attitude_dd(2));

		paramest2D.Step(state_acczatt, input, deltaT);

		// Update the internal structure taking the estimate from the
        // components.
		paramest1D[0].GetParams(&params_.alpha_x, &params_.beta_x);
		paramest1D[1].GetParams(&params_.alpha_y, &params_.beta_y);
        params_.alpha2d = paramest2D.GetAlphaParams();
        params_.beta2d = paramest2D.GetBetaParams();
		//paramest2D.GetParams(, params_.beta2d);
		
		params_.valid = true;
}

void DDParamEstimator::SetGains(const std::array<double, 2>& gains_x, const std::array<double, 2>& gains_y,
				const std::array<double, DDESTPAR_ALPHA2DSIZE>& gains_alpha2d,
				const std::array<double, DDESTPAR_BETA2DSIZE>& gains_beta2d) {
		paramest1D[0].SetGains(gains_x);
		paramest1D[1].SetGains(gains_y);
		paramest2D.SetGains(gains_alpha2d, gains_beta2d);
}

void DDParamEstimator::SetParams(const DDParams& pa) {
		params_ = pa;

		paramest1D[0].SetParams(pa.alpha_x, pa.beta_x);

		paramest1D[1].SetParams(pa.alpha_y, pa.beta_y);

		paramest2D.SetParams(pa.alpha2d, pa.beta2d);
}

void DDParamEstimator::SetBounds(const std::array<double, 2>& beta_x, const std::array<double, 2>& beta_y,
				const std::array<double, DDESTPAR_BETA2DSIZE>& blbounds,
				const std::array<double, DDESTPAR_BETA2DSIZE>& bubounds) {

		paramest1D[0].SetBetaBounds(beta_x);
		paramest1D[1].SetBetaBounds(beta_y);

		paramest2D.SetBetaBounds(blbounds, bubounds);
}

DDParams DDParamEstimator::GetParams() {
		return params_;
}

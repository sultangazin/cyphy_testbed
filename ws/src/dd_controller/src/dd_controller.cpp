#include "Eigen/Dense"
#include "dd_controller/dd_controller.hpp"
#include "geometry_msgs/Vector3Stamped.h"

using namespace Eigen;

DDController::DDController() {
    isInit_ = false;
};

DDController::~DDController() {};

/**
 * Initialize the Class
 */
bool DDController::Initialize(const ros::NodeHandle& n) {

    ros::NodeHandle nl(n);

    // Compose the name
    name_ = ros::this_node::getName().c_str();

    // Load parameters
    if (!LoadParameters(nl)) {
        ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
        return false;
    }

    // Register callback
    if (!RegisterCallbacks(nl)) {
        ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
        return false;
    }

    // Advertise topics
    dd_state_pub_ = 
        nl.advertise<geometry_msgs::Vector3Stamped> ("cf1/dd_estimate", \
                10);

    estimatorDDInit();

    msg_counter_ = 0;

    return true;
}

bool DDController::LoadParameters(const ros::NodeHandle& n) {

    ros::NodeHandle np("~");
    
    // Parameter loading here

    return true;
}

bool DDController::RegisterCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);
    sens_channel_ = nl.subscribe("/cf1/external_position", 
            15, 
            &DDController::estimatorDDNewMeasurement, this);

    return true;
}


void DDController::eval_pseudoinv(
        const Matrix<float, BUFF_SIZE, STATE_SIZE>& Obs, 
		Matrix<float, STATE_SIZE, BUFF_SIZE>& Pseudo) {
	
	Matrix<float, STATE_SIZE, BUFF_SIZE> Otrans;
	Otrans = Obs.transpose();	

    // (O' x O)^-1 x O' = Pseudo inverse
	Pseudo = (Otrans * Obs).inverse() * Otrans; 
}



/**
 * Estimate state
 */
void DDController::estimate_state(
        const Matrix<float, STATE_SIZE, BUFF_SIZE>& Oinv, 
		const Matrix<float, BUFF_SIZE, 1>& Y,
        Matrix<float, STATE_SIZE, 1>& Xest) {

	/*
	// Save the old state before the update
	for (int i = 0; i < 3; i++) {
		X_old[i] = X[i];
	}
	*/

	Xest = Oinv * Y;

    return;
}


/**
 * Estimate params
 */
void DDController::estimate_params() {
    float alpha_new;

	if (ctrl_dd_active_) {
		Step_++;
		switch (Step_) {
			case 1:
				alpha_new = alpha_ - (alpha_ + ctrl_dd_ * beta_) +  1.0f/TotalTime_ * (X_[1] - X_old[1]); 
				beta_ = beta_;
				alpha_ = alpha_new; 
				u_ = 41000.0f;
				estimatorDDSetControl(u_);
				break;
			case 2:
				alpha_new = alpha_ + ctrl_ddd_ / (TotalTime_ * (ctrl_ddd_ - ctrl_dd_))* ((X_[1] - X_old[1]) - TotalTime_ * (alpha_ + ctrl_dd_ * beta_)); 
				beta_ = beta_ + ctrl_ddd_ / (TotalTime_ * (ctrl_dd_ - ctrl_ddd_)) * ((X_[1] - X_old[1]) - TotalTime_ * (alpha_ + ctrl_dd_ * beta_));
				alpha_ = alpha_new;
				break;  
			default:
				alpha_new = alpha_ - gamma1_ * (TotalTime_) * (alpha_ + ctrl_dd_ * beta_) +  gamma1_ * (X_[1] - X_old[1]);
				// beta_ = beta_ - gamma1_ * ctrl_dd_ * TotalTime_ * (alpha_ + ctrl_dd_ * beta_) +  gamma1_ * ctrl_dd_* (X_[1] - X_old[1]);
				alpha_ = alpha_new;
				break;  
		}
	}
	else {
		alpha_new = alpha_ - gamma1_ * (TotalTime_) * (alpha_ + ctrl_dd_ * beta_) +  gamma1_ * (X_[1] - X_old[1]);
		// beta_ = beta_ - gamma1_ * ctrl_dd_ * TotalTime_ * (alpha_ + ctrl_dd_ * beta_) +  gamma1_ * ctrl_dd_* (X_[1] - X_old[1]);
		alpha_ = alpha_new; 
	}
	if (beta_<1e-6f){
		beta_ = 1e-6f;
	}
	return;
}


/**
 * Compute control value
 */
void DDController::compute_ctrl() {
	float u_fb = 0;

	// Update the control gain
	Kdd_[0] = -P1_ * P2_;
	Kdd_[1] = P1_ + P2_;
	Kdd_[2] = 0.0f; 

	u_p_ = Kdd_[0] * (X_[0] - Tracking_[0]);
	u_d_ = Kdd_[1] * (X_[1] - Tracking_[1]);
	u_a_ = Kdd_[2] * (X_[2] - Tracking_[2]);

	u_fb = u_p_ + u_d_ + u_a_;
	
	u_ = (1.0f / beta_) * (-alpha_ + u_fb);

	if (u_ < 0.0f) {
		u_ = 0.0f;
	}
	if (u_ > 65535.0f) {
		u_ = 65535.0f;
	}

	estimatorDDSetControl(u_);
}

/** 
 * Insert the k-th measurement in the buffer
 */
void DDController::insert_newmeas_batch(float y, float stamp, int k) {
	if (k < 0) {
		// Error
	}
	int index = (BUFF_SIZE - 1) - (k % BUFF_SIZE);
	Ybuff_(index) = y;
	Tbuff_[index] = stamp;
}

void DDController::insert_newmeas_circ(float y, float stamp) {

	for (int index = BUFF_SIZE-1; index > 0; index--) { 
		Ybuff_(index) = Ybuff_(index-1);
		Tbuff_[index] = Tbuff_[index-1];       
	}
	Ybuff_(0) = y;
	Tbuff_[0] = stamp;
}

void DDController::update_O(
        Matrix<float, BUFF_SIZE, STATE_SIZE>& O,
        const float t[BUFF_SIZE]) {
    int i;

    for (i = 0; i < BUFF_SIZE; i++) {
        O(i, 0) = 1.0;
        O(i, 1) = -(i * t[i]);
        O(i, 2) = 0.5f * pow(i * t[i], 2);
    }
    return;
}


void DDController::finalize_data() {

	// Finalize the DT vector, computing the differences
	// [0, dt1, dt1 + dt2, ...]
	DTbuff_[0] = Tbuff_[0];
	for (int i = 0; i < BUFF_SIZE; i++) {
		DTbuff_[i] = Tbuff_[0] - Tbuff_[i];
	}

	//TotalTime_ = DTbuff_[BUFF_SIZE-1];
	TotalTime_ = DTbuff_[4];


	// Update the Observability matrix
	update_O(O_, DTbuff_);

	// Update the pseduoinverse matrix
	// TODO: Either make everything static with void calls,
	// 	either pass the values inside all the chain of calls
	eval_pseudoinv(O_, O_inv_);

}

/**
 * Estimator step function
 */
void DDController::DDEstimator_step_circ(float y, float stamp) {
	if (!isInit_) {
		estimatorDDInit();
	}

	// Update the buffer
	insert_newmeas_circ(y, stamp);

	if (Nmeas_ >= BUFF_SIZE) {
		finalize_data();

		estimate_state(O_inv_, Ybuff_, X_);
/*
		estimate_params();
		if (ctrl_dd_active_ && Step_ > 1) {	
			compute_ctrl();
		}
		set_estimator_ready();
*/
		geometry_msgs::Vector3Stamped X_msg;

		ros::Time stamp = ros::Time::now();
		X_msg.header.stamp = stamp;
		X_msg.vector.x = X_(0);
		X_msg.vector.y = X_(1);
		X_msg.vector.z = X_(2);
        
        dd_state_pub_.publish(X_msg);
	}
	else{
		Nmeas_++;
	}    
}

void DDController::DDEstimator_step_batch(float y, float stamp) {
	if (!isInit_) {
		estimatorDDInit();
	}

	// Update the buffer
	insert_newmeas_batch(y, stamp, Nmeas_);
	Nmeas_++;

	if (Nmeas_ == BUFF_SIZE) {
		Nmeas_ = 0;

		finalize_data();

		estimate_state(O_inv_, Ybuff_, X_);

		estimate_params();

		if (ctrl_dd_active_ && Step_ > 1) {	
			compute_ctrl();
		}
		//set_estimator_ready();
	}
}



void DDController::init_O() {
	int i;

	for (i = 0; i < BUFF_SIZE; i++) {
		O_(i, 0) = 1;
		O_(i, 1) = -(i * TS);
		O_(i, 2) = 0.5f * (i * i * TS2);
	}  
}

/**
 * Initialization function 
 */
void DDController::estimatorDDInit(void) {

	if (isInit_)  {
		return;
	}

    beta_ = 2.8577f*1e-4f;

    // Estimator Parametrs
    gamma1_ = 1.0f;

    // Control gain
    P1_ = 1.0f;
    P2_ = 1.0f;

    Tracking_[0] = 1.2f;
    Tracking_[1] = 0.0;
    Tracking_[2] = 0.0;

    u_ = 0.0;

    // Step Counter
    Step_ = 3;

    ctrl_dd_active_ = false;
    updated_ = false;

    // Filter Data
    Nmeas_ = 0;

    Kdd_[0] = -P1_ * P2_;
    Kdd_[1] = -P1_ + P2_;
	Kdd_[2] = 0.0f; 

    alpha_ = -11.900f;
    beta_ = 1.0;

	init_O();	

	eval_pseudoinv(O_, O_inv_);

	isInit_ = true;

    ROS_INFO("[%s] Initialized!", name_.c_str());
    std::cout << "Observability Matrix: (Nominal Ts = 0.004)" << std::endl << O_ << std::endl;
    std::cout << "Inverse Observability Matrix: " << std::endl << O_inv_ << std::endl; 
    std::cout << "Kdd = " << Kdd_ << std::endl;

	return;
}



// This function is triggered by the arrival of new measurements
void DDController::estimatorDDNewMeasurement(const boost::shared_ptr<geometry_msgs::PointStamped const>& pos) {

	msg_counter_ = msg_counter_ + 1;

	// Measure the timestamp
	float t_s = pos->header.stamp.sec + pos->header.stamp.nsec/1e9; // Time in second
	
	float meas = pos->point.z;

	// Do something with the new measurement 
	DDEstimator_step_circ(meas, t_s);

}

float DDController::estimatorDDGetEstimatedZ() {
	return X_(0);
}

/**
 * Check whether the estimator is ready. In that case
 * reset the flag and return 'true'. Otherwise, return 'false'.
 */
bool DDController::estimatorDDHasNewEstimate() {
	bool out;

	out = updated_;
	if (out)
		updated_ = false; // Reset the flag

	return out;
}

void DDController::estimatorDDSetControl(const float u) {
	ctrl_ddd_=ctrl_dd_;
	ctrl_dd_ = u;

    return;
}

float DDController::estimatorDDGetControl() {
	return ctrl_dd_;
}

void DDController::estimatorDDParamLeastSquares(void) {   
	ctrl_dd_active_ = true;
    
    return;
} 



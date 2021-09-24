#include <ros/ros.h>
#include "dd_controller/dd_controller_ros.hpp"


int main(int argc, char** argv) {

        ros::init(argc, argv, "DD_Controller");
        ros::NodeHandle nh;

        ROS_INFO("Starting Data Driven Control Node");

        DDControllerROS ddctrl_node;

        if(!ddctrl_node.Initialize(nh)) {
                ROS_ERROR("%s: Failed to initialize dd_controller.",
                                ros::this_node::getName().c_str());
                return EXIT_FAILURE;
        }

        ros::MultiThreadedSpinner spinner(3);
        spinner.spin();

        return EXIT_SUCCESS;
}

/*
#define CTRL_THRESHOLD (0.01f)



// ==================================================================
//			GLOBALS
// Global counter of received sensor messages 
unsigned long int msg_counter = 0;
DDParams pp;

static DDEstimator ddestimator_;
static DDParamEstimator ddparamestimator_;

static float state_vec[DDEST_FULLSTATESIZE];

// ==================================================================
// Helpers
bool control_valid(const float v[4]) {
	bool out = false;

	float norm = 0.0;
	float temp;
	for (int i = 0; i < 4; i++) {
		temp = powf(v[i], 2); 
		norm += temp;
	}

	if (norm > CTRL_THRESHOLD) {
		out = true;
	} else {
		out = false;
	}

	return out;
}


void estimatorDDSetparams(float ax, float bx, float ay, float by, 
		float a2d[DDESTPAR_ALPHA2DSIZE],
		float b2d[DDESTPAR_BETA2DSIZE]) {

	DDParams par; 
	par.valid = false; // Send the parameters with the false flag
	
	par.alpha_x = ax;
	par.alpha_y = ay;
	
	par.beta_x = bx;
	par.beta_y = by;

	for (int i = 0; i < DDESTPAR_ALPHA2DSIZE; i++) {
		 par.alpha2d[i] = a2d[i];
	}

	for (int i = 0; i < DDESTPAR_BETA2DSIZE; i++) {
		 par.beta2d[i] = b2d[i];
	}

	par.alpha2dsize = DDESTPAR_ALPHA2DSIZE;
	par.beta2dsize = DDESTPAR_BETA2DSIZE; 

	DDParamEstimator_SetParams(&ddparamestimator_, par);
}

// ===================================================================
// 				DDEstimator



// ==================================================================
// 				PRIVATE
void attitude_preprocessing(float rpy_dest[3], const quaternion_t q) {

	struct quat q_src = {q.x, q.y, q.z, q.w}; 
	struct vec rpy = quat2rpy(q_src);

	rpy_dest[0] = rpy.x;
	rpy_dest[1] = rpy.y;
	rpy_dest[2] = rpy.z;
}

// ==================================================================



void estimatorDDInit(void) {
	// Initialize the estimators
	DDEstimator_Init(&ddestimator_);
	DDParamEstimator_Init(&ddparamestimator_);
	
	// Initialize the parameters
	estimatorDDSetparams(
			alpha_xy_init,
			beta_x_init,
			alpha_xy_init,
			beta_y_init, 
			alpha2d_init,
			beta2d_init
			);

	// Initialize the bounds
	DDParamEstimator_SetBounds(&ddparamestimator_, 
		beta_x_bounds, beta_y_bounds,
		beta2d_lowerb, beta2d_upperb);

	// Initialize the gains
	DDParamEstimator_SetGains(&ddparamestimator_,
			gains_x, gains_y,
			gains_alpha2d,
			gains_beta2d);
}

bool estimatorDDTest(void) {
	return true;
}

bool estimatorDD_Step(state_t *state,
		const float controls[4],
		const uint32_t tick) {
	bool updated = false;

	// Update the gains
	DDParamEstimator_SetGains(&ddparamestimator_, gains_x,
			gains_y, gains_alpha2d, gains_beta2d);

	updated = DDEstimator_Step(&ddestimator_);

	if (updated) {
		// Export the estimated state into the system.
		DDEstimator_ExportState(&ddestimator_, state);

		float deltaT = 0;
		DDEstimator_GetMeasuresTimeInterval(
				&ddestimator_, &deltaT);

		// Check if the controller is issuing something
		if (control_valid(controls)) {
			// Run the parameter estimator
			DDParamEstimator_Step(&ddparamestimator_, state,
					controls, deltaT);
			pp = DDParamEstimator_GetParams(&ddparamestimator_);
		}
	}

	return updated;
}


 
DDParams estimatorDD_GetParam() {
	DDParams out = DDParamEstimator_GetParams(&ddparamestimator_);
	return out;
}



float estimatorDD_GetTMeasTimespan() {
	float out = 0.0;
	DDEstimator_GetMeasuresTimeInterval(&ddestimator_, &out);
	return out;
}
*/

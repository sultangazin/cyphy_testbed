#ifndef ESTIMATOR_DD_H 
#define ESTIMATOR_DD_H

#include <ros/ros.h>

#include <time.h>
#include "Eigen/Dense"

#include "geometry_msgs/PointStamped.h"

#define STATE_SIZE (3)
#define BUFF_SIZE (20)
#define TS (0.004f)
#define TS2 ((TS) * (TS))

#define  droneMass (0.032f);

// ===================================


using namespace Eigen;

// =================================================================
// CLASS
//
class DDController {

	public:
		DDController();
		~DDController();

		bool Initialize(const ros::NodeHandle& n);
        
        void estimatorDDInit(void);

        bool LoadParameters(const ros::NodeHandle& n);

        bool RegisterCallbacks(const ros::NodeHandle& n);

		// Callback on Measurement 
		void estimatorDDNewMeasurement(const boost::shared_ptr<geometry_msgs::PointStamped const>& pos); 

        float estimatorDDGetEstimatedZ();

        bool estimatorDDHasNewEstimate();

        void estimatorDDSetControl(const float u);

        float estimatorDDGetControl();

        void estimatorDDParamLeastSquares(void);

	private:

        // FUNCTIONS
        void DDEstimator_step_circ(float y, float stamp);
        void DDEstimator_step_batch(float y, float stamp);

        void estimate_params();

        void finalize_data();

        void update_O(Matrix<float, BUFF_SIZE, STATE_SIZE>& O, 
                const float t[BUFF_SIZE]);
    
        void insert_newmeas_circ(float y, float stamp);

        void insert_newmeas_batch(float y, float stamp, int k);

        void compute_ctrl();

        void eval_pseudoinv(
                const Matrix<float, BUFF_SIZE, STATE_SIZE>& O, 
                Matrix<float, STATE_SIZE, BUFF_SIZE>& Pseudo);

        void estimate_state(
                const Matrix<float, STATE_SIZE, BUFF_SIZE>& Oinv, 
                const Matrix<float, BUFF_SIZE, 1>& Y,
                Matrix<float, STATE_SIZE, 1>& X_est);

        void init_O();

        // MATRICES AND VECTORS 
		// On line i: [1, -sum(ts(k)), 1/2 * (sum(ts(k))^2]
		Matrix<float, BUFF_SIZE,  STATE_SIZE> O_;

		// Pseudo inverse
		Matrix<float, STATE_SIZE, BUFF_SIZE> O_inv_;

		// Vectors 
		Matrix<float, BUFF_SIZE, 1>  Ybuff_;

		float Tbuff_[BUFF_SIZE];
		float DTbuff_[BUFF_SIZE];

		Matrix<float, STATE_SIZE, 1> X_;
        Matrix<float, STATE_SIZE, 1> X_old;



		bool isInit_;

        std::string name_;

        ros::Publisher dd_state_pub_;
        ros::Subscriber  sens_channel_;

        int msg_counter_;

		// ====================================
        // Control Partial Signals
        float u_p_;
        float u_d_;
        float u_a_;

        float u_;

		// Estimator State 
		float alpha_;
		float alpha_new_;
		float beta_;
		// /droneMass;

		// Estimator Parametrs
		float gamma1_;

		// Control gain
		float P1_;
		float P2_;
		float Kdd_[3];
		float ctrl_dd_;
		float ctrl_ddd_;
		float Tracking_[3];

		
		// Step Counter
		int Step_;
		bool ctrl_dd_active_;
		bool updated_;

		// ====================================
		// Filter Data
		int Nmeas_;
	
		float TotalTime_;
};

#endif

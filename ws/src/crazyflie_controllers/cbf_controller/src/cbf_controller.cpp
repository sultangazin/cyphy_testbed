#include "cbf_controller/cbf_controller.hpp"

// XXX Function to be defined
double sigma(double);
double alpha(double);

CBFController::CBFController() {
}

CBFController::~CBFController() {
}

void CBFController::setPosState(const Vector3d& p, const Vector3d& v, const Vector3d& a) {
	position_ = p;
	velocity_ = v;
	acceleration_ = a;
}

void CBFController::setAttState(const Quaterniond& q, const Vector3d& w) {
	quaternion_ = q;
	ang_velocity_ = w;
}

void CBFController::setPosRef(const Vector3d& p, const Vector3d& v, const Vector3d& a) {
	ref_position_ = p;
	ref_velocity_ = v;
	ref_acceleration_ = a;
}

void CBFController::setAttRef(const Quaterniond& q) {
	ref_quaternion_ = q;
}

void CBFController::updateObstacles(int obst_id, const Vector3d& p, const Vector3d& v) {
	if (obst_map_.count(obst_id) > 0) {
		obst_map_[obst_id].position = p;
		obst_map_[obst_id].velocity = v;
	} else {
		obst_map_.insert(pair<int, ObstData>(obst_id, {p, v}));
	}
}

void CBFController::step() {
	mx_.lock();

	// Tracking Error (Position)
	computeTrackingError();
	solvePositionQP();

	computeAttError();
	generateAttitudeQP();
	solveAttitudeQP();

	mx_.unlock();
}

void CBFController::computeTrackingError() {
	position_err_ = position_ref_ - position_;	
	velocity_err_ = velocity_ref_ - velocity_;
	acceleration_err = acceleration_ref_ - acceleration_;
}

void CBFController::solvePositionQP() {
	// Compute Lyapunov function associated with the position error
	double Vx = computeLyapunovPos();

	Ax = -(M_ * velocity_err_.transpose() + epsx_ * position_err_.transpose()) / M_; 
	bx = Vx + Kx_ * position_err_.dot(velocity_err_) + epsx_ * velocity_err_.dot(velocity_err_);

	// XXX: 
	// minimize f(u)
	// subj. to Ax * u + b < 0
	//
	virtual_f_ = Eigen::Vector3d::Zeros();
}


void CBFController::generateSafetyConstraints() {
 	// For every obstacle:
	// Gamma1 * F + Gamma2 * M + Gamma3 + ... >= 0;
	Quaterniond b_q_w = quaternion.inverse();
	Vector3d b_d = b_q_w * r;
	double s = b_d(2);
}


void CBFController::solveAttitudeQP() {
	double f_norm = virtual_f_.norm(); 
	Eigen::Vector3d versor = virtual_f_.normalized();

	// Compute the Orientation error
	
	// Solve the QP
	double Vr = computeLyapunovAtt();

	Vector3d dist = position_ - obst_p;
	Vector3d dvel = velocity_ - obst_v; 
	Quaterniond b_q_w = quaternion.inverse();
	
	Vector3d b_d = b_q_w * dist;
	Vector3d b_dvel = b_q_w * dvel;

	Vector3d zb = b_q_w * Vector3d::UnitZ;
	//double s = r.dot(zb);
	double s = b_d(2);
	double s_dot = b_dvel(2) + dist.dot(omega_.cross(zb));

	// Compute g_hat
	double ghat = pow(dist.norm(), 2) - beta_ * b_safe - sigma(s);
	double ghat_dot = 2.0 * dvel.dot(dist) - beta_ * b_dot_ - sigma_dot(s) * s_dot;
	double hhat = gamma_ * alpha(ghat) + ghat_dot;

	Gamma1 = 1/M_ * (2 * s - sigma_dot(s));
	Gamma2 = (sigma_dot(s) * ((b_q_w * dist) * vee(Vecto3d::UnitZ) * J.inverse()));
	Gamma3 = gamma * alpha_dot(ghat) * ghat_dot +
		2.0 * pow(dvel.norm(), 2) - beta_ * b_safe_ddot - sigma_ddot(s) * pow(s_dot, 2) -
		sigma_dot(s) * (2.0 * ang_velocity_.cross(zb).dot(dvel) + dist.dot(ang_velocity_.cross(ang_velocity_.cros(zb)))) +
		sigma_dot(s) * obst_acc.dot(zb) - 2.0 * obst_acc.dot(dist);
	//Ar = ...;
	//br = ...;
	
	// XXX:
	// minimize f(M, F)
	// subj. to Ar 

	// Assign the actuation
	thrust_ = 0.0;
	torques_ = Eigen::Vector3d::Zeros();
}

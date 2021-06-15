#include <CGAL/MP_Float.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

#include "cbf_controller/cbf_controller.hpp"

using namespace Eigen;

typedef CGAL::MP_Float ET;
// program and solution types
typedef CGAL::Quadratic_program<ET> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;


// XXX Function to be defined
double sigma(double);
double alpha_fun(double x) {
	return x * x;
};

CBFController::CBFController() : 
	thrust_(0), rates_(Vector3d::Zero()),
	virtual_f_(Vector3d::Zero()),
	position_(Vector3d::Zero()),
	velocity_(Vector3d::Zero()),
	acceleration_(Vector3d::Zero()),
	quaternion_(Quaterniond::Identity()),
	ang_velocity_(Vector3d::Zero()),
	ref_position_(Vector3d::Zero()),
	ref_velocity_(Vector3d::Zero()),
	ref_acceleration_(Vector3d::Zero()) {
		// Instatiate the map of the control constants
		Kmap_.insert(std::pair<std::string, double>("Kpos", 0));
		Kmap_.insert(std::pair<std::string, double>("Kvel", 0));
		Kmap_.insert(std::pair<std::string, double>("Kx", 0));
		Kmap_.insert(std::pair<std::string, double>("Kr", 0));
		Kmap_.insert(std::pair<std::string, double>("eta", 0));
		Kmap_.insert(std::pair<std::string, double>("delta_x", 0));

		Kmap_.insert(std::pair<std::string, double>("gamma", 0));
		Kmap_.insert(std::pair<std::string, double>("beta", 0));
		Kmap_.insert(std::pair<std::string, double>("b_safe", 0));
		Kmap_.insert(std::pair<std::string, double>("b_safe_dot", 0));
		Kmap_.insert(std::pair<std::string, double>("b_safe_ddot", 0));
		Mass_ = 1.0;

		ObstData el;
		el.position = Vector3d(1,0,0.75);
		obst_map_.insert(std::pair<int, ObstData>(0, el));
		el.position = Vector3d(1,0,0.35);
		obst_map_.insert(std::pair<int, ObstData>(1, el));

	}

CBFController::~CBFController() {
}

void CBFController::setTranslationState(const Vector3d& p, const Vector3d& v, const Vector3d& a) {
	position_ = p;
	velocity_ = v;
	acceleration_ = a;
}

void CBFController::setAttitudeState(const Quaterniond& q, const Vector3d& w) {
	quaternion_ = q;
	ang_velocity_ = w;
}

void CBFController::setTranslationRef(const Vector3d& p, const Vector3d& v, const Vector3d& a) {
	ref_position_ = p;
	ref_velocity_ = v;
	ref_acceleration_ = a;
}

void CBFController::setAttitudeRef(const Quaterniond& q) {
}

void CBFController::updateObstacles(int obst_id, const Vector3d& p, const Vector3d& v) {
	if (obst_map_.count(obst_id) > 0) {
		obst_map_[obst_id].position = p;
		obst_map_[obst_id].velocity = v;
	} else {
		obst_map_.insert(std::pair<int, ObstData>(obst_id, {p, v}));
	}
}

void CBFController::step(double dt) {
	mx_.lock();

	// Tracking Error (Position)
	Vector3d f_des = solvePositionQP();

	Matrix3d w_R_b = quaternion_.normalized().toRotationMatrix();
	Vector3d w_zb = w_R_b.col(2);
	thrust_ = f_des.dot(w_zb);

	// Attitude Control
	Eigen::Vector3d z_des = f_des.normalized();
	Quaterniond qerr = computeAttError(z_des);
	solveAttitudeCtrl(qerr);

	mx_.unlock();
}


Vector3d CBFController::solvePositionQP() {
	Vector3d position_err_ = ref_position_ - position_;	
	Vector3d velocity_err_ = ref_velocity_ - velocity_;
	Vector3d acceleration_err = ref_acceleration_ - acceleration_;

	Vector3d Gg = -9.81 * Vector3d::UnitZ(); 

	// Compute Lyapunov function associated with the position error
	double ep_ep = position_err_.dot(position_err_);
	double ep_ev = position_err_.dot(velocity_err_);
	double ev_ev = velocity_err_.dot(velocity_err_);

	double V_p = 0.5 * (Kmap_["Kpos"] * ep_ep);
	double V_v = 0.5 * (Kmap_["Kvel"] * Mass_ * ev_ev);
	double V_pv = Kmap_["Kx"] * ep_ev;
	double Vx = V_p + V_v + V_pv;

	// Ax + bx < 0
	Vector3d Ax = -(Kmap_["Kvel"] * Mass_ * velocity_err_.transpose() + Kmap_["Kx"] * position_err_.transpose()) / Mass_; 
	double bx = Kmap_["Kpos"] * ep_ev + Kmap_["Kx"] * ev_ev - Mass_ * Ax.dot(ref_acceleration_ - Gg);
	bx = -bx - (Kmap_["eta"] * Vx) + Kmap_["delta_x"]; // Ax < -bx


	// Compute inequalities for obstacle avoidance 
	std::vector<Vector3d> Aobst;
	std::vector<double> bobst;
	for (std::unordered_map<int, ObstData>::iterator it = obst_map_.begin(); it != obst_map_.end(); it++) {
		ObstData el = it->second;

		Vector3d de = position_ - el.position;
		Vector3d dv = velocity_ - el.velocity;

		double ghat = pow(de.norm(), 2) - Kmap_["beta"] * Kmap_["b_safe"];
		double ghat_dot = 2 * de.dot(dv) - Kmap_["beta"] * Kmap_["b_safe_dot"];
		double alpha_ghat = alpha_fun(ghat);
		double alpha_prime = 2 * ghat;
		double hhat = Kmap_["gamma"] * alpha_ghat + ghat_dot;
		double square_1 = Kmap_["gamma"] * alpha_prime * ghat_dot;
		double square_2 = square_1 + 2.0 * pow(dv.norm(), 2) -
			Kmap_["beta"] * Kmap_["b_safe_ddot"] +
			2.0 * de.dot(Gg); //el.acceleration);
		int a = 1;
		double square_3 = square_2 + Kmap_["gamma"] * pow(hhat, 2 * a + 1);
		
		Aobst.push_back(-2.0 * de.transpose() / Mass_);
		bobst.push_back(square_3);

		std::cout << "Obst Distance: " << de.transpose() << std::endl;
	}

	Program qp (CGAL::SMALLER, true, 0, false, 0);
	const int X = 0; const int Y = 1; const int Z = 2;

	// Set the stabilization inequality (only one row) 
	qp.set_a(X, 0,  Ax(X)); qp.set_a(Y, 0, Ax(Y)); qp.set_a(Z, 0, Ax(Z));
	qp.set_b(0, bx);  //  Ax  <= bx

	for (int i = 0; i < obst_map_.size(); i++) {
		qp.set_a(X, i + 1,  Aobst[i](X)); qp.set_a(Y, i + 1, Aobst[i](Y)); qp.set_a(Z, i + 1, Aobst[i](Z));
		qp.set_b(i + 1, bobst[i]);  //  Ax  <= bx
	}

	// Direct constraints on the components...
	qp.set_u(X, true, 1.0); qp.set_u(Y, true, 1.0); qp.set_u(Z, true, 1.0);
	qp.set_l(X, true, -1.0); qp.set_l(Y, true, -1.0); qp.set_l(Z, true, -1.0);

	// Cost
	qp.set_d(X, X, 2.0); qp.set_d (Y, Y, 2.0); qp.set_d(Z, Z, 0.02); 

	// solve the program, using ET as the exact type
	Solution s = CGAL::solve_quadratic_program(qp, ET());
	assert(s.solves_quadratic_program(qp));

	for (auto it = s.variable_values_begin(); it != s.variable_values_end(); it++) {
		size_t index = it - s.variable_values_begin();
		virtual_f_(index) = CGAL::to_double(*it);
	}
	
	std::cout << "Err: " << position_err_.transpose() << std::endl;
	std::cout << "F: " << virtual_f_.transpose() << std::endl;
	std::cout << "Ax: " << Ax.transpose()  << std::endl;
	std::cout << "Ax * f: " << Ax.dot(virtual_f_) << std::endl;
	std::cout << "bx: " << bx << std::endl;
	//std::cout << "Aobst * f - bobst: " << Aobst[0].dot(virtual_f_) - bobst[0]<< std::endl;
	std::cout << std::endl;
	return virtual_f_;
}


void CBFController::generateSafetyConstraints() {
	// For every obstacle:
	// Gamma1 * F + Gamma2 * M + Gamma3 + ... >= 0;
	Quaterniond b_q_w = quaternion_.inverse();
	//Vector3d b_d = b_q_w * r;
	//double s = b_d(2);
}


void CBFController::solveAttitudeCtrl(Quaterniond& qerr) {
	/*
	   double f_norm = virtual_f_.norm(); 
	   Vector3d versor = virtual_f_.normalized();

	// Compute the Orientation error

	// Solve the QP
	double Vr = computeLyapunovAtt();

	Vector3d dist = position_ - obst_p;
	Vector3d dvel = velocity_ - obst_v; 
	Quaterniond b_q_w = quaternion.inverse();

	Vector3d b_d = b_q_w * dist;
	Vector3d b_dvel = b_q_w * dvel;

	Vector3d zb = b_q_w * Vector3d::UnitZ();
	//double s = r.dot(zb);
	double s = b_d(2);
	double s_dot = b_dvel(2) + dist.dot(omega_.cross(zb));

	// Compute g_hat
	double ghat = pow(dist.norm(), 2) - bKmap_["eta"] * b_safe - sigma(s);
	double ghat_dot = 2.0 * dvel.dot(dist) - bKmap_["eta"] * b_dot_ - sigma_dot(s) * s_dot;
	double hhat = gamma_ * alpha(ghat) + ghat_dot;

	Gamma1 = 1/Mass_ * (2 * s - sigma_dot(s));
	Gamma2 = (sigma_dot(s) * ((b_q_w * dist) * vee(Vecto3d::UnitZ()) * J.inverse()));
	Gamma3 = gamma * alpha_dot(ghat) * ghat_dot +
	2.0 * pow(dvel.norm(), 2) - bKmap_["eta"] * b_safe_ddot - sigma_ddot(s) * pow(s_dot, 2) -
	sigma_dot(s) * (2.0 * ang_velocity_.cross(zb).dot(dvel) + dist.dot(ang_velocity_.cross(ang_velocity_.cros(zb)))) +
	sigma_dot(s) * obst_acc.dot(zb) - 2.0 * obst_acc.dot(dist);
	//Ar = ...;
	//br = ...;

	// XXX:
	// minimize f(M, F)
	// subj. to Ar 

	// Assign the actuation
	thrust_ = 0.0;
	*/

	rates_ = Kmap_["Kr"] * qerr.vec(); 
}

Vector3d CBFController::getControlRates() {
	return rates_;
}


void CBFController::setK(std::string name, double val) {
	if (Kmap_.count(name) == 0) {
		std::cout << "CBFController::setK() Error!" << std::endl;
	} else {
		std::cout << "Setting " << name << ": " << val << std::endl;
		Kmap_[name] = val;
	}
}

void CBFController::setVehicleMass(double m) {
	Mass_ = m;
}

double CBFController::getControlThrust() {
	return thrust_;
}


Quaterniond CBFController::computeAttError(Vector3d& w_z_des) {
	Matrix3d w_R_b = quaternion_.normalized().toRotationMatrix();
	Vector3d w_zb = w_R_b.col(2);
	Vector3d n = w_zb.cross(w_z_des);
	double alpha = acos(w_zb.dot(w_z_des));

	Quaterniond q_err;
	q_err.w() = cos(alpha/2.0);
	q_err.vec() = sin(alpha/2.0) * n.normalized();

	q_err = quaternion_.inverse() * q_err;

	return q_err;
}

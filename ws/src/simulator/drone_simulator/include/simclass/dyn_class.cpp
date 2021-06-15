#include "dyn_class.hpp"
#include <iostream>

Eigen::Quaterniond rpy2quat(const Eigen::Vector3d& rpy) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy(0), Eigen::Vector3d::UnitX());

    return q;
}

using namespace Eigen;



IDynamics::IDynamics() {};
IDynamics::~IDynamics() {};




Dynamics_UAngles::Dynamics_UAngles(const SimParams& sp) {
	params_ = sp;
	StateDim_ = 10;
	InputDim_ = 4;

	pos_ = Eigen::Vector3d::Zero();
	vel_ = pos_;
	acc_ = pos_;
	quat_ = Eigen::Quaterniond::Identity();

	thrust_ = 0;
	rpy_ = Eigen::Vector3d::Zero();

}

Dynamics_UAngles::~Dynamics_UAngles() {}


void Dynamics_UAngles::set_state(const std::vector<double>& x0) {
	if (x0.size() != StateDim_) {
		std::cout << "Wrong state size!" << std::endl;
		return;
	}

	for (int i = 0; i < 3; i++) {
		pos_(i) = x0[i];
		vel_(i) = x0[i + 3];
		quat_.vec()(i) = x0[i + 7];
	}
	quat_.w() = x0[6];
}

void Dynamics_UAngles::step(double dt) {

	mx_.lock();
	// Extract Model Parameters
	double Mass = params_.Mass; 
	double ldrag = params_.c_drag;
	double adrag = params_.a_drag;  

	Vector3d i_TVect = quat_ * (Vector3d::UnitZ() * thrust_);

	std::cout << "Actuated F: " << i_TVect.transpose() << std::endl;

	acc_ = (i_TVect - vel_ * ldrag) / Mass - 9.81 * Vector3d::UnitZ();
	// Integrate the dynamics
	pos_ = pos_ + vel_ * dt + 0.5 * acc_ * dt*dt; 
	vel_ = vel_ + acc_ * dt;
	quat_ = rpy2quat(rpy_);

	// Consider the Ground
	if (pos_(2) <= 0.0) {
		pos_(2) = 0.0;
		vel_(2) = (vel_(2) < 0) ? 0.0 : vel_(2);
		acc_(2) = (acc_(2) < 0) ? 0.0 : acc_(2);
	}
	mx_.unlock();
}

std::vector<double> Dynamics_UAngles::get_state() {
	// Cast the Eigen Data into a std::vector
	std::vector<double> output(StateDim_, 0);

	mx_.lock();
	for (int i = 0; i < 3; i++) {
		output[i] = pos_(i);
		output[i + 3] = vel_(i);
		output[i + 7] = quat_.vec()(i);
	}
	output[6] = quat_.w();
	mx_.unlock();

	return output;
}

std::vector<double> Dynamics_UAngles::get_acceleration() {
	std::vector<double> output(3, 0);

	mx_.lock();
	for (int i = 0; i < 3; i++) {
		output[i] = acc_(i);
	}
	mx_.unlock();

	return output;
}

void Dynamics_UAngles::set_inputs(
		const std::vector<double>& u) {

	mx_.lock();
	// Extract the Control Vector
	thrust_ = (u[0] > 0.1) ? u[0] : 0.1;

	for (int i = 0; i < 3; i++) {
		rpy_(i) = u[i + 1]; 
	}
	// XXX The yaw is creating problems. :-P
	rpy_(2) = 0.0;
	mx_.unlock();
}



// DRONE CONTROLLED WITH RATES COMMANDS
Dynamics_URates::Dynamics_URates(const SimParams& sp) {
	params_ = sp;
	StateDim_ = 10;
	InputDim_ = 4;

	pos_ = Eigen::Vector3d::Zero();
	vel_ = pos_;
	acc_ = pos_;
	quat_ = Eigen::Quaterniond::Identity();

	thrust_ = 0;
	b_angvel_ = Eigen::Vector3d::Zero();

}

Dynamics_URates::~Dynamics_URates() {}


void Dynamics_URates::set_state(const std::vector<double>& x0) {
	if (x0.size() != StateDim_) {
		std::cout << "Wrong state size!" << std::endl;
		return;
	}

	mx_.lock();
	for (int i = 0; i < 3; i++) {
		pos_(i) = x0[i];
		vel_(i) = x0[i + 3];
		quat_.vec()(i) = x0[i + 7];
	}
	quat_.w() = x0[6];
	mx_.unlock();
}
void Dynamics_URates::step(double dt) {
	mx_.lock();
	// Extract Model Parameters
	double Mass = params_.Mass; 
	double ldrag = params_.c_drag;
	double adrag = params_.a_drag;  

	Vector3d i_TVect = quat_ * (Vector3d::UnitZ() * thrust_);

	/*
	std::cout << "Actuated F: " << i_TVect.transpose() << std::endl;
	std::cout << "Actuated T: " << thrust_ << std::endl;
	*/


	acc_ = (i_TVect - vel_ * ldrag) / Mass - 9.81 * Vector3d::UnitZ();

	// Compute the rotation axis
	Eigen::Vector3d n = b_angvel_.normalized();
	double dtheta = b_angvel_.norm() * dt;

	// Integrate the dynamics
	pos_ = pos_ + vel_ * dt + 0.5 * acc_ * dt * dt; 
	vel_ = vel_ + acc_ * dt;
	quat_ = quat_ * Eigen::AngleAxis<double>(dtheta, n);

	// Consider the Ground
	if (pos_(2) <= 0.0) {
		pos_(2) = 0.0;
		vel_(2) = (vel_(2) < 0) ? 0.0 : vel_(2);
		acc_(2) = (acc_(2) < 0) ? 0.0 : acc_(2);
	}
	mx_.unlock();
}

std::vector<double> Dynamics_URates::get_state() {
	// Cast the Eigen Data into a std::vector
	std::vector<double> output(StateDim_, 0);

	mx_.lock();
	for (int i = 0; i < 3; i++) {
		output[i] = pos_(i);
		output[i + 3] = vel_(i);
		output[i + 7] = quat_.vec()(i);
	}
	output[6] = quat_.w();
	mx_.unlock();

	return output;
}

std::vector<double> Dynamics_URates::get_acceleration() {
	std::vector<double> output(3, 0);

	mx_.lock();
	for (int i = 0; i < 3; i++) {
		output[i] = acc_(i);
	}
	mx_.unlock();

	return output;
}

void Dynamics_URates::set_inputs(
		const std::vector<double>& u) {

	mx_.unlock();
	// Extract the Control Vector
	thrust_ = (u[0] > 0.05) ? u[0] : 0.05;

	for (int i = 0; i < 3; i++) {
		b_angvel_(i) = u[i + 1]; 
	}
	b_angvel_(2) = 0.0;
	mx_.unlock();
}

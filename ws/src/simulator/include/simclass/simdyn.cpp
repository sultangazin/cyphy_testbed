#include <iostream>
#include "simdyn.hpp"


SimDyn::SimDyn(int Nx, int Nu, int Ny, int Nz, 
		fdyn_type f, fmeas_type g, fperf_type h) {
	StateDim_ = Nx;
	InputDim_ = Nu;
	MeasDim_ = Ny;
	PerfDim_ = Nz;

	dyn_fun = f;
	meas_fun = g;
	perf_fun = h;

	X_ = std::vector<double>(Nx, 0.0);
	U_ = std::vector<double>(Nu, 0.0);
	Y_ = std::vector<double>(Ny, 0.0);
	Z_ = std::vector<double>(Nz, 0.0);
}

SimDyn::~SimDyn() {};

void SimDyn::assing_f(fdyn_type f) {
	dyn_fun = f;
}

void SimDyn::assing_g(fmeas_type f) {
	meas_fun = f;
}

void SimDyn::assing_h(fperf_type f) {
	perf_fun = f;
}

bool SimDyn::set_U(const std::vector<double>& u) {
	data_mx_.lock();
	if (u.size() != InputDim_) {
		std::cerr << "Error in setting the inputs!" << std::endl;
		data_mx_.unlock();
		return false;
	}
	
	U_ = u;
	data_mx_.unlock();

	return true;
}

bool SimDyn::set_X(const std::vector<double>& x) {
	data_mx_.lock();
	if (x.size() != StateDim_) {
		std::cerr << "Error in setting the inputs!" << std::endl;
		data_mx_.unlock();
		return false;
	}
	
	X_ = x;
	data_mx_.unlock();

	return true;
}

bool SimDyn::get_Y(std::vector<double>& y) const {
	data_mx_.lock();
	y = Y_;
	data_mx_.unlock();

	return true;
}

bool SimDyn::get_X(std::vector<double>& x) const {
	data_mx_.lock();
	x = X_;
	data_mx_.unlock();

	return true;
}

bool SimDyn::get_Z(std::vector<double>& z) const {
	data_mx_.lock();
	z = Z_;
	data_mx_.unlock();

	return true;
}

/**
 * Simulation step function:
 * - Update the system state
 * - Update the measurement 
 *
 */
bool SimDyn::sim_step(double dt, void* pparam) {

	std::vector<double> x_new;

	data_mx_.lock();		
	if (dyn_fun) {
		dyn_fun(x_new, X_, U_, dt, pparam);
		X_ = x_new;
	}
	if (meas_fun) {
		meas_fun(Y_, X_, U_);
	}

	data_mx_.unlock();

	return true;
}

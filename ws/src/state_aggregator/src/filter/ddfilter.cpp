/**
 * @author Luigi Pannocchi
 * @file ddfilter.cpp
 *
 */

#include <Eigen/Eigenvalues>
#include <iostream>
#include <stdio.h>

#include "filter/ddfilter.hpp"

using namespace Eigen;

DDFilter::DDFilter():
	_x{DDXMat::Zero()}, _y{DDYMat::Zero()}, _u{DDUMat::Zero()} {

		setPos(Eigen::Vector3d::Zero());
		setSteps(5);
		computeC();
		updateSampleTime(0.03);
	}


DDFilter::DDFilter(const Vector3d& p0, int Nsteps, double dt) : 
	_x {DDXMat::Zero()}, _y{DDYMat::Zero()}, _u{DDUMat::Zero()} {

		setPos(p0);
		setSteps(Nsteps);

		computeC();

		updateSampleTime(dt);
	}

DDFilter::~DDFilter() {}

void DDFilter::resetPosition(const Vector3d& p0) {
	setPos(p0);
}

void DDFilter::setU(const DDUMat& u) {}

void DDFilter::setSteps(int n) {
	_Nsteps = n;
}


DDXMat DDFilter::getState() const {
	_mx.lock();
	DDXMat out {_x};
	_mx.unlock();
	return out;
}

const Vector3d DDFilter::getPos() const {
	_mx.lock();
	Vector3d out(_x.block<3,1>(0,0));
	_mx.unlock();
	return out;
}

const Vector3d DDFilter::getVel() const {
	_mx.lock();
	Vector3d out(_x.block<3,1>(3,0));
	_mx.unlock();
	return out;
}

const Vector3d DDFilter::getAcc() const {
	_mx.lock();
	Vector3d out(_x.block<3,1>(6,0));
	_mx.unlock();
	return out;
}


void DDFilter::setPos(const Vector3d& p){
	_mx.lock();
	_x.block<3,1>(0,0) = p;
	_mx.unlock();
}

void DDFilter::setVel(const Vector3d& v) {
	_mx.lock();
	_x.block<3,1>(3,0) = v;
	_mx.unlock();
}

void DDFilter::setAcc(const Vector3d& a) {
	_mx.lock();
	_x.block<3,1>(6,0) = a;
	_mx.unlock();
}


void DDFilter::prediction(double dt) {
	DDXMat s = getState();

	double dt2 = dt * dt;

	// Integrate position 
	Vector3d new_pos = ExtractPos(s) + ExtractVel(s) * dt +
		ExtractAcc(s) * (dt2 / 2.0);

	// Integrate the velocity
	Vector3d new_vel = ExtractVel(s) + ExtractAcc(s) * dt; 

	setPos(new_pos);
	setVel(new_vel);
}


void DDFilter::update(const DDYMat& y, double timestamp) {
	// Update Measurement queue
	_YQueue.push(y);
	_tQueue.push(timestamp);

	if (_YQueue.size() <= _Nsteps) {
		return;
	}

	_YQueue.pop();
	_tQueue.pop();

	// Create Measurement Vector
	Matrix<double, Dynamic, 1> Meas; 
	Matrix<double, Dynamic, 1> times;
	Meas.resize(DDFILTER_MEAS_DIM * _Nsteps, 1);
	times.resize(_Nsteps, 1);

	for (int i = 0; i < _Nsteps; i++) {
		int index = _Nsteps -1 - i;
		Meas.block<3,1>(3 * index, 0) = _YQueue.front();
		times(index, 0) = _tQueue.front();
		_YQueue.pop();
		_tQueue.pop();
	}

	for (int i = 0; i < _Nsteps; i++) {
		int index = _Nsteps -1 - i;
		_YQueue.push(Meas.block<3,1>(3 * index, 0));
		_tQueue.push(times((_Nsteps -1) - i, 0));
	}

	// Create ObsMatrix
	Matrix<double, Dynamic, DDFILTER_STATE_DIM> Theta;
	Theta.resize(DDFILTER_MEAS_DIM * _Nsteps, DDFILTER_STATE_DIM);
	for (int i = 0; i < _Nsteps; i++) {
		double dt = (times(i) - times(0));
		int index = i * DDFILTER_MEAS_DIM;
		Theta.block<DDFILTER_MEAS_DIM, DDFILTER_STATE_DIM>(index, 0) = _C;
		Theta.block<DDFILTER_MEAS_DIM, 3>(index, 3) = -Matrix<double, 3, 3>::Identity() * dt;
		Theta.block<DDFILTER_MEAS_DIM, 3>(index, 6) = Matrix<double, 3, 3>::Identity() * (dt * dt) / 2.0;
	}

	// Calculate PseudoInv
	//Eigen::MatrixXd pinv = Theta.completeOrthogonalDecomposition().pseudoInverse();
	//std::cout << "Size = " << pinv.rows() << " " << pinv.cols() << std::endl;
	// Compute State as Obs^-1 * Meas

	DDXMat dx = Theta.completeOrthogonalDecomposition().solve(Meas);
	// Update internal State
	setPos(dx.head<3>());
	setVel(dx.segment<3>(3));
	setAcc(dx.segment<3>(6));
}


/**
 * Generate the dynamics matrix of the system.
 */
DDDynMat DDFilter::computeA(double T) {
	DDDynMat out = DDDynMat::Identity();

	out.block<3,3>(0, 3) =
		Matrix<double, 3, 3>::Identity() * T;

	out.block<3,3>(0, 6) =
		Matrix<double, 3, 3>::Identity() * T * T / 2.0;

	out.block<3,3>(3, 6) =
		Matrix<double, 3, 3>::Identity() * T;
	return out;
}

/**
 * Compute C Matrix
 */
DDCMat DDFilter::computeC() {
	_C = DDCMat::Zero();
	_C.block<3,3>(0, 0) = Matrix<double, 3, 3>::Identity();

	return _C;
}


void DDFilter::updateSampleTime(double dt) {
	_T = dt;
	_A = computeA(dt); 
}


// ===========================================================
const Vector3d DDFilter::ExtractPos(const DDXMat& _x) const {
	Vector3d out(_x.block<3,1>(0,0));
	return out;
}

const Vector3d DDFilter::ExtractVel(const DDXMat& _x) const {
	Vector3d out(_x.block<3,1>(3,0));
	return out;
}

const Vector3d DDFilter::ExtractAcc(const DDXMat& _x) const {
	Vector3d out(_x.block<3,1>(6,0));
	return out;
}

/**
 * @author Luigi Pannocchi
 * @file polyfilter.cpp
 *
 */

#include <Eigen/Eigenvalues>
#include <iostream>
#include <stdio.h>

#include "filter/polyfilter.hpp"

using namespace Eigen;

PolyFilter::PolyFilter(const Vector3d& p0,
		const SigmaXMat& sigmastate_,
		const SigmaYMat& sigmameas_,
		double dt) : 
	state_ {XMat::Zero()}, meas_{YMat::Zero()},
	input_{UMat::Zero()}, _P{PMat::Zero()} {

		setPos(p0);

		updateSigmas(sigmastate_, sigmameas_);
		updateSampleTime(dt);

		_P = PMat::Identity();
	}

PolyFilter::~PolyFilter() {}

void PolyFilter::resetPosition(const Vector3d& p0) {
	setPos(p0);
}

void PolyFilter::reset(const XMat& x0) {

	mx_.lock();
	state_ = x0;
	mx_.unlock();

	_P = PMat::Identity();
}


void PolyFilter::setGain(const KMat& k) {
	FilterGain_ = k;
}

void PolyFilter::setU(const UMat& u) {
	mx_.lock();
	input_ = u;
	mx_.unlock();
}

const Vector3d PolyFilter::getPos() const {
	mx_.lock();
	Vector3d out(state_.block<3,1>(0,0));
	mx_.unlock();
	return out;
}

const Vector3d PolyFilter::getVel() const {
	mx_.lock();
	Vector3d out(state_.block<3,1>(3,0));
	mx_.unlock();
	return out;
}

const Vector3d PolyFilter::getAcc() const {
	mx_.lock();
	Vector3d out(state_.block<3,1>(6,0));
	mx_.unlock();
	return out;
}

const QMat PolyFilter::getQ() const {
	mx_.lock();
	QMat out(_params.Q);
	mx_.unlock();

	return out;	
}

const DynMat PolyFilter::getA() const {
	mx_.lock();
	QMat out(_params.A);
	mx_.unlock();

	return out;	
}

const PMat PolyFilter::getP() const {
	mx_.lock();
	QMat out(_P);
	mx_.unlock();

	return out;	
}

const HMat PolyFilter::getH() const {
	mx_.lock();
	HMat out(_params.H);
	mx_.unlock();

	return out;	
}

const RMat PolyFilter::getR() const {
	mx_.lock();
	RMat out(_params.R);
	mx_.unlock();

	return out;	
}

void PolyFilter::setPos(const Vector3d& p){
	mx_.lock();
	state_.block<3,1>(0,0) = p;
	mx_.unlock();
}

void PolyFilter::setPosNoBlk(const Vector3d& p){
	state_.block<3,1>(0,0) = p;
}

void PolyFilter::setVel(const Vector3d& v) {
	mx_.lock();
	state_.block<3,1>(3,0) = v;
	mx_.unlock();
}

void PolyFilter::setVelNoBlk(const Vector3d& v) {
	state_.block<3,1>(3,0) = v;
}

void PolyFilter::setAcc(const Vector3d& a) {
	mx_.lock();
	state_.block<3,1>(6,0) = a;
	mx_.unlock();
}

void PolyFilter::setAccNoBlk(const Vector3d& a) {
	state_.block<3,1>(6,0) = a;
}



/**
 * Predict the covariance given the linearized model of the system.
 */
PMat PolyFilter::predictXcovariance(
		PMat& Px,
		QMat& Q,
		DynMat& A) {
	PMat P_ = A * Px * A.transpose() + Q;

	return P_; 
}

PMat PolyFilter::updateXcovariance(
		PMat& Px,
		HMat& H,
		KMat& K,
		RMat& R) {
	RMat S = H * Px * H.transpose() + R;
	PMat P = Px - K * S * K.transpose();
	// Force symmetry
	P = 0.5 * (P + P.transpose());
	return P;
}

KMat PolyFilter::computeK(QMat& P, HMat& H, RMat& R) {
	RMat Temp = H * P * H.transpose() + R;
	KMat K = P * H.transpose() * Temp.inverse();

	return K;
}


void PolyFilter::prediction(double dt) {

	mx_.lock();
	UMat U = input_; 

	double dt2 = dt * dt;
	double dt3 = dt2 * dt;

	// Integrate position 
	Vector3d new_pos = Pos() + Vel() * dt +
		0.5 * Acc() * dt2 + U * dt3 / 6.0;

	// Integrate the velocity
	Vector3d new_vel = Vel() + Acc() * dt +
		0.5 * U * dt2;

	Vector3d new_acc = Acc() + U * dt;

	// =================
	if (_params.T != dt) {
		updateSampleTime(dt);
	}

	// =================
	// Covariance Prediction
	// P_  = A' * P * A + Q
	_P = predictXcovariance(
			_P,
			_params.Q,
			_params.A);	

	setPosNoBlk(new_pos);
	setVelNoBlk(new_vel);
	setAccNoBlk(new_acc);

	mx_.unlock();
}


void PolyFilter::update(const YMat& y) {


	HMat H_pos = HMat::Zero();
	H_pos.block<3,3>(0,0) = Matrix<double, 3, 3>::Identity();

	mx_.lock();
	// Compute the kalman gain
	KMat K = computeK(_P, H_pos, _params.R); 

	Vector3d innov = y - Pos();
	XMat dx = K * innov;
	//std::cout << dx.transpose() << std::endl;

	setPosNoBlk(Pos() + dx.head<3>());
	setVelNoBlk(Vel() + dx.segment<3>(3));
	setAccNoBlk(Acc() + dx.tail<3>());

	_P = updateXcovariance(_P, H_pos, K, _params.R);

	mx_.unlock();
}


/**
 * Generate the dynamics matrix of the system.
 */
DynMat PolyFilter::computeA(double T) {
	DynMat out = DynMat::Identity();

	out.block<3,3>(0, 3) =
		Matrix<double, 3, 3>::Identity() * T;

	out.block<3,3>(0, 6) =
		Matrix<double, 3, 3>::Identity() * T * T * 0.5;

	out.block<3,3>(3, 6) =
		Matrix<double, 3, 3>::Identity() * T;

	return out;
}


QMat PolyFilter::computeQ(double dt, SigmaXMat& sigmas) {
	QMat Q = QMat::Identity();

	double p = pow(dt, 2.0) * 0.5;
	double v = dt;
	double a = 1.0;

	double w[] {p, v, a};

	for (int i = 0; i < 3; i++) {
		Q.block<3, 3>(i * 3, 0) =
			Matrix3d::Identity() * w[i] * w[0] * sigmas(SIGMA_ACC_IND);

		Q.block<3, 3>(i * 3, 3) =
			Matrix3d::Identity() * w[i] * w[1] * sigmas(SIGMA_ACC_IND);

		Q.block<3, 3>(i * 3, 6) =
			Matrix3d::Identity() * w[i] * w[2] * sigmas(SIGMA_ACC_IND);
	}

	return Q;
}


void PolyFilter::updateSigmas(const SigmaXMat& x, 
		const SigmaYMat& y) {
	_params.sigmastate_ = x;
	_params.sigmameas_ = y;

	_params.R = RMat::Identity();
	_params.R(0,0) = y(0);
	_params.R(1,1) = y(1);
	_params.R(2,2) = y(2);

	std::cout << "UPDATED SIGMAS" << std::endl;
	std::cout << "X: " << _params.sigmastate_.transpose() << std::endl;
	std::cout << "Y: " << _params.sigmameas_.transpose() << std::endl;
}

void PolyFilter::updateSampleTime(double dt) {
	_params.T = dt;
	_params.A = computeA(dt); 
	_params.Q = computeQ(dt, _params.sigmastate_);
}


void PolyFilter::updateSigmaY(const Vector3d& s) {
	_params.sigmameas_ = s;

	_params.R = RMat::Identity();
	_params.R(0,0) = s(0);
	_params.R(1,1) = s(1);
	_params.R(2,2) = s(2);
}

// ===========================================================
const Vector3d PolyFilter::Pos() const {
	Vector3d out(state_.block<3,1>(0,0));
	return out;
}

const Vector3d PolyFilter::Vel() const {
	Vector3d out(state_.block<3,1>(3,0));
	return out;
}

const Vector3d PolyFilter::Acc() const {
	Vector3d out(state_.block<3,1>(6,0));
	return out;
}

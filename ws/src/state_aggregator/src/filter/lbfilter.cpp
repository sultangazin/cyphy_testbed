/**
 * @author Luigi Pannocchi
 * @file lbfilter.cpp
 *
 */

#include <Eigen/Eigenvalues>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#include <vector>
#include <ros/package.h>

#include "filter/lbfilter.hpp"

using namespace Eigen;
using namespace std;

template<typename M>
M load_csv (const string& path) {
	ifstream indata;
	indata.open(path);
	string line;
	vector<double> values;
	uint rows = 0;
	while (getline(indata, line)) {
		stringstream lineStream(line);
		string cell;
		while (getline(lineStream, cell, ',')) {
			values.push_back(stod(cell));
		}
		++rows;
	}
	return Map<const Matrix<typename M::Scalar,
	       M::RowsAtCompileTime,
	       M::ColsAtCompileTime,
	       RowMajor>>(values.data(), rows, values.size()/rows);
}


LBFilter::LBFilter(const Vector3d& p0, double dt) : 
	state_ {XMat::Zero()}, meas_{YMat::Zero()},
	input_{UMat::Zero()} {
		setPos(p0);
		string path = ros::package::getPath("state_aggregator");
		string filename = path + "/config/Kobs.csv";
		FilterGain_ = load_csv<KMat>(filename);
	}

LBFilter::~LBFilter() {}

void LBFilter::resetPosition(const Vector3d& p0) {
	setPos(p0);
}

void LBFilter::reset(const XMat& x0) {
	mx_.lock();
	state_ = x0;
	mx_.unlock();
}

void LBFilter::setU(const UMat& u) {
	input_ = u;
}

const Vector3d LBFilter::getPos() const {
	mx_.lock();
	Vector3d out(state_.block<3,1>(0,0));
	mx_.unlock();
	return out;
}

const Vector3d LBFilter::getVel() const {
	mx_.lock();
	Vector3d out(state_.block<3,1>(3,0));
	mx_.unlock();
	return out;
}

const Vector3d LBFilter::getAcc() const {
	mx_.lock();
	Vector3d out(state_.block<3,1>(6,0));
	mx_.unlock();
	return out;
}


void LBFilter::setPos(const Vector3d& p){
	mx_.lock();
	state_.block<3,1>(0,0) = p;
	mx_.unlock();
}

void LBFilter::setGain(const KMat& k) {
	FilterGain_ = k;
}

void LBFilter::setPosNoBlk(const Vector3d& p){
	state_.block<3,1>(0,0) = p;
}

void LBFilter::setVel(const Vector3d& v) {
	mx_.lock();
	state_.block<3,1>(3,0) = v;
	mx_.unlock();
}

void LBFilter::setVelNoBlk(const Vector3d& v) {
	state_.block<3,1>(3,0) = v;
}

void LBFilter::setAcc(const Vector3d& a) {
	mx_.lock();
	state_.block<3,1>(6,0) = a;
	mx_.unlock();
}

void LBFilter::setAccNoBlk(const Vector3d& a) {
	state_.block<3,1>(6,0) = a;
}



void LBFilter::prediction(double dt) {

	mx_.lock();

	UMat& U = input_;

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
	if (Ts_ != dt) {
		Ts_ = dt;
	}

	setPosNoBlk(new_pos);
	setVelNoBlk(new_vel);
	setAccNoBlk(new_acc);

	mx_.unlock();
}


void LBFilter::update(const YMat& y) {
	mx_.lock();
	// Compute the kalman gain
	Vector3d innov = y - Pos();
	XMat dx = FilterGain_ * innov;

	setPosNoBlk(Pos() + dx.head<3>());
	setVelNoBlk(Vel() + dx.segment<3>(3));
	setAccNoBlk(Acc() + dx.tail<3>());

	mx_.unlock();
}


// ===========================================================
const Vector3d LBFilter::Pos() const {
	Vector3d out(state_.block<3,1>(0,0));
	return out;
}

const Vector3d LBFilter::Vel() const {
	Vector3d out(state_.block<3,1>(3,0));
	return out;
}

const Vector3d LBFilter::Acc() const {
	Vector3d out(state_.block<3,1>(6,0));
	return out;
}

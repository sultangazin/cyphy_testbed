#include <iostream>
#include "dd_controller/dd_estimator.hpp"

// ===========================================================
//			DATA BUFFERS
// ===========================================================


// ===========================================================
// DDMEAS DATA STRUCTURE
//
DDMeas::DDMeas() : 
	meas(buffer_t {}),
	timestamps(buffer_t {}) {
		filled = false;
        empty = true;
		num_elements = 0;
        head = 0;
        tail = 0;
	}

DDMeas::~DDMeas() {
}

void DDMeas::Reset() {
	filled = false;
    empty = true;
	num_elements = 0;
    head = 0;
    tail = 0;
}


void DDMeas::AddMeas(double m, double stamp) {
    if (empty) {
        meas[head] = m;
        timestamps[head] = stamp;
        empty = false;
        num_elements++;
    } else {
        num_elements = 
            (num_elements >= DDEST_BUFFERSIZE) ? DDEST_BUFFERSIZE : (num_elements + 1);
        head = (head + 1) % DDEST_BUFFERSIZE;
        meas[head] = m;
        timestamps[head] = stamp;
        if (head == tail) {
            tail = (tail + 1) % DDEST_BUFFERSIZE;
        }
    }

    if (num_elements == DDEST_BUFFERSIZE) {
        filled = true;
    }
}

buffer_t DDMeas::get_meas() const {
    buffer_t out;

    unsigned int cind = head; 
    int counter = 0;
    while (1) { 
        double m = meas[cind];
        out[counter++] = m;
        cind = (cind - 1) % DDEST_BUFFERSIZE;
        if (cind == head) {
            break;
        }
    }

    return out;
}

buffer_t DDMeas::get_timestamps() const{
	return timestamps;
}

double DDMeas::get_last_timestamp() const {
	return timestamps[head];
}

double DDMeas::get_timeinterval() const {
    if (!empty)
        return (timestamps[head] - timestamps[tail]);
    else
        return 0;
}

buffer_t DDMeas::get_deltas() const {
    buffer_t out;

    unsigned int cind = head; 
    int counter = 0;
    while (1) { 
        double dT = timestamps[head] - timestamps[cind];
        out[counter++] = dT;
        cind = (cind - 1) % DDEST_BUFFERSIZE;
        if (cind == head) {
            break;
        }
    }

    return out;
}

bool DDMeas::is_filled() const {
	return filled;
}


// 
// DDESTIMATOR1D DATA STRUCTURE
//
DDEstimator1D::DDEstimator1D() {
	ready = false;
}

DDEstimator1D::~DDEstimator1D() {
}

void DDEstimator1D::Reset() {
	meas_data.Reset();
	Obs = Eigen::Matrix<double, DDEST_BUFFERSIZE, DDEST_STATESIZE1D>::Zero();
	state_est = Eigen::Matrix<double, DDEST_STATESIZE1D, 1>::Zero();
	ready = false;
}

void DDEstimator1D::AddMeas(double m, double t) {
	meas_data.AddMeas(m, t);

	ready = meas_data.is_filled();
}


bool DDEstimator1D::Step() {

	if (!ready) {
		return false;
	}

	buffer_t deltaT = meas_data.get_deltas();

	// Fill the Obs Matrix
	for (int i = 0; i < DDEST_BUFFERSIZE; i++) {
		double T = deltaT[i];
		Obs(i, 0) = 1;
		Obs(i, 1) = -T;
		Obs(i, 2) = 0.5f * (T * T);
	}

	// Map the measurements array into an Eigen structure
	Eigen::Matrix<double, DDEST_BUFFERSIZE, 1> b(meas_data.get_meas().data());

	state_est = Obs.colPivHouseholderQr().solve(b);

	return true;
}

void DDEstimator1D::get_state(std::array<double, DDEST_STATESIZE1D> s) {
	for (int i = 0; i < DDEST_STATESIZE1D; i++) {
		s[i] = state_est(i);
	}
}

std::array<double, DDEST_STATESIZE1D> DDEstimator1D::get_state() {
	std::array<double, DDEST_STATESIZE1D> out;
	for (int i = 0; i < DDEST_STATESIZE1D; i++) {
		out[i] = state_est(i);
	}

	return out;
}


buffer_t DDEstimator1D::get_timestamps() {
	return meas_data.get_timestamps();
}


double DDEstimator1D::get_last_timestamp() {
	double out = meas_data.get_last_timestamp();
	return out;
}

double DDEstimator1D::get_timeinterval() {
	double out = meas_data.get_timeinterval();
	return out;
}

bool DDEstimator1D::is_ready() {
	return ready;
}

// 
// DDESTIMATOR DATA STRUCTURE
//
DDEstimator::DDEstimator() {
	sensors_mrt = 0.0;
	ready = false;
}

DDEstimator::~DDEstimator() {
}

void DDEstimator::Reset() {
	for (auto& el : estimators) {
		el.Reset();
	}
	sensors_mrt = 0.0;
	ready = false;
}

void DDEstimator::AddMeas(
		const std::array<double, DDEST_NUMOFCHANNELS> m,
                double tstamp) {

	for (int i = 0; i < DDEST_NUMOFCHANNELS; i++) {
		estimators[i].AddMeas(m[i], tstamp);
    }

	if (estimators[0].is_ready())
		ready = true;
}


bool DDEstimator::Step() {
	if (ready) {
		// Check whether there are new measurements
		double mrt = estimators[0].get_last_timestamp();
		if (mrt <= sensors_mrt) {
			return false;
		}
		sensors_mrt = mrt;

		// Run the estimation on each channel
		for (int i = 0; i < DDEST_NUMOFCHANNELS; i++) {
			estimators[i].Step();
		}
	} else {
		return false;
	}

	return true;
}

double DDEstimator::GetMeasuresTimeInterval() {
	double out = 0.0;
    out = estimators[0].get_timeinterval();
	return out;
}


void DDEstimator::GetState(state_t* ps) {
	for (int i = 0; i < DDEST_NUMOFCHANNELS; i++) {
		state1d_t state1d = estimators[i].get_state();

		double rpy[3] = {0,0,0};
		uint32_t osTick = 0;
		switch (i) {
			case DDEST_XCHANNEL:
			case DDEST_YCHANNEL:
			case DDEST_ZCHANNEL:
				ps->position(i - DDEST_XCHANNEL) = state1d[0];
				ps->velocity(i - DDEST_XCHANNEL) = state1d[1];
				ps->acceleration(i - DDEST_XCHANNEL) = state1d[2];
				break;
			case DDEST_ROLLCHANNEL:
			case DDEST_PITCHCHANNEL:
			case DDEST_YAWCHANNEL:
                		rpy[i - DDEST_ROLLCHANNEL] = state1d[0]; 
				ps->attitude(i - DDEST_ROLLCHANNEL) = state1d[0];
				ps->attitude_d(i - DDEST_ROLLCHANNEL) = state1d[1];
				ps->attitude_dd(i - DDEST_ROLLCHANNEL) = state1d[2];
				break;
			default:
				puts("Something queer is going on here\n");
				break;
		}

		// Update the quaternion
		ps->attitudeQuaternion =
			Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) * 
			Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());

		// Update the timestamps
		ps->timestamp.tv_sec = 0;
		ps->timestamp.tv_nsec = 0;
	}
}

/*
 *
 * Copyright (c) 2020 Luigi Pannocchi 
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * estimatorDD_data.h - DataDriven Estimator Data Structures 
 *
 */
#ifndef __ESTIMATOR_DD_OBJECTS_H__
#define __ESTIMATOR_DD_OBJECTS_H__

#include <Eigen/Dense>
#include <time.h>

#define DDEST_BUFFERSIZE (4)
#define DDEST_NUMOFCHANNELS (6)
#define DDEST_NUMOFINPUTS (6)
#define DDEST_STATESIZE1D (3)
#define DDEST_FULLSTATESIZE ((DDEST_NUMOFCHANNELS)*(DDEST_STATESIZE1D))

#define DDEST_XCHANNEL (0)
#define DDEST_YCHANNEL (1)
#define DDEST_ZCHANNEL (2)
#define DDEST_ROLLCHANNEL (3)
#define DDEST_PITCHCHANNEL (4)
#define DDEST_YAWCHANNEL (5)

typedef struct state_s {
    // Position 
    Eigen::Vector3d position;

    // Velocity 
    Eigen::Vector3d velocity;

    // Acceleration
    Eigen::Vector3d acceleration;

    // Attitude Quaternion
    Eigen::Quaterniond attitudeQuaternion;

    // Attitude 
    Eigen::Vector3d attitude;

    // Attitude First Derivative
    Eigen::Vector3d attitude_d;

    // Attitude Second Derivative 
    Eigen::Vector3d attitude_dd;

    timespec timestamp;
} state_t;


typedef std::array<double, DDEST_BUFFERSIZE> buffer_t;
typedef std::array<double, DDEST_STATESIZE1D> state1d_t;

/**
 * Data structure for the measurements
 */
class DDMeas {
    public:
        DDMeas();
        ~DDMeas();

        void Reset();

        // Add measurement to the buffer
        void AddMeas(double measurement, double tstamp);

        // Get the measurement data
        buffer_t get_meas() const;
        buffer_t get_timestamps() const;
        double get_last_timestamp() const;

        // Check if the buffer is filled
        bool is_filled() const;

    private:
        // Array of measurements
        buffer_t meas;
        // Vector of timestamps (of the measurements)
        buffer_t timestamps;

        // Number of measurement effectively in the buffer
        int num_elements;

        bool filled;
};



/**
 * Estimator 1D data structure
 */
class DDEstimator1D {
    public:
        // Public Methods
        DDEstimator1D();
        ~DDEstimator1D();
        void Reset();

        // Push a new measurement into the estimator
        void AddMeas(double m, double t);

        // Step function
        bool Step();

        // Getters
        void get_state(std::array<double, DDEST_STATESIZE1D> s);
        std::array<double, DDEST_STATESIZE1D> get_state();

        // Return the timevectot of the measurements
        buffer_t get_timestamps();
        double get_last_timestamp();

        bool is_ready();

    private:
        // Measurement Data
        DDMeas meas_data;

        // Observation Data
        Eigen::Matrix<double, DDEST_BUFFERSIZE, DDEST_STATESIZE1D> Obs;

        // Data buffer for the status
        Eigen::Matrix<double, DDEST_STATESIZE1D, 1> state_est;

        // Status of the system: when it is possible to get an estimate.
        bool ready;
};



/**
 * Estimator Multi-D data structure
 */
class DDEstimator {
    public:
        // Public Methods
        DDEstimator();
        ~DDEstimator();

        void Reset();

        /**
         * Push a new measurement in the estimator
         */
        void AddMeas(const std::array<double, DDEST_NUMOFCHANNELS> m,
                double tstamp);

        /**
         * Update the control input 
         */
        void UpdateCtrl( const std::array<double, DDEST_NUMOFINPUTS> u);

        bool Step();

        double GetMeasuresTimeInterval();
        void GetState(state_t* state);
    private:
        // Measurement Data
        DDEstimator1D estimators[DDEST_NUMOFCHANNELS];

        state_t state;

        bool ready;

        double sensors_mrt;
};


#endif //__ESTIMATOR_DD_DATA_H__

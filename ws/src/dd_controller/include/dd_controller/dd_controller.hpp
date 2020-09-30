#ifndef __CONTROLLER_DD_HPP__
#define __CONTROLLER_DD_HPP__

#include "dd_controller/dd_estimator_param.hpp"

#define DDCTRL_OUTPUTSIZE (4)

typedef struct setpoint_s {
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
} setpoint_t;



class DDController {
	public:
		DDController();
		~DDController(); 

		void SetSetpoint(setpoint_t* ps);

		void SetKxy(const std::array<double, 2>  k);
		void SetKz(const std::array<double, 2> k);
		void SetKatt(const std::array<double, 2>  k);
		void SetKyaw(const std::array<double, 2>  k);

		void Step(const state_t* ps, DDParams* pp, double T);

		void getControls(double m_ctrls[DDCTRL_OUTPUTSIZE]);
	private:
		setpoint_t ctrl_setpoint;

		std::array<double, 2> Kxy_;
		std::array<double, 2> Kz_;
		std::array<double, 2> Katt_;
		std::array<double, 2> Kyaw_;

		Eigen::Matrix<double, DDCTRL_OUTPUTSIZE, 1> inputs;

		double lin2angle(const double setpoint[2],
						const double state_lin[2], const double state_ang[2],
						double alpha, double beta, double deltaT);

};


#endif //__CONTROLLER_DD_DATA_H__

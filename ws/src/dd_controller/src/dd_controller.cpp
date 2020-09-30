/**
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
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * controller_dd_objects.h - DataDriven Controller Interface
 */

#include <math.h>
#include "dd_controller/dd_controller.hpp"

#define MAXTILT (3.0f * M_PI / 8.0f)
#define MAXANGULARSPEED (10000.0f)


// Limit values
double boundval(double input, double llim, double ulim) {
		return fmaxf(fminf(input, ulim), llim);
}

// Split the setpoint in different components
void setpoint2arrays(const setpoint_t* sp,
				double xsp[2],
				double ysp[2],
				double zsp[2],
				double yawsp[2]) {

		xsp[0] = sp->position(0);
		xsp[1] = sp->velocity(0);

		ysp[0] = sp->position(1);
		ysp[1] = sp->velocity(1);

		zsp[0] = sp->position(2);
		zsp[1] = sp->velocity(2);

		yawsp[0] = sp->attitude(2);
		yawsp[1] = sp->attitude_d(2);
}

// Split the state estimate structure in different components
void state2arrays(const state_t* sp,
				double xest[2],
				double yest[2],
				double zest[2],
				double rollest[2],
				double pitchest[2],
				double yawest[2]) {

		xest[0] = sp->position(0);
		xest[1] = sp->velocity(0);

		yest[0] = sp->position(1);
		yest[1] = sp->velocity(1);

		zest[0] = sp->position(2);
		zest[1] = sp->velocity(2);

		rollest[0] = sp->attitude(0);
		rollest[1] = boundval(sp->attitude_d(0),
						-MAXANGULARSPEED,
						MAXANGULARSPEED);

		pitchest[0] = sp->attitude(1);
		pitchest[1] = boundval(sp->attitude_d(1),
						-MAXANGULARSPEED,
						MAXANGULARSPEED);

		yawest[0] = sp->attitude(2);
		yawest[1] = boundval(sp->attitude_d(2),
						-MAXANGULARSPEED,
						MAXANGULARSPEED);
}




// PUBLIC
DDController::DDController() {
	SetKxy({-0.25, -1.0});
	SetKz({-100.0, -20.0});
	SetKatt({-80.0, -20.0});
	SetKyaw({-100, -20});
}

DDController::~DDController() {
}

void DDController::SetSetpoint(setpoint_t* ps) {
		memcpy(&ctrl_setpoint, ps, sizeof(setpoint_t));
}

void DDController::SetKxy(const std::array<double, 2>  k) {
		for (int i = 0; i < 2; i++) {
				Kxy_[i] = k[i];
		}
}

void DDController::SetKz(const std::array<double, 2>  k) {
		for (int i = 0; i < 2; i++) {
				Kz_[i] = k[i];
		}
}

void DDController::SetKatt(const std::array<double, 2>  k) {
		for (int i = 0; i < 2; i++) {
				Katt_[i] = k[i];
		}
}

void DDController::SetKyaw(const std::array<double, 2>  k) {
		for (int i = 0; i < 2; i++) {
				Kyaw_[i] = k[i];
		}
}


void DDController::Step(const state_t *state, DDParams* par,
				double deltaT) {

		// Get the current state estimates
		double x_est[2];
		double y_est[2];
		double z_est[2];
		double roll_est[2];
		double pitch_est[2];
		double yaw_est[2];
		state2arrays(state, x_est, y_est, z_est,
						roll_est, pitch_est, yaw_est);

		// Get the current setpoint
		double x_setpoint[2];
		double y_setpoint[2];
		double z_setpoint[2];
		double yaw_setpoint[2];
		setpoint2arrays(&ctrl_setpoint,
						x_setpoint, y_setpoint, z_setpoint, yaw_setpoint);

		// Compute along Z
		double phatz = Kz_.at(0) * (z_est[0] - z_setpoint[0]) +
				Kz_.at(1) * (z_est[1] - z_setpoint[1]);

		// Compute the Yaw part 
		double phatyaw = Kyaw_.at(0)* (yaw_est[0] - yaw_setpoint[0]) +
				Kyaw_.at(1) * (yaw_est[1] - yaw_setpoint[1]);

		// Compute the mapping position error --> demanded_correction --> demanded_angle
		double phatpitch = lin2angle(x_setpoint, x_est, pitch_est,
						par->alpha_x, par->beta_x, deltaT);

		double phatroll = lin2angle(y_setpoint, y_est, roll_est,
						par->alpha_y, par->beta_y, deltaT);


		Eigen::Matrix<double, 4, 1> phat(phatz, phatroll, phatpitch, phatyaw);

		// Inputs = Beta^-1 * (Phat  - Alpha) 
		inputs = par->beta2d.inverse() * (par->alpha2d - phat);

		for (int i = 0; i < DDCTRL_OUTPUTSIZE; i++) {
				if (inputs(i) > 1.0) {
						inputs(i) = 1.0;
				} else if (inputs(i) < 0.0) {
						inputs(i) = 0.0;
				}
		}
}

/**
 * Control Action along a single axis
 */
double DDController::lin2angle(
				const double setpoint[2],
				const double state_posvel[2],
				const double state_att[2],
				double alpha, double beta,
				double deltaT) {

		double angle_output = 0;

		double pos = state_posvel[0];
		double vel = state_posvel[1];

		double angle = state_att[0];
		double angle_vel = state_att[1];

		// Compute the acc = Kp * (ep) + Kd * (ev)
		double x[2] = {
				pos - setpoint[0],
				vel - setpoint[1]
		};
		double acc_dem = Kxy_.at(0) * x[0] + Kxy_.at(1) * x[1];

		// Compute the u to get the acceleration along this axis
		double phix = (-alpha + acc_dem) / beta;
		phix = boundval(phix, -MAXTILT, MAXTILT);

		// That is related to the angle
		double ex[2] = {
				angle - phix,
				angle_vel - 
						(Kxy_.at(0) * vel + Kxy_.at(1) * (alpha + beta * angle)) / beta
		};

		angle_output = Katt_.at(0) * ex[0] +
				Katt_.at(1) * ex[1] +
				deltaT * (Kxy_.at(0) * (alpha/beta + angle) + Kxy_.at(1)* angle_vel);

		return angle_output;
}

void DDController::getControls(double m_ctrls[DDCTRL_OUTPUTSIZE]) {
		for (int i = 0; i < DDCTRL_OUTPUTSIZE; i++) {
				m_ctrls[i] = inputs[i];
		}
}

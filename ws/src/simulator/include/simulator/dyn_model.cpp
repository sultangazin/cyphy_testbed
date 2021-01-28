#include "simulator/simulator.hpp"

using namespace std;

void Xstd2Eigen(const vector<double>& x, 
        Eigen::Vector3d& p, Eigen::Vector3d& v, Eigen::Quaterniond& q) {

    // Extract the State Vector
    // The x vector is a 3 + 3 + 4 = 10
    // Split the overal state vector into the components
    vector<double> p_(x.begin(), x.begin() + 3);
    vector<double> v_(x.begin() + 3, x.begin() + 6);
    vector<double> q_(x.begin() + 6, x.end());

    // Convert std::vector into eigen vectors
    p = Eigen::Vector3d(p_.data());
    v = Eigen::Vector3d(v_.data());
    q = Eigen::Quaterniond(q_[0], q_[1], q_[2], q_[3]);
}

void XEigen2std(vector<double>& x,
        const Eigen::Vector3d& p, const Eigen::Vector3d& v,
        const Eigen::Quaterniond& q) {

    vector<double> x_new(10, 0.0);

    for (int i = 0; i < 3; i++) {
        x_new[i] = p(i);
        x_new[i + 3] = v(i);
    }

    x_new[6] = q.w();
    for (int i = 0; i < 3; i++) {
        x_new[i + 7] = q.vec()(i);
    }
    x = x_new;
}

Eigen::Quaterniond rpy2quat(const vector<double>& rpy) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());

    return q;
}



// ========================================================================
//                      DYNAMICS
// ========================================================================

/**
 * Simple Dynamic Model of the vehicle 
 * The attitude kinematics is neglected, assuming that the internal controller
 * of the drone is able to achieve the requested attitude.
 * This describe the interface of the flat controller, which is sending
 * Thrust, RPY angles as a reference points to the internal controller.
 *
 *  State: Position(3) | Velocity(3) | Quaternion(4)
 *  Input: Thrust(1) | Orientation(3)
 */
bool f1(vector<double>& x_,
	const vector<double>& x,
	const vector<double>& u, 
	double dt, void* param) {
	
    SimParam* parg = (SimParam*) param;

    // Extract Model Parameters
    double Mass = parg->Mass; 
    double ldrag = parg->c_drag;
    double adrag = parg->a_drag;  

    Eigen::Vector3d p, v;
    Eigen::Quaterniond q;

    // Convert the std vectors in Eigen
    Xstd2Eigen(x, p, v, q);

    // Extract the Control Vector
    double T = (u[0] > 0.1) ? u[0] : 0.1;
    vector<double> rpy(u.begin() + 1, u.end());
    // XXX The yaw is creating problems.
    rpy[2] = 0.0;
     
    Eigen::Vector3d ThrustVect = q * (Eigen::Vector3d::UnitZ() * T);
    Eigen::Vector3d nv = v.normalized();
    Eigen::Vector3d acc = (ThrustVect - nv * ldrag) / Mass - 9.81 * Eigen::Vector3d::UnitZ();

    // Integrate the dynamics
    p = p + v * dt + 0.5 * acc * dt*dt; 
    v = v + acc * dt;
    q = rpy2quat(rpy);

    // Consider the Ground
    if (p(2) < 0.0) {
        p(2) = 0.0;
        v(2) = 0.0;
    }

    // Put the Eigen vector back into the standard vector
    // Return the updated vector
    XEigen2std(x_, p, v, q);

	return true;
}



/**
 * Dynamic Model of the vehicle 
 *
 *  State: Position(3) | Velocity(3) | Quaternion(4)
 *  Input: Thrust(1) | AngularVel(3)
 */
bool f2(vector<double>& x_,
	const vector<double>& x,
	const vector<double>& u, 
	double dt, void* param) {
	
    SimParam* parg = (SimParam*) param;

    // Extract Model Parameters
    double Mass = parg->Mass; 
    double ldrag = parg->c_drag;
    double adrag = parg->a_drag; 

    Eigen::Vector3d p, v;
    Eigen::Quaterniond q;

    // Convert the std vectors in Eigen
    Xstd2Eigen(x, p, v, q);

    // Extract the Control Vector
    double T = (u[0] > 0.05) ? u[0] : 0.05;
    vector<double> omega_(u.begin() + 1, u.end());
    Eigen::Vector3d omega_b(omega_.data());
 
    Eigen::Vector3d ThrustVect = q * (Eigen::Vector3d::UnitZ() * T);
    Eigen::Vector3d nv = v.normalized();
    Eigen::Vector3d acc = (ThrustVect - nv * ldrag) / Mass - 9.81 * Eigen::Vector3d::UnitZ();

    // Compute the rotation axis
    Eigen::Vector3d n = omega_b.normalized();
    double dtheta = omega_b.norm() * dt;

    // Integrate the dynamics
    p = p + v * dt + 0.5 * acc * dt*dt; 
    v = v + acc * dt;
    q = q * Eigen::AngleAxis<double>(dtheta, n);

    // Consider the Ground
    if (p(2) < -0.01) {
        p(2) = 0.0;
        v(2) = 0.0;
    }

    // Put the Eigen vector back into the standard vector
    // Return the updated vector
    XEigen2std(x_, p, v, q);

	return true;
};




// ========================================================================
//                              SENSORS 
// ========================================================================

/**
 * Function to simulate the sensor data
 */
bool g1 (std::vector<double>& y,
		const std::vector<double>& x,
		const std::vector<double>& u) {
	y[0] = x[0];
	return true;
}

// ========================================================================
//                              COST 
// ========================================================================

/**
 * Function to compute the performance
 */
bool h1 (std::vector<double>& z,
		const std::vector<double>& x,
		const std::vector<double>& u) {
	return true;
}

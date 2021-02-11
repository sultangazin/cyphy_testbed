#ifndef CUSTOM_CONVERSION_HPP
#define CUSTOM_CONVERSION_HPP

#include "Eigen/Dense"

bool healthy_vector(Eigen::Vector3d& v); 
Eigen::Quaterniond qsum(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
Eigen::Quaterniond qsm(Eigen::Quaterniond q1, double a); 
Eigen::Vector3d qd2w(Eigen::Quaterniond q, Eigen::Quaterniond qd);
Eigen::Vector3d q2eul(Eigen::Quaterniond q);

#endif

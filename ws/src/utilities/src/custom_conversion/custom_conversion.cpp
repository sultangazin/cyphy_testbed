#include "utilities/custom_conversion/custom_conversion.hpp"

using namespace Eigen;

bool healthy_vector(Eigen::Vector3d& v) {
    bool ret = true;
    for (int i = 0; i < 3; i++) {
        if (std::isnan(v(i))) {
            v = Eigen::Vector3d::Zero();
            ret = false;
            break;
        }
    }
    return ret;
}

Quaterniond qsum(Quaterniond q1, Quaterniond q2) {
    Quaterniond res(Quaterniond::Identity());

    res.x() = q1.x() + q2.x();
    res.y() = q1.y() + q2.y();
    res.z() = q1.z() + q2.z();
    res.w() = q1.w() + q2.w();

    return res;
}

Quaterniond qsm(Quaterniond q1, double a) {
    Quaterniond res(Quaterniond::Identity());

    res.x() = q1.x() * a;
    res.y() = q1.y() * a;
    res.z() = q1.z() * a; 
    res.w() = q1.w() * a;

    return res;
}

Vector3d qd2w(Quaterniond q, Quaterniond qd) {
    Vector3d out;
    Matrix<double, 3, 4> M; 
    Vector4d v;
    v.block<3,1>(0,0) = qd.vec();
    v(3) = qd.w();
    M << -q.x(), q.w(), q.z(), -q.y(),
      -q.y(), -q.z(), q.w(), q.x(),
      -q.z(), q.y(), -q.x(), q.w();

    out = M * v;
    return out;
}

// math_utils.hpp
#pragma once

#include <Eigen/Dense>

inline Eigen::Matrix3d skew(const Eigen::Vector3d& t) {
    Eigen::Matrix3d t_hat;
    t_hat <<     0, -t[2],  t[1],
              t[2],     0, -t[0],
             -t[1],  t[0],     0;
    return t_hat;
}

inline Eigen::Vector3d wedge(const Eigen::Matrix3d& A) {
    Eigen::Vector3d vec;
    vec << A(2,1), A(0,2), A(1,0);
    return vec;
}

struct Trajectory {
    std::vector<Eigen::Vector4d> q;       // Position and yaw
    std::vector<Eigen::Vector4d> dot_q;   // Velocity and yaw rate
    std::vector<Eigen::Vector4d> ddot_q;  // Acceleration and yaw acceleration
};

Trajectory generateHelicalTrajectory(double R, double omega, double vz, double T, double Ts) {
    int n = static_cast<int>(T / Ts);
    Trajectory traj;
    traj.q.resize(n, Eigen::Vector4d::Zero());
    traj.dot_q.resize(n, Eigen::Vector4d::Zero());
    traj.ddot_q.resize(n, Eigen::Vector4d::Zero());

    double x0 = R * cos(0);
    double y0 = R * sin(0);  

    for (int i = 0; i < n; ++i) {
        double t = i * Ts;

        double x     = R * cos(omega * t) - x0;
        double y     = R * sin(omega * t) - y0;
        double z     = vz * t;
        double yaw   = omega * t;

        double dx    = -R * omega * sin(omega * t);
        double dy    = R * omega * cos(omega * t);
        double dz    = vz;
        double dyaw  = omega;

        double ddx   = -R * omega * omega * cos(omega * t);
        double ddy   = -R * omega * omega * sin(omega * t);
        double ddz   = 0.0;
        double ddyaw = 0.0;

        traj.q[i]      << x, y, z, yaw;
        traj.dot_q[i]  << dx, dy, dz, dyaw;
        traj.ddot_q[i] << ddx, ddy, ddz, ddyaw;
    }

    return traj;
}

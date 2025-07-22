// trajectory_utils.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::Matrix;

struct UAVParameters {
    double mass;
    Matrix3d Ib;
};

struct InitialFinalConditions {
    Vector4d q0, qf, dot_q0, dot_qf, ddot_q0, ddot_qf, dddot_q0, dddot_qf;
};

struct Trajectory {
    std::vector<Vector4d> q;
    std::vector<Vector4d> dot_q;
    std::vector<Vector4d> ddot_q;
};

geometry_msgs::msg::Vector3 EigenToVector3(const Eigen::Vector3d& vec) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = vec.x();
    msg.y = vec.y();
    msg.z = vec.z();
    return msg;
}

Trajectory generateTrajectory(const InitialFinalConditions& conditions, double t_iniz, double ttot, double tdead, double Ts) {
    int n = static_cast<int>(ttot / Ts);
    int total_n = n + static_cast<int>(tdead / Ts);

    Trajectory traj;
    traj.q.resize(n, Vector4d::Zero());
    traj.dot_q.resize(n, Vector4d::Zero());
    traj.ddot_q.resize(n, Vector4d::Zero());

    Vector4d a0, a1, a2, a3, a4, a5, a6, a7;

    for (int j = 0; j < 4; j++) {
        Matrix<double, 8, 8> A;
        A << pow(t_iniz, 7), pow(t_iniz, 6), pow(t_iniz, 5), pow(t_iniz, 4), pow(t_iniz, 3), pow(t_iniz, 2), t_iniz, 1,
             pow(ttot, 7), pow(ttot, 6), pow(ttot, 5), pow(ttot, 4), pow(ttot, 3), pow(ttot, 2), ttot, 1,
             7 * pow(t_iniz, 6), 6 * pow(t_iniz, 5), 5 * pow(t_iniz, 4), 4 * pow(t_iniz, 3), 3 * pow(t_iniz, 2), 2 * t_iniz, 1, 0,
             7 * pow(ttot, 6), 6 * pow(ttot, 5), 5 * pow(ttot, 4), 4 * pow(ttot, 3), 3 * pow(ttot, 2), 2 * ttot, 1, 0,
             42 * pow(t_iniz, 5), 30 * pow(t_iniz, 4), 20 * pow(t_iniz, 3), 12 * pow(t_iniz, 2), 6 * t_iniz, 2, 0, 0,
             42 * pow(ttot, 5), 30 * pow(ttot, 4), 20 * pow(ttot, 3), 12 * pow(ttot, 2), 6 * ttot, 2, 0, 0,
             210 * pow(t_iniz, 4), 120 * pow(t_iniz, 3), 60 * pow(t_iniz, 2), 24 * t_iniz, 6, 0, 0, 0,
             210 * pow(ttot, 4), 120 * pow(ttot, 3), 60 * pow(ttot, 2), 24 * ttot, 6, 0, 0, 0;

        Matrix<double, 8, 1> b;
        b << conditions.q0(j), conditions.qf(j), conditions.dot_q0(j), conditions.dot_qf(j),
             conditions.ddot_q0(j), conditions.ddot_qf(j), conditions.dddot_q0(j), conditions.dddot_qf(j);

        Matrix<double, 8, 1> a_temp = A.fullPivLu().solve(b);

        a7(j) = a_temp(0);
        a6(j) = a_temp(1);
        a5(j) = a_temp(2);
        a4(j) = a_temp(3);
        a3(j) = a_temp(4);
        a2(j) = a_temp(5);
        a1(j) = a_temp(6);
        a0(j) = a_temp(7);
    }

    for (int i = 0; i < n; i++) {
        double t = i * Ts;
        for (int j = 0; j < 4; j++) {
            traj.q[i][j] = a7(j) * pow(t, 7) + a6(j) * pow(t, 6) + a5(j) * pow(t, 5) + a4(j) * pow(t, 4) + a3(j) * pow(t, 3) + a2(j) * pow(t, 2) + a1(j) * t + a0(j);
            traj.dot_q[i][j] = 7 * a7(j) * pow(t, 6) + 6 * a6(j) * pow(t, 5) + 5 * a5(j) * pow(t, 4) + 4 * a4(j) * pow(t, 3) + 3 * a3(j) * pow(t, 2) + 2 * a2(j) * t + a1(j);
            traj.ddot_q[i][j] = 42 * a7(j) * pow(t, 5) + 30 * a6(j) * pow(t, 4) + 20 * a5(j) * pow(t, 3) + 12 * a4(j) * pow(t, 2) + 6 * a3(j) * t + 2 * a2(j);
        }
    }

    traj.q.resize(total_n, traj.q[n - 1]);
    traj.dot_q.resize(total_n, traj.dot_q[n - 1]);
    traj.ddot_q.resize(total_n, traj.ddot_q[n - 1]);

    RCLCPP_INFO(rclcpp::get_logger("trajectory_utils"), "Trajectory generated successfully");
    return traj;
}

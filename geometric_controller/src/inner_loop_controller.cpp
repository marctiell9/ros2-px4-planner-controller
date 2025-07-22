
#include "geometric_controller/math_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <algorithm>
#include "rclcpp/qos.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using Eigen::Vector3d;
using Eigen::Matrix3d;

class InnerLoopController : public rclcpp::Node {
public:
    InnerLoopController() : Node("inner_loop_controller") {
        // Gains
        Kr_ << 2, 0, 0,
               0, 2.3, 0,
               0, 0, 0.15;

        Kw_ << 0.4, 0, 0,
               0, 0.52, 0,
               0, 0, 0.18;

        Ib_ << 0.02166666666666667, 0, 0,
                0, 0.02166666666666667, 0,
                0, 0, 0.04000000000000001;

        zb_des_ << 0, 0, 1;
        Rb_des_.setIdentity();
        omega_bb_des_.setZero();
        ang_vel_.setZero();
        Rb_.setIdentity();
        dot_Rb_des.setIdentity();
        Rb_des_old_.setIdentity();
        psi_ = 0.0;
        u_T_ = 0.0;
        dt_ = 0.001;
        prev_omega_bb_des_ << 0, 0, 0;

        // PX4 thrust and torque normalization limits
        max_thrust_ = 34.2f;                          
        max_torque_ << 4.27, 4.27, 0.274;                

        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

        // Subscribers
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile, std::bind(&InnerLoopController::odom_callback, this, _1));

        zb_des_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/geom_control/zb_des", 10, std::bind(&InnerLoopController::zb_des_callback, this, _1));

        psi_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/geom_control/psi_des", 10, std::bind(&InnerLoopController::psi_callback, this, _1));

        ut_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/geom_control/u_T", 10, std::bind(&InnerLoopController::ut_callback, this, _1));

        // Publishers
        torque_pub_ = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
            "/fmu/in/vehicle_torque_setpoint", 10);

        thrust_pub_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
            "/fmu/in/vehicle_thrust_setpoint", 10);

        // Timer
        timer_ = this->create_wall_timer(1ms, std::bind(&InnerLoopController::control_loop, this));
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr zb_des_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr psi_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ut_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    Matrix3d Rb_, Rb_des_, Kr_, Kw_, Ib_, dot_Rb_des, Rb_des_old_;
    Vector3d zb_des_, omega_bb_des_, ang_vel_, max_torque_, prev_omega_bb_des_;
    float psi_, u_T_, max_thrust_, dt_;

    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        Rb_ = q.toRotationMatrix();
        ang_vel_ << msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2];
    }

    void zb_des_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        zb_des_ << msg->x, msg->y, msg->z;
    }

    void psi_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        psi_ = msg->data;
    }

    void ut_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        u_T_ = msg->data;
    }

    void control_loop() {
        Vector3d xb_des(std::cos(psi_), std::sin(psi_), 0);
        Vector3d yb_des = (skew(zb_des_) * xb_des)/(skew(zb_des_) * xb_des).norm();
        Vector3d xb_true = (yb_des.cross(zb_des_)).normalized();
        Rb_des_.col(0) = xb_true;
        Rb_des_.col(1) = yb_des;
        Rb_des_.col(2) = zb_des_;   

        dot_Rb_des = (Rb_des_ - Rb_des_old_) / dt_;
        Rb_des_old_ = Rb_des_;

        omega_bb_des_ = wedge(Rb_des_.transpose() * dot_Rb_des);
        Vector3d dot_omega_bb_des_ = (omega_bb_des_ - prev_omega_bb_des_) / dt_;
        prev_omega_bb_des_ = omega_bb_des_;  // save for next iteration
        
        omega_bb_des_ = Vector3d::Zero();  
        dot_omega_bb_des_ = Vector3d::Zero();

        Vector3d err_r = 0.5 * wedge((Rb_des_.transpose() * Rb_ - Rb_.transpose() * Rb_des_));
        Vector3d err_w = ang_vel_ - Rb_.transpose() * Rb_des_ * omega_bb_des_;
        Vector3d tau_b = - Kr_*err_r - Kw_*err_w + skew(ang_vel_)*Ib_*ang_vel_ - Ib_*(skew(ang_vel_)*Rb_.transpose()*Rb_des_*omega_bb_des_ - Rb_.transpose()*Rb_des_*dot_omega_bb_des_);
        
        Vector3d tau_norm;
        for (int i = 0; i < 3; ++i)
            tau_norm[i] = std::clamp(tau_b[i] / max_torque_[i], -1.0, 1.0);

        auto torque_msg = px4_msgs::msg::VehicleTorqueSetpoint();
        torque_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        torque_msg.xyz[0] = tau_norm[0];
        torque_msg.xyz[1] = tau_norm[1];
        torque_msg.xyz[2] = tau_norm[2];
        torque_pub_->publish(torque_msg);

        // Normalize and clamp thrust
        float u_T_norm = std::clamp(u_T_ / max_thrust_, -1.0f, 1.0f);
        auto thrust_msg = px4_msgs::msg::VehicleThrustSetpoint();
        thrust_msg.timestamp = torque_msg.timestamp;
        thrust_msg.xyz[0] = 0.0f;
        thrust_msg.xyz[1] = 0.0f;
        thrust_msg.xyz[2] = -u_T_norm; 
        thrust_pub_->publish(thrust_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InnerLoopController>());
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include "rclcpp/qos.hpp"

class OffboardInitNode : public rclcpp::Node {
public:
    OffboardInitNode() : Node("offboard_init_node") {
        using namespace std::chrono_literals;

        // Publishers
        vehicle_cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_ctrl_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

        // Timer
        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardInitNode::timer_callback, this));
    }

private:
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool mode_sent_ = false;
    bool armed_ = false;

    void timer_callback() {
        auto timestamp = this->get_clock()->now().nanoseconds() / 1000;

        px4_msgs::msg::OffboardControlMode mode_msg{};
        mode_msg.timestamp = timestamp;

        mode_msg.position = false;
        mode_msg.velocity = false;
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;  
        mode_msg.thrust_and_torque = true;
        offboard_ctrl_mode_pub_->publish(mode_msg);

        if (!mode_sent_) {
            px4_msgs::msg::VehicleCommand cmd_msg{};
            cmd_msg.timestamp = timestamp;
            cmd_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            cmd_msg.param1 = 1.0;  
            cmd_msg.param2 = 6.0;  
            cmd_msg.target_system = 1;
            cmd_msg.target_component = 1;
            cmd_msg.source_system = 1;
            cmd_msg.source_component = 1;
            cmd_msg.from_external = true;
            vehicle_cmd_pub_->publish(cmd_msg);
            RCLCPP_INFO(this->get_logger(), "Sent OFFBOARD mode command");
            mode_sent_ = true;
        }

        if (!armed_) {
            px4_msgs::msg::VehicleCommand arm_cmd{};
            arm_cmd.timestamp = timestamp;
            arm_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            arm_cmd.param1 = 1.0;  
            arm_cmd.target_system = 1;
            arm_cmd.target_component = 1;
            arm_cmd.source_system = 1;
            arm_cmd.source_component = 1;
            arm_cmd.from_external = true;
            vehicle_cmd_pub_->publish(arm_cmd);
            RCLCPP_INFO(this->get_logger(), "Sent ARM command");
            armed_ = true;
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardInitNode>());
    rclcpp::shutdown();
    return 0;
}

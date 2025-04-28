#ifndef FORWARD_ORION_HPP
#define FORWARD_ORION_HPP

#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <std_msgs/msg/float32.hpp>

namespace orion_control
{
    class ForwardOrion : public hardware_interface::SystemInterface
    {
    public:
        ForwardOrion() = default;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& prev_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& prev_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev_state) override;
    
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration& period) override;

    private:
        const std::string servo_sub_name_param{"feedback_topic"};
        const std::string servo_pub_name_param{"cmd_topic"};
        const std::string servo_joint_name_param{"servo_name"};

        std::string servo_sub_topic_;
        std::string servo_pub_topic_;

        struct ServoComp
        {
            std::string joint_name_;
            double feedback_;
            double cmd_;
        };

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr servo_pose_sub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_cmd_pub_;

        std_msgs::msg::Float32 servo_pose_;
        std_msgs::msg::Float32 servo_cmd_;

        rclcpp::Node::SharedPtr node_;

        ServoComp servo_;
        
        rclcpp::Logger logger_{rclcpp::get_logger("ForwardOrion")};

    };
}

#endif
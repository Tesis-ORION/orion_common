#ifndef DIFFDRIVE_ORION_HPP
#define DIFFDRIVE_ORION_HPP

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

#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>

#include "orion_control/wheel.hpp"

namespace orion_control
{
    class DiffDriveOrion : public hardware_interface::SystemInterface
    {
    public:
        DiffDriveOrion() = default;

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& prev_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& prev_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev_state) override;
    
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration& period) override;

    private:
        const std::string wheel_left_name_param{"left_wheel_name"};
        const std::string wheel_right_name_param{"right_wheel_name"};
        const std::string wheel_ticks_per_rev_param{"enc_ticks_per_rev"};

        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_left_enc;
        rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_right_enc;
        rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr pub_cmd_speed;

        rclcpp::Node::SharedPtr node_;
        std_msgs::msg::Int64MultiArray cmd_speed;
        std_msgs::msg::Int64 left_enc_;
        std_msgs::msg::Int64 right_pos_;

        struct Config
        {
            std::string left_wheel_name = "left_wheel";
            std::string right_wheel_name = "right_wheel";
            int enc_tics_per_rev = 0;
        };

        Config config_;

        Wheel left_wheel_;
        Wheel right_wheel_;
        
        rclcpp::Logger logger_{rclcpp::get_logger("DiffDriveOrion")};

    };
}

#endif
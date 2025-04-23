#include "orion_control/diffdrive_orion.hpp"

namespace orion_control
{
    hardware_interface::CallbackReturn DiffDriveOrion::on_init(const hardware_interface::HardwareInfo& info)
    {
        if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "Diff: Begin [on_init]...");

        this->config_.left_wheel_name = info_.hardware_parameters[this->wheel_left_name_param];
        this->config_.right_wheel_name = info_.hardware_parameters[this->wheel_right_name_param];
        this->config_.enc_tics_per_rev = std::stoi(info_.hardware_parameters[this->wheel_ticks_per_rev_param]);

        rclcpp::NodeOptions options;
        options.arguments({"--ros-args", "-r", "__node:=orion_ros2_control" + info_.name});

        this->pub_cmd_speed = this->node_->create_publisher<std_msgs::msg::Int64MultiArray>(
            "/diff_ctl_motor_cmd", rclcpp::QoS(1));

        

    }


    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& prev_state)
    {

    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces()
    {

    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces()
    {

    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& prev_state)
    {

    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev_state)
    {

    }

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period)
    {

    }

    hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration& period)
    {

    }
}
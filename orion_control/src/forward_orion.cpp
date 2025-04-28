#include "orion_control/forward_orion.hpp"
#include <cmath>

namespace orion_control
{
    hardware_interface::CallbackReturn ForwardOrion::on_init(const hardware_interface::HardwareInfo& info)
    {
        if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(this->logger_, "Fwd:: Begin [on_init]...");

        this->servo_sub_topic_ = info_.hardware_parameters[this->servo_sub_name_param];
        this->servo_pub_topic_ = info_.hardware_parameters[this->servo_pub_name_param];
        this->servo_.joint_name_ = info_.hardware_parameters[this->servo_joint_name_param];

        rclcpp::NodeOptions options;
        options.arguments({"--ros-args", "-r", "__node:=orion_fwd_ros2_control" + info_.name});
        
        this->node_ = rclcpp::Node::make_shared("_", options);
        
        this->servo_cmd_pub_ = this->node_->create_publisher<std_msgs::msg::Float32>(
            this->servo_pub_topic_, rclcpp::QoS(1));


        RCLCPP_INFO(this->logger_, "Fwd:: End [on_init]...");
        
        this->servo_pose_sub_= this->node_->create_subscription<std_msgs::msg::Float32>(
            this->servo_sub_topic_, rclcpp::QoS(1),
            [this](const std_msgs::msg::Float32::SharedPtr pose)
            {
                this->servo_pose_ = *pose;
            });

        return hardware_interface::CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn ForwardOrion::on_configure(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->logger_, "Fwd:: Begin [on_configure]...");
        RCLCPP_INFO(this->logger_, "Fwd:: End [on_configure]...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ForwardOrion::export_state_interfaces()
    {
        RCLCPP_INFO(this->logger_, "Fwd:: Begin [export_state_interfaces]...");
        std::vector<hardware_interface::StateInterface> state_interfaces;
        
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(this->servo_.joint_name_, 
                hardware_interface::HW_IF_POSITION, &this->servo_.feedback_));


        RCLCPP_INFO(this->logger_, "Fwd:: End [export_state_interfaces]...");

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ForwardOrion::export_command_interfaces()
    {
        RCLCPP_INFO(this->logger_, "Fwd:: Begin [export_command_interfaces]...");
        
        std::vector <hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(this->servo_.joint_name_, 
                hardware_interface::HW_IF_POSITION, &this->servo_.cmd_));
        
        RCLCPP_INFO(this->logger_, "Fwd:: End [export_command_interfaces]...");

        return command_interfaces;
    }

    hardware_interface::CallbackReturn ForwardOrion::on_activate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->logger_, "Fwd:: Begin [on_activate]...");
        RCLCPP_INFO(this->logger_, "Fwd:: End [on_activate]...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ForwardOrion::on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->logger_, "Fwd:: Begin [on_deactivate]...");
        RCLCPP_INFO(this->logger_, "Fwd:: End [on_deactivate]...");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ForwardOrion::read(const rclcpp::Time&, const rclcpp::Duration&)
    {
        RCLCPP_DEBUG(this->logger_, "Fwd:: Begin [read]...");

        if(rclcpp::ok())
        {
            rclcpp::spin_some(this->node_);
        }

        // objective =  objective * (M_PI / 180) - M_PI/2
        this->servo_.feedback_ = this->servo_pose_.data;

        RCLCPP_DEBUG(this->logger_, "Fwd:: End [read]...");
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ForwardOrion::write(const rclcpp::Time&, const rclcpp::Duration&)
    {
        RCLCPP_DEBUG(this->logger_, "Fwd:: Begin [write]...");

        std_msgs::msg::Float32 objective;
        objective.data = (float) this->servo_.cmd_;
        // objective.data = (objective + M_PI/2) * (180.0 / M_PI)

        if(rclcpp::ok())
        {
            this->servo_cmd_pub_->publish(objective);
        }

        RCLCPP_DEBUG(this->logger_, "Fwd:: Begin [write]...");
        return hardware_interface::return_type::OK;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(orion_control::ForwardOrion, hardware_interface::SystemInterface)
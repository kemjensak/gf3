#include "gf3_hardware/gf3_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// #include "hw_can_bridge.hpp"

namespace gf3_hardware
{

  CallbackReturn Gf3HardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
   
    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    prev_hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
    if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end())
    {
      return it->second;
    }
    return default_value;
  };
  
    // Add random ID to prevent warnings about multiple publishers within the same node
  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=topic_based_ros2_control_" + info_.name });

  node_ = rclcpp::Node::make_shared("_", options);

  // TODO: specify Moteus msg type
  topic_based_joint_commands_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_commands_topic", "/robot_joint_commands"), rclcpp::QoS(1));
  topic_based_joint_states_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_states_topic", "/robot_joint_states"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr joint_state)
      { latest_joint_state_ = *joint_state; });

  return CallbackReturn::SUCCESS;
  }

  CallbackReturn Gf3HardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Successfully configured!");

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  Gf3HardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  Gf3HardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  CallbackReturn Gf3HardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // command and state should be equal when starting
    for (uint i = 0; i < hw_states_.size(); i++)
    {
      hw_commands_[i] = hw_states_[i];
    }

    RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "Successfully activated!");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn Gf3HardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Gf3HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // read the state from the hardware
    // RCLCPP_INFO(rclcpp::get_logger("Gf3HardwareInterface"), "read!");

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Gf3HardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // TODO: topic publisher, MoteusCommandArray
    // send one position with speed and torque limits
    double stop_position = 0;
    double velocity = 0.05;
    double max_torque = 1;
    double feedforward_torque = 0;
    return hardware_interface::return_type::OK;
  }

} // namespace

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    gf3_hardware::Gf3HardwareInterface, hardware_interface::SystemInterface)
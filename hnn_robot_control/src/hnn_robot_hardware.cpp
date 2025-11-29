// Copyright 2024 Hnn Robot Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hnn_robot_control/hnn_robot_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hnn_robot_control
{
hardware_interface::CallbackReturn HnnRobotHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read robot_type parameter
  if (info_.hardware_parameters.find("robot_type") == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(
      get_logger(), "Parameter 'robot_type' not found in hardware parameters. "
      "Please specify robot_type in the ros2_control xacro file.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  robot_type_ = info_.hardware_parameters["robot_type"];
  RCLCPP_INFO(get_logger(), "Initializing hardware for robot type: '%s'", robot_type_.c_str());

  // Validate robot_type
  if (robot_type_ != "hnn_diff_robot")
  {
    RCLCPP_FATAL(
      get_logger(), "Unknown robot_type: '%s'. Supported types: hnn_diff_robot",
      robot_type_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from hardware parameters
  // TODO: Add required parameters for Hnn robot (serial port, baud rate, etc.)
  hw_start_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ =
    hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // HnnRobotHardware has velocity command interface and position/velocity state interfaces
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HnnRobotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Hnn Robot Hardware for '%s'...", robot_type_.c_str());

  // Route to robot-specific configure method
  if (robot_type_ == "hnn_diff_robot")
  {
    return configure_hnn_diff_robot();
  }

  RCLCPP_FATAL(get_logger(), "Unsupported robot_type in configure: '%s'", robot_type_.c_str());
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn HnnRobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Hnn Robot Hardware for '%s'...", robot_type_.c_str());

  // Route to robot-specific activate method
  if (robot_type_ == "hnn_diff_robot")
  {
    return activate_hnn_diff_robot();
  }

  RCLCPP_FATAL(get_logger(), "Unsupported robot_type in activate: '%s'", robot_type_.c_str());
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn HnnRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Hnn Robot Hardware for '%s'...", robot_type_.c_str());

  // Route to robot-specific deactivate method
  if (robot_type_ == "hnn_diff_robot")
  {
    return deactivate_hnn_diff_robot();
  }

  RCLCPP_FATAL(get_logger(), "Unsupported robot_type in deactivate: '%s'", robot_type_.c_str());
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type HnnRobotHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Route to robot-specific read method
  if (robot_type_ == "hnn_diff_robot")
  {
    return read_hnn_diff_robot(time, period);
  }

  RCLCPP_ERROR(get_logger(), "Unsupported robot_type in read: '%s'", robot_type_.c_str());
  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type HnnRobotHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Route to robot-specific write method
  if (robot_type_ == "hnn_diff_robot")
  {
    return write_hnn_diff_robot(time, period);
  }

  RCLCPP_ERROR(get_logger(), "Unsupported robot_type in write: '%s'", robot_type_.c_str());
  return hardware_interface::return_type::ERROR;
}

// ============================================================================
// Hnn Diff specific implementations
// ============================================================================

hardware_interface::CallbackReturn HnnRobotHardware::configure_hnn_diff_robot()
{
  RCLCPP_INFO(get_logger(), "Configuring Hnn Diff Hardware...");

  // TODO: Initialize connection with real Hnn Diff robot
  // Example: open serial port, connect network, initialize API client
  // if (!initialize_hnn_diff_robot_connection()) {
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  // Reset values when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured Hnn Diff Hardware!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HnnRobotHardware::activate_hnn_diff_robot()
{
  RCLCPP_INFO(get_logger(), "Activating Hnn Diff Hardware...");

  // TODO: Activate real Hnn Diff robot
  // Example: enable motors, enable controllers

  // Command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated Hnn Diff Hardware!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HnnRobotHardware::deactivate_hnn_diff_robot()
{
  RCLCPP_INFO(get_logger(), "Deactivating Hnn Diff Hardware...");

  // TODO: Stop real Hnn Diff robot
  // Example: disable motors, disable controllers

  RCLCPP_INFO(get_logger(), "Successfully deactivated Hnn Diff Hardware!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HnnRobotHardware::read_hnn_diff_robot(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // TODO: Read data from real Hnn Diff robot
  // Example: read encoder values, position, velocity from hardware
  // Then update state interfaces:
  // set_state(joint_name, position_value);
  // set_state(joint_name + "/velocity", velocity_value);

  // Temporary: simulate reading data (similar to example)
  // Replace this section with actual code that reads from hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      // Temporary: integrate velocity to calculate position
      // In practice, you will read position from Hnn Diff robot encoder
      auto velo = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_VELOCITY);
      set_state(name, get_state(name) + period.seconds() * velo);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HnnRobotHardware::write_hnn_diff_robot(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO: Send commands to real Hnn Diff robot
  // Example: send velocity commands to motors
  // for (const auto & [name, descr] : joint_command_interfaces_) {
  //   double velocity_cmd = get_command(name);
  //   send_velocity_command_to_hnn_diff_robot(name, velocity_cmd);
  // }

  // Temporary: simulate sending commands
  // Replace this section with actual code that sends commands to hardware
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    // Temporary: update state to simulate
    // In practice, you will send velocity command to Hnn Diff robot
    double cmd = get_command(name);
    (void)cmd;  // Suppress unused variable warning until implementation
    // send_velocity_to_hnn_diff_robot(name, cmd);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hnn_robot_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hnn_robot_control::HnnRobotHardware, hardware_interface::SystemInterface)


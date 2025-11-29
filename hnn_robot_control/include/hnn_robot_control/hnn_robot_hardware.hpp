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

#ifndef HNN_ROBOT_CONTROL__HNN_ROBOT_HARDWARE_HPP_
#define HNN_ROBOT_CONTROL__HNN_ROBOT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hnn_robot_control
{
class HnnRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HnnRobotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Robot type identifier
  std::string robot_type_;

  // Robot-specific configuration methods
  hardware_interface::CallbackReturn configure_hnn_diff_robot();
  hardware_interface::CallbackReturn activate_hnn_diff_robot();
  hardware_interface::CallbackReturn deactivate_hnn_diff_robot();
  hardware_interface::return_type read_hnn_diff_robot(
    const rclcpp::Time & time, const rclcpp::Duration & period);
  hardware_interface::return_type write_hnn_diff_robot(
    const rclcpp::Time & time, const rclcpp::Duration & period);

  // TODO: Add variables for communication with real Hnn robot
  // Example: serial port, network socket, or API client
  // std::string serial_port_;
  // int baud_rate_;
  
  // Parameters
  double hw_start_sec_;
  double hw_stop_sec_;
};

}  // namespace hnn_robot_control

#endif  // HNN_ROBOT_CONTROL__HNN_ROBOT_HARDWARE_HPP_


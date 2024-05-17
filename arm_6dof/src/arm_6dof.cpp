// Copyright (c) 2024, snow10100
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#include <limits>
#include <vector>

#include "arm_6dof/arm_6dof.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_6dof
{
hardware_interface::CallbackReturn Arm6DOF::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arm6DOF::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces
  hw_commands_.resize(6, 0.0);

  // Initialize the serial port
  serial_port_.setPort("/dev/ttyACM0");
  serial_port_.setBaudrate(9600);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
  serial_port_.setTimeout(timeout);

  try {
    serial_port_.open();
  } catch (const serial::IOException &e) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Unable to open port ");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Arm6DOF::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Arm6DOF::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Arm6DOF::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands
  RCLCPP_INFO(rclcpp::get_logger("Arm6DOF"), "Activating ...please wait...");

  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    if (std::isnan(hw_commands_[i])) {
      hw_commands_[i] = 0.0;
    }
  }
  for (size_t i = 0; i < hw_states_.size(); ++i) {
    if (std::isnan(hw_states_[i])) {
      hw_states_[i] = 0.0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("Arm6DOF"), "Successfully activated!");


  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arm6DOF::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Arm6DOF::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Arm6DOF::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): write robot's commands'
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    std::string command = "M" + std::to_string(i+1) + "," + std::to_string(static_cast<int>(hw_commands_[i])) + "\n";
    serial_port_.write(command);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arm_6dof

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_6dof::Arm6DOF, hardware_interface::SystemInterface)

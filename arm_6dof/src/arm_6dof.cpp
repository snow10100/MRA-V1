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
#include <cmath>

#include "arm_6dof/arm_6dof.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_6dof
{

// Convert radians to stepper motor steps (based on your Arduino code)
// Assuming 400 microsteps/rev * 38.4 ratio = 15360 steps/rev
const double STEPS_PER_RADIAN = 15360.0 / (2.0 * M_PI);
const int MAX_STEPS = 7680;  // ±π radians

int radiansToSteps(double radians)
{
  int steps = static_cast<int>(radians * STEPS_PER_RADIAN);
  return std::max(-MAX_STEPS, std::min(MAX_STEPS, steps));
}

double stepsToRadians(int steps)
{
  return static_cast<double>(steps) / STEPS_PER_RADIAN;
}

hardware_interface::CallbackReturn Arm6DOF::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize state and command vectors
  hw_states_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  
  // Store previous commands to detect changes
  prev_commands_.resize(info_.joints.size(), 0.0);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Arm6DOF::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters from hardware_info
  std::string port = "/dev/ttyACM0";
  int baudrate = 9600;
  
  // Check if custom port is specified in hardware info
  if (info_.hardware_parameters.find("device") != info_.hardware_parameters.end()) {
    port = info_.hardware_parameters["device"];
  }
  if (info_.hardware_parameters.find("baudrate") != info_.hardware_parameters.end()) {
    baudrate = std::stoi(info_.hardware_parameters["baudrate"]);
  }

  RCLCPP_INFO(rclcpp::get_logger("Arm6DOF"), "Connecting to %s at %d baud", port.c_str(), baudrate);

  // Initialize the serial port
  try {
    serial_port_.setPort(port);
    serial_port_.setBaudrate(baudrate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    serial_port_.setTimeout(timeout);
    serial_port_.open();
    
    if (!serial_port_.isOpen()) {
      RCLCPP_ERROR(rclcpp::get_logger("Arm6DOF"), "Failed to open serial port %s", port.c_str());
      return CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("Arm6DOF"), "Successfully connected to Arduino");
    
    // Wait for Arduino to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Query current positions from Arduino
    queryCurrentPositions();
    
  } catch (const serial::IOException &e) {
    RCLCPP_ERROR(rclcpp::get_logger("Arm6DOF"), "Unable to open port %s: %s", port.c_str(), e.what());
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
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Arm6DOF::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Arm6DOF"), "Activating hardware interface...");

  // Initialize commands and states to current positions
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    if (std::isnan(hw_commands_[i])) {
      hw_commands_[i] = hw_states_[i];
    }
    prev_commands_[i] = hw_commands_[i];
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
  RCLCPP_INFO(rclcpp::get_logger("Arm6DOF"), "Deactivating hardware interface...");
  
  // Stop all motors
  try {
    for (size_t i = 0; i < hw_commands_.size(); ++i) {
      std::string command = "M" + std::to_string(i+1) + ",1,STOP\n";
      serial_port_.write(command);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("Arm6DOF"), "Failed to stop motors: %s", e.what());
  }
  
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Arm6DOF::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read current positions from Arduino periodically
  static auto last_read = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  
  // Query positions every 100ms to avoid overwhelming the Arduino
  if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read).count() > 100) {
    if (serial_port_.isOpen()) {
      try {
        queryCurrentPositions();
        last_read = now;
      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("Arm6DOF"), 
                             *rclcpp::Clock().get_clock(), 
                             5000, 
                             "Failed to read from Arduino: %s", e.what());
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Arm6DOF::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_.isOpen()) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    // Send commands only if they have changed significantly
    for (size_t i = 0; i < hw_commands_.size(); ++i) {
      double command_diff = std::abs(hw_commands_[i] - prev_commands_[i]);
      
      // Only send command if change is significant (> 0.01 radians ≈ 0.57 degrees)
      if (command_diff > 0.01) {
        int steps = radiansToSteps(hw_commands_[i]);
        
        // Use moveTo command (M1,2,steps) for position control
        std::string command = "M" + std::to_string(i+1) + ",2," + std::to_string(steps) + "\n";
        serial_port_.write(command);
        
        prev_commands_[i] = hw_commands_[i];
        
        RCLCPP_DEBUG(rclcpp::get_logger("Arm6DOF"), 
                    "Joint %zu: %.3f rad -> %d steps", i+1, hw_commands_[i], steps);
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("Arm6DOF"), 
                          *rclcpp::Clock().get_clock(), 
                          1000, 
                          "Failed to write to Arduino: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void Arm6DOF::queryCurrentPositions()
{
  if (!serial_port_.isOpen()) return;
  
  try {
    // Clear input buffer
    serial_port_.flushInput();
    
    // Query each joint position
    for (size_t i = 0; i < hw_states_.size(); ++i) {
      std::string query = "M" + std::to_string(i+1) + ",3\n";
      serial_port_.write(query);
      
      // Small delay between queries
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Read responses
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    if (serial_port_.available()) {
      std::string response = serial_port_.read(serial_port_.available());
      parsePositionResponse(response);
    }
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("Arm6DOF"), "Failed to query positions: %s", e.what());
  }
}

void Arm6DOF::parsePositionResponse(const std::string& response)
{
  std::istringstream iss(response);
  std::string line;
  
  while (std::getline(iss, line)) {
    // Look for lines like "M1, steps, 1234"
    if (line.find(", steps, ") != std::string::npos) {
      try {
        // Extract motor number and steps
        size_t m_pos = line.find('M');
        size_t comma_pos = line.find(',');
        size_t steps_pos = line.find(", steps, ");
        
        if (m_pos != std::string::npos && comma_pos != std::string::npos && 
            steps_pos != std::string::npos) {
          
          int motor_num = std::stoi(line.substr(m_pos + 1, comma_pos - m_pos - 1));
          int steps = std::stoi(line.substr(steps_pos + 9));
          
          if (motor_num >= 1 && motor_num <= static_cast<int>(hw_states_.size())) {
            hw_states_[motor_num - 1] = stepsToRadians(steps);
          }
        }
      } catch (const std::exception& e) {
        RCLCPP_DEBUG(rclcpp::get_logger("Arm6DOF"), "Failed to parse line: %s", line.c_str());
      }
    }
  }
}

}  // namespace arm_6dof

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_6dof::Arm6DOF, hardware_interface::SystemInterface)
// Copyright 2021 ros2_control Development Team
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

#include "isaac_ros2_control/isaac_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace isaac_ros2_control
{
  CallbackReturn IsaacSystem::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    old_rad_.resize(info_.joints.size(), 0.0f);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // IsaacSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("IsaacSystem"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("IsaacSystem"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 3)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("IsaacSystem"),
            "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("IsaacSystem"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("IsaacSystem"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("IsaacSystem"),
            "Joint '%s' have '%s' as third state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
        return CallbackReturn::ERROR;
      }
    }

    shm.openMemory(info.name);

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> IsaacSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> IsaacSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
  }

  CallbackReturn IsaacSystem::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("IsaacSystem"), "Starting ...please wait...");

    // set some default values
    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
      if (std::isnan(hw_positions_[i]))
      {
        hw_positions_[i] = 0;
        hw_velocities_[i] = 0;
        hw_commands_[i] = 0;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("IsaacSystem"), "System Successfully started!");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn IsaacSystem::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("IsaacSystem"), "Stopping ...please wait...");

    RCLCPP_INFO(rclcpp::get_logger("IsaacSystem"), "System successfully stopped!");

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type IsaacSystem::read(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    for (uint i = 0; i < hw_commands_.size(); i++)
    {
      float rad = 0.0f;
      float diff_rad = 0.0f;
      shm.readRad(i, &rad);
      if (fabs(rad - old_rad_[i]) < 3.14f)
      {
        diff_rad = rad - old_rad_[i];
      }
      else
      {
        diff_rad = rad + old_rad_[i];
      }
      old_rad_[i] = rad;
      hw_positions_[i] += static_cast<double>(diff_rad);
      float radps = 0.0f;
      shm.readRadps(i, &radps);
      hw_velocities_[i] = static_cast<double>(radps);
      float torque = 0.0f;
      shm.readTorque(i, &torque);
      hw_efforts_[i] = static_cast<double>(torque);
      /*
          RCLCPP_INFO(
            rclcpp::get_logger("IsaacSystem"),
            "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
            hw_velocities_[i], info_.joints[i].name.c_str());
      */
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type IsaacSystem::write(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    for (auto i = 0u; i < hw_commands_.size(); i++)
    {
      // Generate the motor command message
      shm.writeRadps(i, hw_commands_[i]);
    }

    return hardware_interface::return_type::OK;
  }

} // namespace isaac_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    isaac_ros2_control::IsaacSystem, hardware_interface::SystemInterface)

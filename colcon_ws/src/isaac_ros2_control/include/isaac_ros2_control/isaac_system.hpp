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

#ifndef ISAAC_ROS2_CONTROL__ISAAC_SYSTEM_HPP_
#define ISAAC_ROS2_CONTROL__ISAAC_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "isaac_ros2_control/visibility_control.h"
#include <isaac_ros2_control/shm_comunicator.h>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace isaac_ros2_control
{
class IsaacSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IsaacSystem);

  ISAAC_SYSTEM_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ISAAC_SYSTEM_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ISAAC_SYSTEM_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ISAAC_SYSTEM_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ISAAC_SYSTEM_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ISAAC_SYSTEM_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ISAAC_SYSTEM_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  rclcpp::Duration getPeriod() const { return rclcpp::Duration(0, 10000000); } // loop period is 0.01s


private:
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  
  std::vector<float> old_rad_;

  ShmComunicator shm;
};

}  // namespace isaac_ros2_control

#endif  // ISAAC_ROS2_CONTROL__ISAAC_SYSTEM_HPP_

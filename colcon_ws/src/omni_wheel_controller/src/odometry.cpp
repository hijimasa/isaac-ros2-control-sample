// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Enrique Fernández
 */

#include <iostream>
#include <cmath>
#include "omni_wheel_controller/odometry.hpp"

#include <Eigen/LU>

namespace omni_wheel_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0)
, x_(0.0)
, y_(0.0)
, heading_(0.0)
, lin_x_(0.0)
, lin_y_(0.0)
, angular_(0.0)
, velocity_rolling_window_size_(velocity_rolling_window_size)
, linear_x_accumulator_(velocity_rolling_window_size)
, linear_y_accumulator_(velocity_rolling_window_size)
, angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(std::vector<double> omni_wheel_pos, const rclcpp::Time & time)
{
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  /// Get current wheel joint positions:
  std::vector<double> omni_wheel_cur_pos;
  for (size_t i = 0; i < omni_wheel_pos.size(); ++i)
  {
    omni_wheel_cur_pos.push_back( omni_wheel_pos[i]);
  }

  /// Estimate movement amount of wheels using old and current position:
  Eigen::VectorXd wheel_movement_vector(omni_wheel_pos.size());
  for (size_t i = 0; i < omni_wheel_pos.size(); i++)
  {
    wheel_movement_vector(i) = omni_wheel_cur_pos[i] - omni_wheel_old_pos_[i];
  }

  /// Update old position with current:
  omni_wheel_old_pos_  = omni_wheel_cur_pos;
  /// Compute linear and angular diff:
  Eigen::VectorXd robot_movement_vector = motion_matrix_inverse_ * wheel_movement_vector;
  const double lin_x = robot_movement_vector(0);
  const double lin_y = robot_movement_vector(1);
  const double angular = robot_movement_vector(2);
//  std::cout << "lin_x: " << lin_x << ", lin_y: " << lin_y << ", ang: " << angular << std::endl;

  // Integrate odometry:
  integrateExact(lin_x, lin_y, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_x_accumulator_.accumulate(lin_x / dt);
  linear_y_accumulator_.accumulate(lin_x / dt);
  angular_accumulator_.accumulate(angular / dt);

  lin_x_ = linear_x_accumulator_.getRollingMean();
  lin_y_ = linear_y_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double lin_x, double lin_y, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity:
  lin_x_ = lin_x;
  lin_y_ = lin_y;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(lin_x * dt, lin_y * dt, angular * dt);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(double omni_wheel_distance, double omni_wheel_radius, std::vector<double> omni_wheel_yaw)
{
  motion_matrix_ = Eigen::MatrixXd::Zero(omni_wheel_yaw.size(), 3);
  for (size_t row = 0; row < omni_wheel_yaw.size(); row++)
  {
    double roller_slip_direction = omni_wheel_yaw[row] - M_PI / 2.0;
    motion_matrix_(row, 0) = cos(roller_slip_direction) / omni_wheel_radius;
    motion_matrix_(row, 1) = sin(roller_slip_direction) / omni_wheel_radius;
    motion_matrix_(row, 2) = - omni_wheel_distance / omni_wheel_radius;
  }
  motion_matrix_inverse_ = motion_matrix_.completeOrthogonalDecomposition().pseudoInverse();
  
  omni_wheel_old_pos_.resize(omni_wheel_yaw.size());
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double lin_x, double lin_y, double angular)
{
  const double direction = heading_ + angular * 0.5;
  /// Runge-Kutta 2nd order integration:
  x_       += lin_x * cos(direction) - lin_y * sin(direction);
  y_       += lin_x * sin(direction) + lin_y * cos(direction);
  heading_ += angular;
}

void Odometry::integrateExact(double lin_x, double lin_y, double angular)
{
  if (fabs(angular) < 1e-6)
  {
    integrateRungeKutta2(lin_x, lin_y, angular);
  }
  else
  {
    /// Exact integration (should solve problems when angular is zero):
    const double heading_old = heading_;
    const double r_x = lin_x/angular;
    const double r_y = lin_y/angular;
    heading_ += angular;
    x_       +=  r_x * (sin(heading_) - sin(heading_old));
    x_       +=  r_y * (cos(heading_) - cos(heading_old));
    y_       += -r_x * (cos(heading_) - cos(heading_old));
    y_       += -r_y * (-sin(heading_) + sin(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace omni_wheel_controller

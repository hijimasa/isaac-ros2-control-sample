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
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 */

#ifndef OMNI_WHEEL_CONTROLLER__ODOMETRY_HPP_
#define OMNI_WHEEL_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
//#include "rcpputils/rolling_mean_accumulator.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

#include <Eigen/QR>

namespace omni_wheel_controller
{
class Odometry
{
public:
  /**
   * \brief Constructor
   * Timestamp will get the current time value
   * Value will be set to zero
   * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
   */
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  /**
   * \brief Initialize the odometry
   * \param time Current time
   */
  void init(const rclcpp::Time & time);
  /**
   * \brief Updates the odometry class with latest wheels position
   * \param omni_wheel_pos  Omni wheel positions [rad]
   * \param time      Current time
   * \return true if the odometry is actually updated
   */
  bool update(std::vector<double> omni_wheel_pos, const rclcpp::Time &time);
  /**
   * \brief Updates the odometry class with latest velocity command
   * \param linear  Linear velocity [m/s]
   * \param angular Angular velocity [rad/s]
   * \param time    Current time
   */
  void updateOpenLoop(double lin_x, double lin_y, double angular, const rclcpp::Time &time);
  void resetOdometry();

  /**
   * \brief x position getter
   * \return x position [m]
   */
  double getX() const
  {
    return x_;
  }
  /**
   * \brief y position getter
   * \return y position [m]
   */
  double getY() const
  {
    return y_;
  }
  /**
   * \brief heading getter
   * \return heading [rad]
   */
  double getHeading() const { return heading_; }
  /**
   * \brief linear x velocity getter
   * \return linear velocity [m/s]
   */
  double getLinearX() const
  {
    return lin_x_;
  }
  /**
   * \brief linear y velocity getter
   * \return linear velocity [m/s]
   */
  double getLinearY() const
  {
    return lin_y_;
  }
  /**
   * \brief angular velocity getter
   * \return angular velocity [rad/s]
   */
  double getAngular() const
  {
    return angular_;
  }
    
  /**
   * \brief motion matrix getter
   * \return motion matrix
   */
  const Eigen::MatrixXd& getMotionMatrix() const
  {
    return motion_matrix_;
  }

  /**
   * \brief Sets the wheel parameters: radius and separation
   * \param wheel_separation   Separation between left and right wheels [m]
   * \param left_wheel_radius  Left wheel radius [m]
   * \param right_wheel_radius Right wheel radius [m]
   */
  void setWheelParams(double omni_wheel_distance, double omni_wheel_radius, std::vector<double> omni_wheel_yaw);
  /**
   * \brief Velocity rolling window size setter
   * \param velocity_rolling_window_size Velocity rolling window size
   */
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
//  using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;
  using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

  void integrateRungeKutta2(double lin_x, double lin_y, double angular);
  void integrateExact(double lin_x, double lin_y, double angular);
  void resetAccumulators();

    /// Current timestamp:
    rclcpp::Time timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double lin_x_;  //   [m/s]
    double lin_y_;  //   [m/s]
    double angular_; // [rad/s]

    /// Wheel kinematic parameters [m]:
    Eigen::MatrixXd motion_matrix_;
    Eigen::MatrixXd motion_matrix_inverse_;

    /// Previou wheel position/state [rad]:
    std::vector<double> omni_wheel_old_pos_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAccumulator linear_x_accumulator_;
    RollingMeanAccumulator linear_y_accumulator_;
    RollingMeanAccumulator angular_accumulator_;
};

}  // namespace omni_wheel_controller

#endif  // OMNI_WHEEL_CONTROLLER__ODOMETRY_HPP_

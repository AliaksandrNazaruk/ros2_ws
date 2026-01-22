// Copyright 2026 Boris
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

#pragma once

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "aehub_msgs/msg/robot_status.hpp"

namespace aehub::robot
{

/**
 * @brief SafetyWatcher - observes robot safety state (motors, e-stop)
 *
 * This is a flexible interface that can be extended for different robot types.
 * Default implementation uses parameter-based checks or topic subscriptions.
 *
 * Pure observer - never publishes commands.
 */
class SafetyWatcher
{
public:
  explicit SafetyWatcher(const rclcpp::Node::SharedPtr & node);

  /**
   * @brief Check if motors are enabled
   * @return true if motors are enabled, false otherwise
   */
  bool motorsEnabled() const;

  /**
   * @brief Check if emergency stop is active
   * @return true if e-stop is active (NOT safe), false if inactive (safe)
   */
  bool emergencyStopActive() const;

  // True if RobotStatus was received recently enough.
  bool hasFreshStatus() const;

private:
  rclcpp::Node::SharedPtr node_;

  std::string robot_status_topic_;
  double status_stale_timeout_s_{1.0};

  rclcpp::Subscription<aehub_msgs::msg::RobotStatus>::SharedPtr status_sub_;

  mutable std::mutex mutex_;
  bool status_received_{false};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};
  aehub_msgs::msg::RobotStatus last_status_;

  void onStatus(const aehub_msgs::msg::RobotStatus::SharedPtr msg);
};

}  // namespace aehub::robot

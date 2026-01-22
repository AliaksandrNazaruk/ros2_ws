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

#include "aehub_robot_readiness/safety_watcher.hpp"

namespace aehub::robot
{

SafetyWatcher::SafetyWatcher(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  if (!node_) {
    throw std::runtime_error("SafetyWatcher: node is null");
  }

  robot_status_topic_ = node_->declare_parameter<std::string>("robot_status_topic", "robot/status");
  status_stale_timeout_s_ = node_->declare_parameter<double>("robot_status_stale_timeout_s", 1.0);

  status_sub_ = node_->create_subscription<aehub_msgs::msg::RobotStatus>(
    robot_status_topic_,
    rclcpp::QoS(1).transient_local(),
    std::bind(&SafetyWatcher::onStatus, this, std::placeholders::_1));
}

void SafetyWatcher::onStatus(const aehub_msgs::msg::RobotStatus::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  last_status_ = *msg;
  status_received_ = true;
  last_status_time_ = node_->now();
}

bool SafetyWatcher::hasFreshStatus() const
{
  if (!node_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!status_received_) {
    return false;
  }
  const double age = (node_->now() - last_status_time_).seconds();
  return age >= 0.0 && age <= status_stale_timeout_s_;
}

bool SafetyWatcher::motorsEnabled() const
{
  if (!node_) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!status_received_) {
    return false;
  }
  if (!last_status_.motors_enabled_valid) {
    return false;
  }
  return last_status_.motors_enabled;
}

bool SafetyWatcher::emergencyStopActive() const
{
  if (!node_) {
    return true;  // If no node, assume unsafe
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!status_received_) {
    return true;  // No status -> unsafe
  }
  if (!last_status_.estop_active_valid) {
    return true;  // Unknown -> unsafe
  }
  return last_status_.estop_active;
}

}  // namespace aehub::robot

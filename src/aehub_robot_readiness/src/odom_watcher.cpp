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

#include "aehub_robot_readiness/odom_watcher.hpp"

#include <chrono>

namespace aehub::robot
{

OdomWatcher::OdomWatcher(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  if (!node_) {
    throw std::runtime_error("OdomWatcher: node is null");
  }

  odom_subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    rclcpp::QoS(10),
    std::bind(&OdomWatcher::odomCallback, this, std::placeholders::_1));
}

bool OdomWatcher::hasOdom() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return odom_received_;
}

bool OdomWatcher::isStale() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!odom_received_) {
    return true;  // No odom = stale
  }

  constexpr double MAX_AGE_SEC = 1.0;
  const auto now = node_->now();
  const double age = (now - last_odom_time_).seconds();
  return age >= MAX_AGE_SEC;
}

rclcpp::Time OdomWatcher::getLastTime() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_odom_time_;
}

void OdomWatcher::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  (void)msg;
  std::lock_guard<std::mutex> lock(mutex_);
  odom_received_ = true;
  last_odom_time_ = node_->now();
}

}  // namespace aehub::robot

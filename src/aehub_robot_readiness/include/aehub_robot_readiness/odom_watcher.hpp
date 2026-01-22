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
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace aehub::robot
{

/**
 * @brief OdomWatcher - observes /odom topic for presence and freshness
 *
 * Pure observer - only subscribes, never publishes.
 */
class OdomWatcher
{
public:
  explicit OdomWatcher(const rclcpp::Node::SharedPtr & node);

  bool hasOdom() const;
  bool isStale() const;
  rclcpp::Time getLastTime() const;

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  mutable std::mutex mutex_;
  bool odom_received_{false};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace aehub::robot

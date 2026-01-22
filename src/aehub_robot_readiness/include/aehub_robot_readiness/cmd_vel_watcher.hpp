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

#include "rclcpp/rclcpp.hpp"

namespace aehub::robot
{

/**
 * @brief CmdVelWatcher - checks if /cmd_vel has subscribers
 *
 * Pure observer - uses ROS2 graph introspection, no subscriptions.
 */
class CmdVelWatcher
{
public:
  explicit CmdVelWatcher(const rclcpp::Node::SharedPtr & node);

  bool hasConsumer() const;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace aehub::robot

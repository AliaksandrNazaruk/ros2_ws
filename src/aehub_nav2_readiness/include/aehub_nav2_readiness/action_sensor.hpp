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
#include <vector>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "aehub_nav2_readiness/world_snapshot.hpp"

namespace aehub::nav2
{

/**
 * @brief ActionSensor - dumb fact collector for action server states
 *
 * CRITICAL: This is a DUMB sensor - it only collects facts, doesn't make decisions.
 *
 * - Checks action server existence periodically (non-blocking)
 * - Updates WorldSnapshot atomically under mutex
 * - NO conclusions, NO decisions, NO logic
 * - Just: "Action server /navigate_to_pose exists/is_ready at time T"
 *
 * ReadinessGate makes all decisions based on snapshot.
 */
class ActionSensor
{
public:
  ActionSensor(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<WorldSnapshot> snapshot,
    const std::vector<std::string> & watched_actions);

  ActionSensor(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<WorldSnapshot> snapshot,
    const std::vector<std::string> & watched_actions);

  /**
   * @brief Update snapshot with current action server states (non-blocking)
   * Called periodically from executor context (not blocking)
   */
  void updateSnapshot();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
  std::shared_ptr<WorldSnapshot> snapshot_;
  std::vector<std::string> watched_actions_;

  // Action client for /navigate_to_pose (just for checking readiness, not sending goals)
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

  void initializeActionClient();
};

}  // namespace aehub::nav2

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

#include "aehub_nav2_readiness/readiness_check.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <memory>

namespace aehub::nav2 {

/**
 * @brief Check that Nav2 action server /navigate_to_pose is available and ready
 * 
 * SNAPSHOT-BASED: Evaluates snapshot (updated by ActionWatcher)
 * NO blocking calls, NO waiting, NO polling
 * 
 * CRITICAL: Uses snapshot from ActionWatcher to prevent deadlocks.
 * ActionWatcher checks action_server_is_ready() asynchronously (non-blocking).
 * 
 * IMPORTANT: Action server ready ≠ Nav2 fully ready.
 * Additional checks needed: lifecycle nodes active, BT loaded, costmap ready.
 */
class ActionServerCheck : public ReadinessCheck {
public:
  explicit ActionServerCheck(rclcpp::Node::SharedPtr node);
  explicit ActionServerCheck(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  
  explicit ActionServerCheck(
    rclcpp::Node::SharedPtr node,
    const std::string& action_name);

  /**
   * @brief Evaluate action server readiness (snapshot, non-blocking)
   * Evaluates snapshot (updated by ActionWatcher), no blocking calls here
   */
  ReadinessItem evaluate(
    const WorldSnapshot& snapshot,
    const rclcpp::Time& now) override;
  std::string getName() const override { return "ActionServerCheck"; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
  std::string action_name_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;

  void initializeActionClient();
  
  // Helper to get appropriate node for action client
  rclcpp::Node::SharedPtr getNode() const;
};

}  // namespace aehub::nav2

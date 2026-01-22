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

#include "aehub_nav2_readiness/world_snapshot.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <memory>
#include <map>
#include <vector>
#include <string>

namespace aehub::nav2 {

/**
 * @brief LifecycleSensor - dumb fact collector for lifecycle states
 *
 * CRITICAL: This is a DUMB sensor - it only collects facts, doesn't make decisions.
 * 
 * - Subscribes to /transition_event topics
 * - Updates WorldSnapshot atomically under mutex
 * - NO conclusions, NO decisions, NO logic
 * - Just: "I saw node X transition to state Y at time T"
 *
 * ReadinessGate makes all decisions based on snapshot.
 */
class LifecycleSensor {
public:
  LifecycleSensor(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<WorldSnapshot> snapshot,
    const std::vector<std::string>& watched_nodes);

  LifecycleSensor(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<WorldSnapshot> snapshot,
    const std::vector<std::string>& watched_nodes);

  /**
   * @brief Non-blocking snapshot refresh via async GetState requests.
   * Call periodically from a spinning executor context.
   */
  void updateSnapshot();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
  std::shared_ptr<WorldSnapshot> snapshot_;
  std::vector<std::string> watched_nodes_;

  std::vector<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr> transition_subscribers_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> get_state_clients_;

  void onTransitionEvent(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);
  void initializeSubscription();
};

}  // namespace aehub::nav2

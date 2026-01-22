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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <vector>

namespace aehub::nav2 {

/**
 * @brief TFSensor - dumb fact collector for TF transform states
 *
 * CRITICAL: This is a DUMB sensor - it only collects facts, doesn't make decisions.
 * 
 * - Checks TF buffer periodically (non-blocking)
 * - Updates WorldSnapshot atomically under mutex
 * - NO conclusions, NO decisions, NO logic
 * - Just: "Transform parent→child exists/not exists at time T"
 *
 * ReadinessGate makes all decisions based on snapshot.
 */
class TFSensor {
public:
  struct TransformChain {
    std::string parent;
    std::string child;
    std::string key() const { return parent + "->" + child; }
  };

  TFSensor(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<WorldSnapshot> snapshot,
    const std::vector<TransformChain>& watched_transforms);

  TFSensor(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<WorldSnapshot> snapshot,
    const std::vector<TransformChain>& watched_transforms);

  /**
   * @brief Update snapshot with current TF states (non-blocking)
   * Called periodically from executor context (not blocking)
   */
  void updateSnapshot();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
  std::shared_ptr<WorldSnapshot> snapshot_;
  std::vector<TransformChain> watched_transforms_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  void initializeTF();
  rclcpp::Clock::SharedPtr getClock() const;
};

}  // namespace aehub::nav2

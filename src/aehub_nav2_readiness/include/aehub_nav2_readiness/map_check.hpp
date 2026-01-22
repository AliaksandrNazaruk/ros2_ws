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
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <memory>
#include <mutex>
#include <chrono>

namespace aehub::nav2 {

/**
 * @brief Check that map server is active
 * 
 * SNAPSHOT-BASED: Only checks lifecycle state of map_server
 * Does NOT check map topic or map content - that's runtime quality
 */
class MapCheck : public ReadinessCheck {
public:
  explicit MapCheck(rclcpp::Node::SharedPtr node);
  explicit MapCheck(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Check map server readiness (snapshot, non-blocking)
   * Checks:
   * - map_server lifecycle node is ACTIVE
   * 
   * Does NOT check:
   * - Map topic publication
   * - Map content validity
   * - Map staleness
   */
  ReadinessResult check() override;
  std::string getName() const override { return "MapCheck"; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
  std::unique_ptr<LifecycleChecker> lifecycle_checker_;

  void initializeChecker();
};

}  // namespace aehub::nav2

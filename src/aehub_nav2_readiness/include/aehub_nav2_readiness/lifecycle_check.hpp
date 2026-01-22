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
#include "aehub_nav2_readiness/lifecycle_checker.hpp"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <string>

namespace aehub::nav2 {

/**
 * @brief Check that critical Nav2 lifecycle nodes are active
 * 
 * SNAPSHOT-BASED: Evaluates snapshot (updated by LifecycleWatcher via /transition_event)
 * NO blocking service calls, NO retries, NO change_state
 * 
 * CRITICAL: Uses snapshot from LifecycleWatcher (async subscription) to prevent deadlocks.
 * If called from lifecycle callback or executor thread, no blocking operations occur.
 */
class LifecycleCheck : public ReadinessCheck {
public:
  explicit LifecycleCheck(rclcpp::Node::SharedPtr node);
  explicit LifecycleCheck(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Evaluate lifecycle states (snapshot, non-blocking)
   * Evaluates snapshot (updated by LifecycleWatcher), no service calls here
   */
  ReadinessItem evaluate(
    const WorldSnapshot& snapshot,
    const rclcpp::Time& now) override;
  std::string getName() const override { return "LifecycleCheck"; }

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<LifecycleChecker> lifecycle_checker_;
  std::vector<std::string> critical_nodes_;

  void initializeCriticalNodes();
};

}  // namespace aehub::nav2

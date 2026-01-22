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
#include <chrono>
#include <string>

namespace aehub::nav2 {

/**
 * @brief Check that TF transform map → base_link is available and fresh
 * 
 * SNAPSHOT-BASED: Only checks canTransform(), NO waiting, NO lookupTransform()
 * Readiness = graph availability, NOT correctness
 * 
 * IMPORTANT: Uses hysteresis to prevent false NOT_READY after restart
 * - Requires N consecutive successful checks (default: 2-3)
 * - Prevents readiness flaps during TF discovery phase
 */
class TFCheck : public ReadinessCheck {
public:
  explicit TFCheck(rclcpp::Node::SharedPtr node);
  explicit TFCheck(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Check TF transform availability (snapshot, non-blocking)
   * Uses canTransform() with TimePointZero - checks if transform exists in graph
   * Does NOT check transform correctness or recency
   * 
   * Uses hysteresis: requires N consecutive successful checks to be READY
   * Prevents false NOT_READY during TF discovery phase (100-300ms after restart)
   */
  ReadinessItem evaluate(
    const WorldSnapshot& snapshot,
    const rclcpp::Time& now) override;
  std::string getName() const override { return "TFCheck"; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;

  // Hysteresis state (to prevent false NOT_READY after restart)
  mutable std::mutex mutex_;
  int consecutive_successes_{0};
  int consecutive_failures_{0};
  static constexpr int HYSTERESIS_SUCCESS_THRESHOLD = 3;  // Need 3 consecutive successes
  static constexpr int HYSTERESIS_FAILURE_THRESHOLD = 1;  // Fail immediately on failure
  
  // Configuration
  static constexpr std::chrono::milliseconds TF_MAX_AGE_MS{500};  // Max TF age for readiness
  
  rclcpp::Clock::SharedPtr getClock() const;
};

}  // namespace aehub::nav2

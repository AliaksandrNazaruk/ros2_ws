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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <chrono>

namespace aehub::nav2 {

/**
 * @brief Check that localization is available
 * 
 * SNAPSHOT-BASED: Checks if amcl node is active and TF map→odom exists
 * Does NOT check pose quality (covariance, frequency) - that's a separate quality monitor
 */
class LocalizationCheck : public ReadinessCheck {
public:
  explicit LocalizationCheck(rclcpp::Node::SharedPtr node);
  explicit LocalizationCheck(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  /**
   * @brief Check localization readiness (snapshot, non-blocking)
   * Checks:
   * - AMCL lifecycle node is ACTIVE
   * - TF transform map → odom exists (via TF buffer)
   * 
   * Does NOT check:
   * - Pose covariance quality
   * - Update frequency
   * - Pose correctness
   */
  ReadinessResult check() override;
  std::string getName() const override { return "LocalizationCheck"; }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;
  std::unique_ptr<LifecycleChecker> lifecycle_checker_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Clock::SharedPtr getClock() const;
  
  void initializeCheckers();
};

}  // namespace aehub::nav2

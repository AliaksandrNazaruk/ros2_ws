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

#include <chrono>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "aehub_nav2_readiness/readiness_gate.hpp"
#include "aehub_nav2_readiness/readiness_result.hpp"

#include "aehub_robot_readiness/odom_watcher.hpp"
#include "aehub_robot_readiness/cmd_vel_watcher.hpp"
#include "aehub_robot_readiness/safety_watcher.hpp"

namespace aehub::robot
{

using aehub::nav2::ReadinessGate;
using aehub::nav2::ReadinessLevel;
using aehub::nav2::ReadinessReport;
using aehub::nav2::ReadinessResult;

/**
 * @brief RobotReadinessGate
 *
 * Determines whether the PHYSICAL ROBOT is ready to accept motion commands.
 *
 * This gate is:
 *  - Pure observer
 *  - Deterministic
 *  - Non-blocking in check()
 *
 * Checked invariants:
 *  - /odom exists and is fresh
 *  - at least one cmd_vel consumer exists
 *  - motors enabled
 *  - emergency stop inactive
 */
class RobotReadinessGate : public ReadinessGate
{
public:
  explicit RobotReadinessGate(const rclcpp::Node::SharedPtr & node);

  ReadinessReport check() override;
  ReadinessResult current() const override;
  bool changed() const override;

  /**
   * @brief Blocking wait (ONLY for non-lifecycle use)
   *
   * ⚠️ Must NOT be called from lifecycle callbacks.
   */
  bool waitUntilReady(std::chrono::milliseconds timeout) override;

private:
  bool checkOdom(ReadinessResult & out);
  bool checkCmdVel(ReadinessResult & out);
  bool checkSafety(ReadinessResult & out);

  rclcpp::Node::SharedPtr node_;

  std::unique_ptr<OdomWatcher> odom_watcher_;
  std::unique_ptr<CmdVelWatcher> cmd_vel_watcher_;
  std::unique_ptr<SafetyWatcher> safety_watcher_;

  // Cached state
  mutable std::mutex mutex_;
  ReadinessResult cached_result_;
  ReadinessLevel last_reported_level_{ReadinessLevel::NOT_READY};
  rclcpp::Time last_check_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace aehub::robot

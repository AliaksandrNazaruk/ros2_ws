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

#include "aehub_robot_readiness/robot_readiness_gate.hpp"

#include <thread>
#include <sstream>

using namespace std::chrono_literals;

using aehub::nav2::FailureClass;
using aehub::nav2::FailureInfo;
using aehub::nav2::FailureSeverity;
using aehub::nav2::ReadinessCapability;
using aehub::nav2::ReadinessFailure;
using aehub::nav2::ReadinessState;

namespace
{
constexpr std::chrono::milliseconds CHECK_INTERVAL{100};
}

namespace aehub::robot
{

RobotReadinessGate::RobotReadinessGate(
  const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  if (!node_) {
    throw std::runtime_error("RobotReadinessGate: node is null");
  }

  odom_watcher_ = std::make_unique<OdomWatcher>(node_);
  cmd_vel_watcher_ = std::make_unique<CmdVelWatcher>(node_);
  safety_watcher_ = std::make_unique<SafetyWatcher>(node_);

  RCLCPP_INFO(node_->get_logger(), "RobotReadinessGate initialized");
}

ReadinessReport RobotReadinessGate::check()
{
  ReadinessResult result;
  result.state = ReadinessState::NOT_READY;

  ReadinessReport report;
  report.snapshot_time = node_->now();
  report.overall_level = ReadinessLevel::NOT_READY;

  // --- Safety data source gate (RobotStatus mirror) ---
  if (!safety_watcher_->hasFreshStatus()) {
    result.reason = "RobotStatus missing or stale";
    result.failures.push_back(ReadinessFailure::ROBOT_STATUS_MISSING);
    report.failures.push_back({
        ReadinessFailure::ROBOT_STATUS_MISSING,
        FailureSeverity::HARD,
        FailureClass::TRANSIENT,
        result.reason,
        "robot_status"
    });
  } else if (!checkOdom(result)) {
    // reason set by checkOdom
    report.failures.push_back({
        result.failures.empty() ? ReadinessFailure::ODOM_MISSING : result.failures.front(),
        FailureSeverity::HARD,
        FailureClass::RECOVERABLE,
        result.reason,
        "odom"
    });
  } else if (!checkCmdVel(result)) {
    report.failures.push_back({
        ReadinessFailure::CMD_VEL_NO_CONSUMERS,
        FailureSeverity::HARD,
        FailureClass::RECOVERABLE,
        result.reason,
        "cmd_vel"
    });
  } else if (!checkSafety(result)) {
    // checkSafety sets correct reason; classify as FATAL if estop, RECOVERABLE if motors disabled
    const bool estop = safety_watcher_->emergencyStopActive();
    report.failures.push_back({
        estop ? ReadinessFailure::ESTOP_ACTIVE : ReadinessFailure::MOTORS_DISABLED,
        FailureSeverity::HARD,
        estop ? FailureClass::FATAL : FailureClass::RECOVERABLE,
        result.reason,
        "safety"
    });
  } else {
    result.state = ReadinessState::READY;
    result.reason = "Robot is ready for motion";
    report.overall_level = ReadinessLevel::READY;
  }

  report.summary = result.reason;
  report.capabilities[ReadinessCapability::MOTION_READY] =
    (report.overall_level == ReadinessLevel::READY);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    cached_result_ = result;
    last_reported_level_ = report.overall_level;
    last_check_time_ = report.snapshot_time;
  }

  return report;
}

bool RobotReadinessGate::checkOdom(ReadinessResult & out)
{
  if (!odom_watcher_->hasOdom()) {
    out.reason = "Odometry not received";
    out.missing.push_back("/odom");
    out.failures.push_back(ReadinessFailure::ODOM_MISSING);
    return false;
  }

  if (odom_watcher_->isStale()) {
    out.reason = "Odometry is stale";
    out.missing.push_back("/odom (stale)");
    out.failures.push_back(ReadinessFailure::ODOM_STALE);
    return false;
  }

  return true;
}

bool RobotReadinessGate::checkCmdVel(ReadinessResult & out)
{
  if (!cmd_vel_watcher_->hasConsumer()) {
    out.reason = "No cmd_vel consumers detected";
    out.missing.push_back("/cmd_vel");
    out.failures.push_back(ReadinessFailure::CMD_VEL_NO_CONSUMERS);
    return false;
  }

  return true;
}

bool RobotReadinessGate::checkSafety(ReadinessResult & out)
{
  if (!safety_watcher_->motorsEnabled()) {
    out.reason = "Motors are disabled";
    out.missing.push_back("motors");
    out.failures.push_back(ReadinessFailure::MOTORS_DISABLED);
    return false;
  }

  if (safety_watcher_->emergencyStopActive()) {
    out.reason = "Emergency stop active";
    out.missing.push_back("estop");
    out.failures.push_back(ReadinessFailure::ESTOP_ACTIVE);
    return false;
  }

  return true;
}

ReadinessResult RobotReadinessGate::current() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_result_;
}

bool RobotReadinessGate::changed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  ReadinessLevel current_level = cached_result_.isReady() ?
    ReadinessLevel::READY : ReadinessLevel::NOT_READY;
  return current_level != last_reported_level_;
}

bool RobotReadinessGate::waitUntilReady(std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    if (std::chrono::steady_clock::now() - start >= timeout) {
      RCLCPP_WARN(node_->get_logger(),
        "RobotReadinessGate timeout after %ld ms", timeout.count());
      return false;
    }

    if (check().isReady()) {
      RCLCPP_INFO(node_->get_logger(), "Robot is ready");
      return true;
    }

    std::this_thread::sleep_for(CHECK_INTERVAL);
  }

  return false;
}

}  // namespace aehub::robot

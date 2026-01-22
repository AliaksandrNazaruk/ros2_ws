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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <aehub_nav2_readiness/readiness_gate.hpp>
#include <aehub_nav2_readiness/readiness_result.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>
#include <map>
#include <vector>

namespace aehub_nav2_adapter::test
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

/**
 * @brief Fake Nav2 action server for testing
 */
class FakeNav2ActionServer : public rclcpp::Node
{
public:
  explicit FakeNav2ActionServer(const std::string& node_name = "fake_nav2_server");
  ~FakeNav2ActionServer() override;

  // Behavior control
  void setShouldAcceptGoals(bool accept) { should_accept_goals_ = accept; }
  void setShouldSucceedGoals(bool succeed) { should_succeed_goals_ = succeed; }
  void setGoalDelaySec(double delay) { goal_delay_sec_ = delay; }
  void rejectNextGoal() { should_accept_goals_ = false; }
  void acceptNextGoal() { should_accept_goals_ = true; }

  // Goal management
  std::vector<std::string> getActiveGoals() const;
  void cancelGoal(const std::string& goal_id = "");

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
  mutable std::mutex mutex_;
  std::map<std::string, std::shared_ptr<GoalHandle>> goal_handles_;
  std::vector<std::thread> goal_threads_;

  bool should_accept_goals_{true};
  bool should_succeed_goals_{true};
  double goal_delay_sec_{0.1};

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void executeGoal(const std::shared_ptr<GoalHandle> goal_handle);
};

/**
 * @brief Event collector for testing
 */
class EventCollector
{
public:
  struct Event
  {
    std::string type;  // "accepted", "succeeded", "failed", "canceled"
    std::string command_id;
    std::string error;  // For failed events
    std::chrono::steady_clock::time_point timestamp;
  };

  void onAccepted(const std::string& command_id);
  void onSucceeded(const std::string& command_id);
  void onFailed(const std::string& command_id, const std::string& error);
  void onCanceled(const std::string& command_id);

  std::vector<Event> getEvents() const;
  void clear();

  bool hasEvent(const std::string& type, const std::string& command_id) const;
  bool waitForEvent(const std::string& type, const std::string& command_id,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) const;

private:
  mutable std::mutex mutex_;
  std::vector<Event> events_;
};

/**
 * @brief Mock ReadinessGate that always returns READY
 * 
 * Used in tests to isolate FSM behavior from Nav2 readiness checks.
 * This allows testing Nav2Adapter FSM without requiring full Nav2 stack.
 */
class AlwaysReadyGate : public aehub::nav2::ReadinessGate
{
public:
  AlwaysReadyGate() = default;
  ~AlwaysReadyGate() override = default;

  aehub::nav2::ReadinessReport check() override
  {
    aehub::nav2::ReadinessReport report;
    report.overall_level = aehub::nav2::ReadinessLevel::READY;
    report.summary = "Mock: Always ready (test mode)";
    report.capabilities[aehub::nav2::ReadinessCapability::TRANSPORT_READY] = true;
    report.capabilities[aehub::nav2::ReadinessCapability::NAV2_READY] = true;
    report.capabilities[aehub::nav2::ReadinessCapability::MOTION_READY] = true;
    return report;
  }

  bool changed() const override
  {
    return false;  // Never changes (always ready)
  }

  aehub::nav2::ReadinessResult current() const override
  {
    aehub::nav2::ReadinessResult result;
    result.state = aehub::nav2::ReadinessState::READY;
    result.reason = "Mock: Always ready (test mode)";
    return result;
  }

  bool waitUntilReady(std::chrono::milliseconds /* timeout */) override
  {
    return true;  // Always ready immediately
  }
};

}  // namespace aehub_nav2_adapter::test

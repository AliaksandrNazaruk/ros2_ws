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

#include "test_helpers.hpp"

#include <chrono>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/types.hpp>

namespace aehub_nav2_adapter::test
{

using namespace std::chrono_literals;

FakeNav2ActionServer::FakeNav2ActionServer(const std::string& node_name)
: Node(node_name)
{
  action_server_ = rclcpp_action::create_server<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "navigate_to_pose",
    [this](const rclcpp_action::GoalUUID& uuid,
           std::shared_ptr<const NavigateToPose::Goal> goal) {
      return this->handleGoal(uuid, goal);
    },
    [this](const std::shared_ptr<GoalHandle> goal_handle) {
      return this->handleCancel(goal_handle);
    },
    [this](const std::shared_ptr<GoalHandle> goal_handle) {
      // Execute in background thread to avoid blocking executor
      // (We keep threads joinable to avoid teardown races in test suite.)
      goal_threads_.emplace_back([this, goal_handle]() { this->executeGoal(goal_handle); });
    });

  RCLCPP_INFO(this->get_logger(), "FakeNav2ActionServer started");
}

FakeNav2ActionServer::~FakeNav2ActionServer()
{
  for (auto& t : goal_threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
}

rclcpp_action::GoalResponse FakeNav2ActionServer::handleGoal(
  const rclcpp_action::GoalUUID& /*uuid*/,
  std::shared_ptr<const NavigateToPose::Goal> /*goal*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!should_accept_goals_) {
    RCLCPP_INFO(this->get_logger(), "Rejecting goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Accepting goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FakeNav2ActionServer::handleCancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  RCLCPP_INFO(this->get_logger(), "Cancel requested for goal: %s", goal_id.c_str());

  // IMPORTANT: Do not call goal_handle->canceled() here.
  // Correct action semantics: accept cancel request, then the execute loop should
  // observe goal_handle->is_canceling() and complete with CANCELED.
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FakeNav2ActionServer::executeGoal(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::string goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());

  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handles_[goal_id] = goal_handle;
  }

  RCLCPP_INFO(this->get_logger(), "Goal executing: %s", goal_id.c_str());

  // Simulate navigation delay in this background thread.
  std::this_thread::sleep_for(std::chrono::duration<double>(goal_delay_sec_));

  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handles_.find(goal_id) == goal_handles_.end()) {
    return;  // Goal was already cleared/cancelled
  }

  try {
    auto result = std::make_shared<NavigateToPose::Result>();
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled: %s", goal_id.c_str());
      goal_handles_.erase(goal_id);
      return;
    }
    if (should_succeed_goals_) {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded: %s", goal_id.c_str());
    } else {
      result->error_code = 1;  // Generic error code for test
      goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), "Goal aborted: %s", goal_id.c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Exception completing goal: %s", e.what());
  }

  goal_handles_.erase(goal_id);
}

std::vector<std::string> FakeNav2ActionServer::getActiveGoals() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> goals;
  for (const auto& [goal_id, handle] : goal_handles_) {
    goals.push_back(goal_id);
  }
  return goals;
}

void FakeNav2ActionServer::cancelGoal(const std::string& goal_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_id.empty()) {
    // Simulate abrupt server shutdown: clear active goals without proper action terminal state.
    goal_handles_.clear();
  } else {
    auto it = goal_handles_.find(goal_id);
    if (it != goal_handles_.end()) {
      goal_handles_.erase(it);
    }
  }
}

void EventCollector::onAccepted(const std::string& command_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Event e;
  e.type = "accepted";
  e.command_id = command_id;
  e.timestamp = std::chrono::steady_clock::now();
  events_.push_back(e);
}

void EventCollector::onSucceeded(const std::string& command_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Event e;
  e.type = "succeeded";
  e.command_id = command_id;
  e.timestamp = std::chrono::steady_clock::now();
  events_.push_back(e);
}

void EventCollector::onFailed(const std::string& command_id, const std::string& error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Event e;
  e.type = "failed";
  e.command_id = command_id;
  e.error = error;
  e.timestamp = std::chrono::steady_clock::now();
  events_.push_back(e);
}

void EventCollector::onCanceled(const std::string& command_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Event e;
  e.type = "canceled";
  e.command_id = command_id;
  e.timestamp = std::chrono::steady_clock::now();
  events_.push_back(e);
}

std::vector<EventCollector::Event> EventCollector::getEvents() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return events_;
}

void EventCollector::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  events_.clear();
}

bool EventCollector::hasEvent(const std::string& type, const std::string& command_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& e : events_) {
    if (e.type == type && e.command_id == command_id) {
      return true;
    }
  }
  return false;
}

bool EventCollector::waitForEvent(const std::string& type, const std::string& command_id,
  std::chrono::milliseconds timeout) const
{
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < timeout) {
    if (hasEvent(type, command_id)) {
      return true;
    }
    // Check every 50ms to be responsive
    std::this_thread::sleep_for(50ms);
  }
  return false;
}

}  // namespace aehub_nav2_adapter::test

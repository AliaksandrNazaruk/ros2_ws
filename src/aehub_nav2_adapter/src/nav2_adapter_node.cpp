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

#include "aehub_nav2_adapter/nav2_adapter_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include <action_msgs/msg/goal_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executors.hpp>

#include <aehub_nav2_readiness/nav2_readiness_gate.hpp>

namespace aehub_nav2_adapter
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

// =============================================================
// Constructor
// =============================================================

Nav2AdapterNode::Nav2AdapterNode(
  const rclcpp::NodeOptions & options,
  std::shared_ptr<aehub::nav2::ReadinessGate> readiness_gate)
: LifecycleNode("nav2_adapter", options),
  internal_state_(AdapterState::UNCONFIGURED),
  cancel_requested_(false),
  // IMPORTANT: keep relative name to support namespace /robot/<id>/...
  action_name_("navigate_to_pose"),
  server_wait_timeout_sec_(5.0),
  goal_response_timeout_sec_(10.0),
  result_timeout_sec_(300.0),
  readiness_timeout_sec_(5.0),
  nav2_ready_(false),
  server_was_ready_(false),
  readiness_gate_(readiness_gate)
{
  declare_parameter("action_name", action_name_);
  declare_parameter("readiness_timeout_sec", readiness_timeout_sec_);
  declare_parameter("server_wait_timeout_sec", server_wait_timeout_sec_);

  get_parameter("action_name", action_name_);
  get_parameter("readiness_timeout_sec", readiness_timeout_sec_);
  get_parameter("server_wait_timeout_sec", server_wait_timeout_sec_);

  // Backward compatible: if user provided absolute "/navigate_to_pose", normalize to relative.
  if (!action_name_.empty() && action_name_[0] == '/') {
    action_name_.erase(0, 1);
  }

  RCLCPP_INFO(get_logger(), "Nav2AdapterNode created");
}

// =============================================================
// Lifecycle API (public wrapper)
// =============================================================

bool Nav2AdapterNode::configure()
{
  return LifecycleNode::configure().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

bool Nav2AdapterNode::activate()
{
  return LifecycleNode::activate().id() ==
         lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

void Nav2AdapterNode::deactivate()
{
  LifecycleNode::deactivate();
}

void Nav2AdapterNode::cleanup()
{
  LifecycleNode::cleanup();
}

// =============================================================
// Lifecycle callbacks
// =============================================================

CallbackReturn Nav2AdapterNode::on_configure(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (internal_state_ != AdapterState::UNCONFIGURED) {
    RCLCPP_ERROR(get_logger(), "Invalid state for configure()");
    return CallbackReturn::FAILURE;
  }

  action_client_ =
    rclcpp_action::create_client<NavigateToPose>(
      shared_from_this(), action_name_);

  // Create readiness gate if not injected (dependency injection for tests)
  if (!readiness_gate_) {
    readiness_gate_ = std::make_shared<aehub::nav2::Nav2ReadinessGate>(
      shared_from_this());
  }

  resetState();
  setState(AdapterState::INACTIVE);

  RCLCPP_INFO(get_logger(), "Nav2Adapter configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Nav2AdapterNode::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (internal_state_ != AdapterState::INACTIVE) {
    RCLCPP_ERROR(get_logger(), "Invalid state for activate()");
    return CallbackReturn::FAILURE;
  }

  nav2_ready_ = false;
  server_was_ready_ = false;

  // 🔁 Asynchronous readiness probe
  nav2_probe_timer_ = create_wall_timer(
    100ms,
    std::bind(&Nav2AdapterNode::probeNav2Readiness, this));

  setState(AdapterState::IDLE);

  RCLCPP_INFO(get_logger(), "Nav2Adapter activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Nav2AdapterNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (nav2_probe_timer_) {
    nav2_probe_timer_->cancel();
    nav2_probe_timer_.reset();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Cancel active goal if any (best-effort, non-blocking)
    if (active_goal_handle_) {
      cancel_requested_ = true;
      // Async cancel without waiting
      if (action_client_) {
        action_client_->async_cancel_goal(active_goal_handle_);
      }
    }

    resetState();
    nav2_ready_ = false;
    setState(AdapterState::INACTIVE);
  }

  RCLCPP_INFO(get_logger(), "Nav2Adapter deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Nav2AdapterNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mutex_);

  action_client_.reset();
  readiness_gate_.reset();
  nav2_probe_timer_.reset();

  resetState();
  setState(AdapterState::UNCONFIGURED);

  RCLCPP_INFO(get_logger(), "Nav2Adapter cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn Nav2AdapterNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  action_client_.reset();
  readiness_gate_.reset();
  resetState();
  return CallbackReturn::SUCCESS;
}

// =============================================================
// Navigation command
// =============================================================

bool Nav2AdapterNode::navigateToPose(
  const std::string & command_id,
  const geometry_msgs::msg::PoseStamped & target)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (internal_state_ != AdapterState::IDLE) {
    RCLCPP_ERROR(get_logger(), "navigateToPose(): adapter not IDLE");
    return false;
  }

  if (!action_client_) {
    RCLCPP_ERROR(get_logger(), "Action client not initialized");
    setState(AdapterState::ERROR);
    if (events_.onFailed) {
      events_.onFailed(command_id, "nav2_unavailable");
    }
    return false;
  }

  // Avoid sending goals before action discovery completes (DDS graph propagation).
  // This is bounded waiting (NOT in lifecycle callbacks) to prevent rclcpp_action
  // "unknown goal/ result response" issues in practice.
  if (!action_client_->action_server_is_ready()) {
    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(server_wait_timeout_sec_));
    (void)action_client_->wait_for_action_server(timeout);
  }
  if (!action_client_->action_server_is_ready()) {
    setState(AdapterState::ERROR);
    if (events_.onFailed) {
      events_.onFailed(command_id, "nav2_unavailable");
    }
    return false;
  }

  if (!readiness_gate_) {
    RCLCPP_ERROR(get_logger(), "ReadinessGate not initialized");
    return false;
  }

  // G-INV-04: Readiness Gate Authority - check readiness before sending goal
  // Use check() instead of waitUntilReady() to avoid blocking (G-INV-03)
  auto report = readiness_gate_->check();
  if (!report.isReady()) {
    RCLCPP_ERROR(
      get_logger(),
      "Nav2 not ready: %s",
      report.summary.c_str());

    setState(AdapterState::ERROR);

    if (events_.onFailed) {
      // SRS: do not leak internal dependency details as "interface".
      // Provide a stable error code; details remain available via health/readiness.
      events_.onFailed(command_id, "nav2_unavailable");
    }
    return false;
  }

  auto goal = NavigateToPose::Goal();
  goal.pose = target;

  active_command_id_ = command_id;

  auto options =
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  options.goal_response_callback =
    [this, command_id](auto gh) {
      goalResponseCallback(gh, command_id);
    };

  options.result_callback =
    [this, command_id](auto res) {
      resultCallback(res, command_id);
    };

  // IMPORTANT: Keep the returned future alive.
  // If it is destroyed immediately, rclcpp_action may log:
  // "unknown goal/ cancel/ result response, ignoring..."
  // and callbacks may not fire (DDS response can't be matched).
  send_goal_future_ = action_client_->async_send_goal(goal, options);

  RCLCPP_INFO(get_logger(), "Goal sent (%s)", command_id.c_str());
  return true;
}

bool Nav2AdapterNode::cancelActiveGoal(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Idempotent: if no active goal, return success
  if (!active_goal_handle_) {
    return true;
  }

  // Idempotent: if already canceling, return success
  if (cancel_requested_ || internal_state_ == AdapterState::CANCELING) {
    return true;
  }

  // Can only cancel from NAVIGATING state
  if (internal_state_ != AdapterState::NAVIGATING) {
    RCLCPP_WARN(
      get_logger(),
      "cancelActiveGoal(): not in NAVIGATING state (current: %d)",
      static_cast<int>(internal_state_));
    return false;
  }

  cancel_requested_ = true;
  setState(AdapterState::CANCELING);

  // Async cancel (non-blocking)
  cancel_goal_future_ = action_client_->async_cancel_goal(active_goal_handle_);

  RCLCPP_INFO(
    get_logger(),
    "Cancel requested for goal (%s), reason: %s",
    active_command_id_.c_str(),
    reason.c_str());

  return true;
}

bool Nav2AdapterNode::hasActiveGoal() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_goal_handle_ != nullptr;
}

// =============================================================
// Async readiness probe
// =============================================================

void Nav2AdapterNode::probeNav2Readiness()
{
  if (!readiness_gate_) {
    return;
  }

  auto report = readiness_gate_->check();
  const bool ready = report.isReady();

  std::lock_guard<std::mutex> lock(mutex_);

  if (ready && !nav2_ready_) {
    nav2_ready_ = true;
    RCLCPP_INFO(get_logger(), "Nav2 became READY");
  }

  if (!ready && nav2_ready_) {
    nav2_ready_ = false;
    RCLCPP_ERROR(get_logger(), "Nav2 became NOT READY");
    handleNav2Restart();
  }
}

// =============================================================
// Goal callbacks
// =============================================================

void Nav2AdapterNode::goalResponseCallback(
  const GoalHandleNav::SharedPtr & goal_handle,
  const std::string & command_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // G-INV-02: Command ID Integrity - ignore callbacks for mismatched command_id
  if (command_id != active_command_id_) {
    RCLCPP_WARN(
      get_logger(),
      "goalResponseCallback: command_id mismatch (expected: %s, got: %s), ignoring",
      active_command_id_.c_str(),
      command_id.c_str());
    return;
  }

  if (!goal_handle) {
    if (events_.onFailed) {
      events_.onFailed(command_id, "goal_rejected");
    }
    resetState();
    setState(AdapterState::IDLE);
    return;
  }

  active_goal_handle_ = goal_handle;
  setState(AdapterState::NAVIGATING);

  if (events_.onAccepted) {
    events_.onAccepted(command_id);
  }
}

void Nav2AdapterNode::resultCallback(
  const GoalHandleNav::WrappedResult & result,
  const std::string & command_id)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // G-INV-02: Command ID Integrity - ignore callbacks for mismatched command_id
  if (command_id != active_command_id_) {
    RCLCPP_WARN(
      get_logger(),
      "resultCallback: command_id mismatch (expected: %s, got: %s), ignoring",
      active_command_id_.c_str(),
      command_id.c_str());
    return;
  }

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (events_.onSucceeded) {
        events_.onSucceeded(command_id);
      }
      break;

    case rclcpp_action::ResultCode::CANCELED:
      if (events_.onCanceled) {
        events_.onCanceled(command_id);
      }
      break;

    default:
      if (events_.onFailed) {
        events_.onFailed(command_id, "navigation_failed");
      }
      break;
  }

  resetState();
  // Transition to IDLE (from either NAVIGATING or CANCELING)
  setState(AdapterState::IDLE);
}

// =============================================================
// Internal helpers
// =============================================================

void Nav2AdapterNode::handleNav2Restart()
{
  if (!active_command_id_.empty() && events_.onFailed) {
    events_.onFailed(active_command_id_, "nav2_restarted");
  }

  resetState();
  setState(AdapterState::ERROR);
}

void Nav2AdapterNode::resetState()
{
  active_command_id_.clear();
  active_goal_handle_.reset();
  send_goal_future_ = SendGoalFuture{};
  cancel_goal_future_ = CancelGoalFuture{};
  cancel_requested_ = false;
  // Note: internal_state_ is set explicitly via setState() after resetState()
}

void Nav2AdapterNode::setState(AdapterState s)
{
  internal_state_ = s;
}

void Nav2AdapterNode::setEvents(Nav2Events events)
{
  events_ = std::move(events);
}

}  // namespace aehub_nav2_adapter

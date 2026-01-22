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
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <chrono>
#include <future>
#include <utility>

#include "aehub_nav2_adapter/nav2_adapter.hpp"
#include "aehub_nav2_adapter/nav2_events.hpp"
#include "aehub_nav2_readiness/nav2_readiness_gate.hpp"
#include "aehub_nav2_readiness/readiness_gate.hpp"

namespace aehub_nav2_adapter
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

/**
 * @brief Explicit adapter state model
 *
 * States track the lifecycle and execution status of the adapter.
 * This ensures deterministic behavior and allows executor to make
 * informed decisions without guessing.
 */
enum class AdapterState
{
  UNCONFIGURED,  ///< Not yet configured (before on_configure)
  INACTIVE,      ///< Configured but not active (after on_configure, before on_activate)
  IDLE,          ///< Active and ready to accept commands
  NAVIGATING,    ///< Navigation goal is active
  CANCELING,     ///< Cancel request in progress
  ERROR          ///< Error state (Nav2 unavailable, unexpected failures)
};

/**
 * @brief Nav2AdapterNode - LifecycleNode implementation of Nav2Adapter
 *
 * This class implements the Nav2Adapter interface as a ROS2 LifecycleNode,
 * providing a capability layer adapter between the application layer and Nav2.
 */
class Nav2AdapterNode final : public rclcpp_lifecycle::LifecycleNode, public Nav2Adapter
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   * @param readiness_gate Optional readiness gate (for dependency injection in tests)
   *                        If nullptr, creates Nav2ReadinessGate internally
   */
  explicit Nav2AdapterNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
    std::shared_ptr<aehub::nav2::ReadinessGate> readiness_gate = nullptr);

  /**
   * @brief Destructor
   */
  ~Nav2AdapterNode() override = default;

  // Lifecycle (Nav2Adapter interface)
  bool configure() override;
  bool activate() override;
  void deactivate() override;
  void cleanup() override;

  // LifecycleNode callbacks
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  // Commands (Nav2Adapter interface)
  bool navigateToPose(
    const std::string& command_id,
    const geometry_msgs::msg::PoseStamped& target) override;

  bool cancelActiveGoal(
    const std::string& reason) override;

  // State (Nav2Adapter interface)
  bool hasActiveGoal() const override;

  /**
   * @brief Set event callbacks
   * @param events Event callback struct (moved)
   */
  void setEvents(Nav2Events events);

private:
  using SendGoalFuture = std::shared_future<GoalHandleNav::SharedPtr>;
  using CancelGoalFuture = decltype(
    std::declval<rclcpp_action::Client<NavigateToPose>&>().async_cancel_goal(
      std::declval<GoalHandleNav::SharedPtr>()));

  // Action client
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

  // Event callbacks
  Nav2Events events_;

  // State tracking (protected by mutex_)
  mutable std::mutex mutex_;
  std::string active_command_id_;
  GoalHandleNav::SharedPtr active_goal_handle_;
  SendGoalFuture send_goal_future_;
  CancelGoalFuture cancel_goal_future_;
  AdapterState internal_state_;
  bool cancel_requested_;

  // Configuration
  std::string action_name_;
  double server_wait_timeout_sec_;
  double goal_response_timeout_sec_;
  double result_timeout_sec_;
  double readiness_timeout_sec_;

  // Nav2 readiness gate (separate responsibility for readiness checking)
  // Can be injected for testing (dependency injection)
  std::shared_ptr<aehub::nav2::ReadinessGate> readiness_gate_;

  // Nav2 readiness tracking (async probe, no blocking)
  // NOTE: This is kept for backward compatibility and restart detection
  // The readiness gate is the authoritative source for readiness
  bool nav2_ready_;
  rclcpp::TimerBase::SharedPtr nav2_probe_timer_;

  // Nav2 restart detection (used by checkServerHealth and probeNav2Readiness)
  bool server_was_ready_;

  // Action client callbacks
  void goalResponseCallback(
    const GoalHandleNav::SharedPtr& goal_handle,
    const std::string& command_id);

  void resultCallback(
    const GoalHandleNav::WrappedResult& result,
    const std::string& command_id);

  // Nav2 readiness probe (async, non-blocking)
  void probeNav2Readiness();

  // Nav2 restart detection (called during operations)
  void checkServerHealth();

  // Helper methods
  void resetState();
  void setState(AdapterState new_state);
  void handleNav2Restart();

  // State invariant checks (production-safe)
  bool checkStateInvariant(AdapterState expected_state, const std::string& operation);
};

}  // namespace aehub_nav2_adapter

// Copyright 2026 Boris
//
// Licensed under the Apache License, Version 2.0

#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <aehub_msgs/action/navigation_execute.hpp>

#include <aehub_nav2_readiness/readiness_gate.hpp>

#include <aehub_nav2_adapter/nav2_adapter_node.hpp>
#include <aehub_nav2_adapter/nav2_events.hpp>

namespace aehub_nav2_capability_server
{

using NavigationExecute = aehub_msgs::action::NavigationExecute;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigationExecute>;

class Nav2CapabilityServerNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Nav2CapabilityServerNode(
    std::shared_ptr<aehub_nav2_adapter::Nav2AdapterNode> adapter,
    std::shared_ptr<aehub::nav2::ReadinessGate> readiness_gate)
  : rclcpp_lifecycle::LifecycleNode("aehub_nav2_capability_server"),
    adapter_(std::move(adapter)),
    readiness_gate_(std::move(readiness_gate))
  {
    // -------- Parameters (safety & conventions) --------
    // Default is relative to support namespace robot/<id>/...
    // IMPORTANT: this process also hosts nav2_adapter, which uses parameter name "action_name"
    // for the Nav2 NavigateToPose action (default: "navigate_to_pose").
    // To avoid parameter collisions across nodes in the same process, use a distinct parameter
    // name for the capability action API.
    declare_parameter("capability_action_name", std::string("capabilities/navigation/execute"));
    // Backward compatibility (deprecated): older launch files used "action_name".
    declare_parameter("action_name", std::string(""));
    declare_parameter("goal_frame", std::string("map"));
    declare_parameter("cmd_vel_topic", std::string("cmd_vel"));
    declare_parameter("stop_burst_enabled", true);
    declare_parameter("stop_burst_duration_s", 0.6);
    declare_parameter("stop_burst_rate_hz", 20.0);
    declare_parameter("health_topic", std::string("health/aehub_nav2_capability_server"));
    declare_parameter("health_publish_period_s", 1.0);

    RCLCPP_INFO(get_logger(), "Nav2CapabilityServerNode constructed (LifecycleNode)");
  }

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    // Prefer new parameter name to avoid collisions. Fallback to legacy.
    action_name_ = get_parameter("capability_action_name").as_string();
    if (action_name_.empty()) {
      action_name_ = get_parameter("action_name").as_string();
    }
    goal_frame_ = get_parameter("goal_frame").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    stop_burst_enabled_ = get_parameter("stop_burst_enabled").as_bool();
    stop_burst_duration_s_ = get_parameter("stop_burst_duration_s").as_double();
    stop_burst_rate_hz_ = get_parameter("stop_burst_rate_hz").as_double();
    health_topic_ = get_parameter("health_topic").as_string();
    health_publish_period_s_ = get_parameter("health_publish_period_s").as_double();

    accepting_goals_ = false;

    // -------- Ownership invariant (documentation + runtime intent) --------
    RCLCPP_INFO(
      get_logger(),
      "NavigationCapability configured. Ownership: Nav2 lifecycle + readiness + STOP + cmd_vel (topic=%s).",
      cmd_vel_topic_.c_str());

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));

    health_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      health_topic_, rclcpp::QoS(1).transient_local());

    // -------- Adapter events wiring --------
    aehub_nav2_adapter::Nav2Events events;
    events.onAccepted = [this](const std::string & command_id) { this->onAdapterAccepted(command_id); };
    events.onSucceeded = [this](const std::string & command_id) { this->onAdapterSucceeded(command_id); };
    events.onCanceled = [this](const std::string & command_id) { this->onAdapterCanceled(command_id); };
    events.onFailed =
      [this](const std::string & command_id, const std::string & error) { this->onAdapterFailed(command_id, error); };
    adapter_->setEvents(std::move(events));

    // -------- Action server --------
    action_server_ = rclcpp_action::create_server<NavigationExecute>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      action_name_,
      std::bind(&Nav2CapabilityServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Nav2CapabilityServerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&Nav2CapabilityServerNode::handle_accepted, this, std::placeholders::_1));

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // Bring adapter online (owned by capability).
    (void)adapter_->configure();
    (void)adapter_->activate();
    accepting_goals_ = true;
    if (health_publish_period_s_ > 0.0) {
      const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(health_publish_period_s_));
      health_timer_ = create_wall_timer(period_ns, [this]() { this->publishHealth(); });
    }
    publishHealth();
    RCLCPP_INFO(get_logger(), "NavigationCapability activated (accepting action goals)");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    accepting_goals_ = false;

    // Best-effort: cancel active goal and STOP (policy).
    (void)adapter_->cancelActiveGoal("capability_deactivate");
    if (stop_burst_enabled_) {
      startStopBurst();
    }

    adapter_->deactivate();
    if (health_timer_) {
      health_timer_->cancel();
      health_timer_.reset();
    }
    publishHealth();
    RCLCPP_INFO(get_logger(), "NavigationCapability deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    accepting_goals_ = false;

    if (stop_timer_) {
      stop_timer_->cancel();
      stop_timer_.reset();
    }

    adapter_->cleanup();

    action_server_.reset();
    cmd_vel_pub_.reset();
    if (health_timer_) {
      health_timer_->cancel();
      health_timer_.reset();
    }
    health_pub_.reset();

    RCLCPP_INFO(get_logger(), "NavigationCapability cleaned up");
    return CallbackReturn::SUCCESS;
  }

  void publishHealth()
  {
    if (!health_pub_) {
      return;
    }

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "aehub_nav2_capability_server";
    st.hardware_id = "";

    std::string summary = "";
    bool ready = false;
    if (readiness_gate_) {
      auto report = readiness_gate_->check();
      ready = report.isReady();
      summary = report.summary;
    } else {
      ready = false;
      summary = "readiness_gate_not_configured";
    }

    const bool ok = accepting_goals_ && ready;
    st.level = ok ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message = ok ? "ready" : ("not_ready: " + summary);

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "accepting_goals";
    kv.value = accepting_goals_ ? "true" : "false";
    st.values.push_back(kv);

    diagnostic_msgs::msg::KeyValue kv2;
    kv2.key = "readiness_summary";
    kv2.value = summary;
    st.values.push_back(kv2);

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(st);
    health_pub_->publish(arr);
  }

  // ---------------- Action callbacks ----------------

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigationExecute::Goal> goal)
  {
    if (!accepting_goals_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->command_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: empty command_id");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // This capability currently requires concrete coordinates.
    if (!std::isfinite(goal->x) || !std::isfinite(goal->y) || !std::isfinite(goal->theta)) {
      RCLCPP_WARN(get_logger(), "Rejecting goal %s: non-finite x/y/theta", goal->command_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    // IMPORTANT:
    // Do not REJECT on readiness/busy here.
    // If we reject at handle_goal(), the client (executor) only sees a generic "goal rejected"
    // and cannot propagate the concrete reason over MQTT.
    // Instead, accept the goal and immediately finish with a detailed error in executeGoal().
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    (void)goal_handle;

    // Cancel request is treated as STOP (owned here).
    if (accepting_goals_ && stop_burst_enabled_) {
      startStopBurst();
    }

    // Best-effort cancel via adapter. Final terminal state is emitted on adapter event.
    (void)adapter_->cancelActiveGoal("action_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Execute asynchronously; do not block executor callback.
    std::thread([this, goal_handle]() { this->executeGoal(goal_handle); }).detach();
  }

  void executeGoal(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const std::string command_id = goal->command_id;

    // One active goal at a time: if busy, fail fast with a concrete reason.
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      if (active_goal_) {
        finishWithError(command_id, "busy: active_navigation_in_progress");
        return;
      }
      active_goal_ = goal_handle;
      active_command_id_ = command_id;
      active_target_id_ = goal->target_id;
      terminal_sent_ = false;
    }

    if (!readiness_gate_) {
      finishWithError(command_id, "readiness_gate_not_configured");
      return;
    }

    // Readiness authority: check here (owned by capability, injected from main).
    auto report = readiness_gate_->check();
    if (!report.isReady()) {
      finishWithError(command_id, "not_ready: " + report.summary);
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = goal_frame_;
    pose.header.stamp = now();
    pose.pose.position.x = goal->x;
    pose.pose.position.y = goal->y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(goal->theta * 0.5);
    pose.pose.orientation.w = std::cos(goal->theta * 0.5);

    if (!adapter_->navigateToPose(command_id, pose)) {
      finishWithError(command_id, "adapter_rejected_or_not_idle");
      return;
    }

    // Terminal completion will happen via adapter event callbacks.
  }

  // ---------------- Adapter event callbacks ----------------

  void onAdapterAccepted(const std::string & command_id)
  {
    auto goal = getActiveGoal(command_id);
    if (!goal) {
      return;
    }
    // No explicit action feedback needed at accept; goal is executing.
    (void)goal;
  }

  void onAdapterSucceeded(const std::string & command_id)
  {
    finishTerminal(command_id, "succeeded", "");
  }

  void onAdapterCanceled(const std::string & command_id)
  {
    finishTerminal(command_id, "canceled", "");
  }

  void onAdapterFailed(const std::string & command_id, const std::string & error)
  {
    finishTerminal(command_id, "error", error);
  }

  // ---------------- STOP burst (owned here) ----------------

  void startStopBurst()
  {
    if (stop_burst_duration_s_ <= 0.0 || stop_burst_rate_hz_ <= 0.0) {
      return;
    }
    if (stop_timer_) {
      stop_timer_->cancel();
      stop_timer_.reset();
    }

    const auto total_msgs = static_cast<int>(std::max(1.0, stop_burst_duration_s_ * stop_burst_rate_hz_));
    stop_remaining_ = total_msgs;

    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / stop_burst_rate_hz_));

    stop_timer_ = create_wall_timer(period_ns, [this]() { this->stopTick(); });
  }

  void stopTick()
  {
    geometry_msgs::msg::Twist msg;
    cmd_vel_pub_->publish(msg);

    stop_remaining_--;
    if (stop_remaining_ <= 0 && stop_timer_) {
      stop_timer_->cancel();
      stop_timer_.reset();
    }
  }

  // ---------------- Terminal helpers ----------------

  std::shared_ptr<GoalHandle> getActiveGoal(const std::string & command_id)
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (!active_goal_ || active_command_id_ != command_id) {
      return nullptr;
    }
    return active_goal_;
  }

  void finishWithError(const std::string & command_id, const std::string & reason)
  {
    finishTerminal(command_id, "error", reason);
  }

  void finishTerminal(const std::string & command_id, const std::string & status, const std::string & reason)
  {
    std::shared_ptr<GoalHandle> goal;
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      if (!active_goal_ || active_command_id_ != command_id) {
        return;
      }
      if (terminal_sent_) {
        return;
      }
      terminal_sent_ = true;
      goal = active_goal_;
    }

    auto result = std::make_shared<NavigationExecute::Result>();
    result->command_id = command_id;
    result->status = status;
    result->reason = reason;

    if (status == "canceled") {
      goal->canceled(result);
    } else if (status == "succeeded") {
      goal->succeed(result);
    } else {
      goal->abort(result);
    }

    // Clear active state after finishing.
    std::lock_guard<std::mutex> lock(active_mutex_);
    active_goal_.reset();
    active_command_id_.clear();
    active_target_id_.clear();
  }

  // ---------------- Members ----------------

  std::shared_ptr<aehub_nav2_adapter::Nav2AdapterNode> adapter_;
  std::shared_ptr<aehub::nav2::ReadinessGate> readiness_gate_;

  rclcpp_action::Server<NavigationExecute>::SharedPtr action_server_;

  std::string action_name_;
  std::string goal_frame_;
  std::string cmd_vel_topic_;

  bool stop_burst_enabled_{true};
  double stop_burst_duration_s_{0.6};
  double stop_burst_rate_hz_{20.0};

  std::string health_topic_;
  double health_publish_period_s_{1.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
  int stop_remaining_{0};

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  // Active goal tracking (single active goal invariant)
  std::mutex active_mutex_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::string active_command_id_;
  std::string active_target_id_;
  bool terminal_sent_{false};

  bool accepting_goals_{false};
};

}  // namespace aehub_nav2_capability_server

// Exposed factory for main.cpp
std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_capability_node(
  std::shared_ptr<aehub_nav2_adapter::Nav2AdapterNode> adapter,
  std::shared_ptr<aehub::nav2::ReadinessGate> readiness_gate)
{
  return std::make_shared<aehub_nav2_capability_server::Nav2CapabilityServerNode>(
    std::move(adapter),
    std::move(readiness_gate));
}


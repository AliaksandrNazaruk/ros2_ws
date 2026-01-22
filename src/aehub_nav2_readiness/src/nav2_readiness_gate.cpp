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

#include "aehub_nav2_readiness/nav2_readiness_gate.hpp"

#include <chrono>
#include <sstream>
#include <thread>

#include <tf2/exceptions.h>

using namespace aehub::nav2;
using namespace std::chrono_literals;

namespace
{
constexpr std::chrono::milliseconds CHECK_INTERVAL_MS{100};
constexpr double AMCL_STALE_TIMEOUT_SEC = 5.0;
}  // namespace

// ============================================================
// Constructors
// ============================================================

Nav2ReadinessGate::Nav2ReadinessGate(
  const rclcpp::Node::SharedPtr & node,
  const std::string & global_frame,
  const std::string & robot_base_frame)
: node_(node),
  lifecycle_node_(nullptr),
  global_frame_(global_frame),
  robot_base_frame_(robot_base_frame)
{
  init();
}

Nav2ReadinessGate::Nav2ReadinessGate(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & lifecycle_node,
  const std::string & global_frame,
  const std::string & robot_base_frame)
: node_(nullptr),
  lifecycle_node_(lifecycle_node),
  global_frame_(global_frame),
  robot_base_frame_(robot_base_frame)
{
  init();
}

// ============================================================
// Initialization
// ============================================================

void Nav2ReadinessGate::init()
{
  if (!node_ && !lifecycle_node_) {
    throw std::runtime_error("Nav2ReadinessGate: node context is null");
  }

  auto clock = getClock();
  if (!clock) {
    throw std::runtime_error("Nav2ReadinessGate: clock unavailable");
  }

  // Snapshot + sensors (non-blocking observers; require spinning executor)
  snapshot_ = std::make_shared<WorldSnapshot>();

  // Required Nav2 lifecycle nodes (minimal set)
  const std::vector<std::string> required_lifecycle_nodes = {
    "controller_server",
    "bt_navigator"
  };
  if (node_) {
    lifecycle_sensor_ = std::make_unique<LifecycleSensor>(node_, snapshot_, required_lifecycle_nodes);
  } else {
    lifecycle_sensor_ = std::make_unique<LifecycleSensor>(lifecycle_node_, snapshot_, required_lifecycle_nodes);
  }

  // Action readiness sensor (navigate_to_pose)
  const std::vector<std::string> watched_actions = { "navigate_to_pose" };
  if (node_) {
    action_sensor_ = std::make_unique<ActionSensor>(node_, snapshot_, watched_actions);
  } else {
    action_sensor_ = std::make_unique<ActionSensor>(lifecycle_node_, snapshot_, watched_actions);
  }

  // TF chain sensor (global_frame->odom, odom->base_link)
  const std::vector<TFSensor::TransformChain> watched_tf = {
    {global_frame_, "odom"},
    {"odom", robot_base_frame_}
  };
  if (node_) {
    tf_sensor_ = std::make_unique<TFSensor>(node_, snapshot_, watched_tf);
  } else {
    tf_sensor_ = std::make_unique<TFSensor>(lifecycle_node_, snapshot_, watched_tf);
  }

  // Subscriptions (evidence only)
  const auto map_qos = rclcpp::QoS(1).transient_local().reliable();
  const auto amcl_qos = rclcpp::QoS(1);

  auto create_subs = [&](auto node_base) {
    map_subscriber_ =
      node_base->template create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos,
        std::bind(&Nav2ReadinessGate::mapCallback, this, std::placeholders::_1));

    amcl_pose_subscriber_ =
      node_base->template create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", amcl_qos,
        std::bind(&Nav2ReadinessGate::amclPoseCallback, this, std::placeholders::_1));
  };

  if (node_) {
    create_subs(node_);
  } else {
    create_subs(lifecycle_node_);
  }

  // Periodically refresh sensor snapshots (non-blocking). Must have executor spinning.
  if (node_) {
    sensors_timer_ = node_->create_wall_timer(
      200ms,
      [this]() {
        if (this->lifecycle_sensor_) {
          this->lifecycle_sensor_->updateSnapshot();
        }
        if (this->action_sensor_) {
          this->action_sensor_->updateSnapshot();
        }
        if (this->tf_sensor_) {
          this->tf_sensor_->updateSnapshot();
        }
      });
  } else if (lifecycle_node_) {
    sensors_timer_ = lifecycle_node_->create_wall_timer(
      200ms,
      [this]() {
        if (this->lifecycle_sensor_) {
          this->lifecycle_sensor_->updateSnapshot();
        }
        if (this->action_sensor_) {
          this->action_sensor_->updateSnapshot();
        }
        if (this->tf_sensor_) {
          this->tf_sensor_->updateSnapshot();
        }
      });
  } else {
    throw std::runtime_error("Nav2ReadinessGate: no node context for sensors timer");
  }

  RCLCPP_INFO(
    node_ ? node_->get_logger() : lifecycle_node_->get_logger(),
    "Nav2ReadinessGate initialized");
}

// ============================================================
// Public API
// ============================================================

ReadinessReport Nav2ReadinessGate::check()
{
  ReadinessResult result;
  result.state = ReadinessState::NOT_READY;

  if (!checkLifecycle(result)) {
  } else if (!checkActionServer(result)) {
  } else if (!checkTopics(result)) {
  } else if (!checkTF(result)) {
  } else {
    result.state = ReadinessState::READY;
    result.reason = "Nav2 is ready";
  }

  ReadinessReport report;
  report.snapshot_time = now();
  report.overall_level =
    result.isReady() ? ReadinessLevel::READY : ReadinessLevel::NOT_READY;
  report.summary = result.reason;

  report.capabilities[ReadinessCapability::TRANSPORT_READY] = result.isReady();
  report.capabilities[ReadinessCapability::NAV2_READY] = result.isReady();
  report.capabilities[ReadinessCapability::MOTION_READY] = result.isReady();

  if (!result.isReady()) {
    report.failures.push_back({
      ReadinessFailure::UNKNOWN_FAILURE,
      FailureSeverity::HARD,
      FailureClass::RECOVERABLE,
      result.reason,
      "nav2"
    });
  }

  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cached_result_ = result;
    last_report_ = report;
    last_reported_level_ = report.overall_level;
    last_check_time_ = report.snapshot_time;
  }

  return report;
}

bool Nav2ReadinessGate::waitUntilReady(std::chrono::milliseconds timeout)
{
  if (lifecycle_node_) {
    throw std::logic_error(
      "waitUntilReady() must not be used with LifecycleNode");
  }

  const auto start = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    if (std::chrono::steady_clock::now() - start >= timeout) {
      RCLCPP_WARN(node_->get_logger(), "Nav2 readiness timeout");
      return false;
    }

    if (check().isReady()) {
      RCLCPP_INFO(node_->get_logger(), "Nav2 is ready");
      return true;
    }

    std::this_thread::sleep_for(CHECK_INTERVAL_MS);
  }

  return false;
}

ReadinessResult Nav2ReadinessGate::current() const
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return cached_result_;
}

bool Nav2ReadinessGate::changed() const
{
  std::lock_guard<std::mutex> lock(cache_mutex_);
  return last_report_.overall_level != last_reported_level_;
}

// ============================================================
// Checks
// ============================================================

bool Nav2ReadinessGate::checkLifecycle(ReadinessResult & out)
{
  if (!snapshot_) {
    out.reason = "Snapshot not initialized";
    out.failures.push_back(ReadinessFailure::LIFECYCLE_CHECK_FAILED);
    return false;
  }

  const auto snap = snapshot_->takeSnapshot();
  static const std::vector<std::string> REQUIRED = {
    "controller_server",
    "bt_navigator"
  };

  std::vector<std::string> inactive;
  for (const auto & node_name : REQUIRED) {
    auto it = snap.lifecycle_states.find(node_name);
    const bool active = (it != snap.lifecycle_states.end()) && it->second.is_active;
    if (!active) {
      inactive.push_back(node_name);
      out.missing.push_back(node_name);
    }
  }

  if (!inactive.empty()) {
    std::ostringstream oss;
    oss << "Required lifecycle nodes not active: ";
    for (size_t i = 0; i < inactive.size(); ++i) {
      oss << inactive[i];
      if (i + 1 < inactive.size()) {
        oss << ", ";
      }
    }
    out.reason = oss.str();
    out.failures.push_back(ReadinessFailure::NAV2_NOT_ACTIVE);
    return false;
  }

  return true;
}

bool Nav2ReadinessGate::checkActionServer(ReadinessResult & out)
{
  if (!snapshot_) {
    out.reason = "Snapshot not initialized";
    out.failures.push_back(ReadinessFailure::UNKNOWN_FAILURE);
    return false;
  }

  const auto snap = snapshot_->takeSnapshot();
  const std::string action_name = "navigate_to_pose";
  auto it = snap.action_servers.find(action_name);
  const bool ready = (it != snap.action_servers.end()) && it->second.is_ready;
  if (!ready) {
    out.reason = "Action server not ready: " + action_name;
    out.missing.push_back(action_name);
    out.failures.push_back(ReadinessFailure::ACTION_SERVER_NOT_READY);
    return false;
  }
  return true;
}

bool Nav2ReadinessGate::checkTopics(ReadinessResult & out)
{
  if (!snapshot_) {
    out.reason = "Snapshot not initialized";
    out.failures.push_back(ReadinessFailure::UNKNOWN_FAILURE);
    return false;
  }

  const auto snap = snapshot_->takeSnapshot();

  const auto map_it = snap.topics.find("map");
  const auto amcl_it = snap.topics.find("amcl_pose");

  const bool map_ok = (map_it != snap.topics.end()) && map_it->second.received;
  const bool amcl_ok = (amcl_it != snap.topics.end()) && amcl_it->second.received;

  if (!map_ok || !amcl_ok) {
    out.reason = "Required topics missing";
    if (!map_ok) out.missing.push_back("map");
    if (!amcl_ok) out.missing.push_back("amcl_pose");
    out.failures.push_back(ReadinessFailure::MAP_NOT_AVAILABLE);
    return false;
  }

  const double age = (now() - amcl_it->second.last_message_time).seconds();
  if (age > AMCL_STALE_TIMEOUT_SEC) {
    out.reason = "AMCL pose stale";
    out.missing.push_back("amcl_pose(stale)");
    out.failures.push_back(ReadinessFailure::LOCALIZATION_STALE);
    return false;
  }

  return true;
}

bool Nav2ReadinessGate::checkTF(ReadinessResult & out)
{
  if (!snapshot_) {
    out.reason = "Snapshot not initialized";
    out.failures.push_back(ReadinessFailure::TF_CHAIN_INCOMPLETE);
    return false;
  }

  const auto snap = snapshot_->takeSnapshot();

  // odom -> base_link
  {
    const std::string key = "odom->" + robot_base_frame_;
    auto it = snap.tf_transforms.find(key);
    if (it == snap.tf_transforms.end() || !it->second.exists) {
      out.reason = "TF odom -> base_link missing";
      out.missing.push_back(key);
      out.failures.push_back(ReadinessFailure::TF_CHAIN_INCOMPLETE);
      return false;
    }
  }

  // map -> odom (only if we have map topic evidence)
  {
    const std::string key = global_frame_ + "->odom";
    auto it = snap.tf_transforms.find(key);
    if (it == snap.tf_transforms.end() || !it->second.exists) {
      out.reason = "TF map -> odom missing";
      out.missing.push_back(key);
      out.failures.push_back(ReadinessFailure::TF_CHAIN_INCOMPLETE);
      return false;
    }
  }

  return true;
}

// ============================================================
// Callbacks
// ============================================================

void Nav2ReadinessGate::mapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr)
{
  if (!snapshot_) {
    return;
  }
  WorldSnapshot::TopicState st;
  st.received = true;
  st.last_message_time = now();
  st.last_check = st.last_message_time;
  snapshot_->updateTopicState("map", st);
}

void Nav2ReadinessGate::amclPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
{
  if (!snapshot_) {
    return;
  }
  WorldSnapshot::TopicState st;
  st.received = true;
  st.last_message_time = now();
  st.last_check = st.last_message_time;
  snapshot_->updateTopicState("amcl_pose", st);
}

// ============================================================
// Time helpers
// ============================================================

rclcpp::Clock::SharedPtr Nav2ReadinessGate::getClock() const
{
  return node_ ? node_->get_clock() :
         lifecycle_node_ ? lifecycle_node_->get_clock() : nullptr;
}

rclcpp::Time Nav2ReadinessGate::now() const
{
  auto clock = getClock();
  return clock ? clock->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
}

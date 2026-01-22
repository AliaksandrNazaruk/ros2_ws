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
#include <lifecycle_msgs/msg/state.hpp>
#include <map>
#include <string>
#include <chrono>
#include <mutex>

namespace aehub::nav2 {

/**
 * @brief WorldSnapshot - snapshot of ROS world state
 *
 * This is updated asynchronously by watchers (lifecycle events, TF listener, etc.)
 * ReadinessGate evaluates checks against this snapshot (pure functions).
 *
 * Thread-safe for concurrent read/write access.
 */
struct WorldSnapshot {
  WorldSnapshot() = default;

  // Custom copy: mutex_ is NOT copied (a new mutex is created).
  // Callers must ensure they are copying from a consistent state (e.g., via takeSnapshot()).
  WorldSnapshot(const WorldSnapshot& other)
  : lifecycle_states(other.lifecycle_states),
    action_servers(other.action_servers),
    tf_transforms(other.tf_transforms),
    topics(other.topics),
    snapshot_time(other.snapshot_time)
  {}

  WorldSnapshot& operator=(const WorldSnapshot& other)
  {
    if (this == &other) {
      return *this;
    }
    lifecycle_states = other.lifecycle_states;
    action_servers = other.action_servers;
    tf_transforms = other.tf_transforms;
    topics = other.topics;
    snapshot_time = other.snapshot_time;
    return *this;
  }

  // Lifecycle states (updated by LifecycleWatcher from /transition_event)
  struct LifecycleState {
    uint8_t state_id{0};
    std::string state_label;
    bool is_active{false};
    rclcpp::Time last_update;
  };
  std::map<std::string, LifecycleState> lifecycle_states;  // node_name -> state

  // Action server states (updated by ActionWatcher)
  struct ActionServerState {
    bool exists{false};
    bool is_ready{false};
    rclcpp::Time last_update;
  };
  std::map<std::string, ActionServerState> action_servers;  // action_name -> state

  // TF transforms (updated by TFWatcher)
  struct TFState {
    bool exists{false};
    rclcpp::Time transform_time;
    rclcpp::Time last_check;
  };
  std::map<std::string, TFState> tf_transforms;  // "parent->child" -> state

  // Topic states (updated by TopicWatcher)
  struct TopicState {
    bool received{false};
    rclcpp::Time last_message_time;
    rclcpp::Time last_check;
  };
  std::map<std::string, TopicState> topics;  // topic_name -> state

  // Timestamp when snapshot was taken (atomic barrier point)
  rclcpp::Time snapshot_time;

  // Thread safety - CRITICAL: All updates must be under this mutex
  mutable std::mutex mutex_;

  /**
   * @brief Take atomic snapshot (read-locked, returns copy)
   * 
   * CRITICAL: This is the ATOMIC BARRIER - all checks see SAME state at SAME time.
   * 
   * Returns a copy of current snapshot state (consistent view).
   * All checks evaluate this SAME snapshot to ensure consistency.
   */
  WorldSnapshot takeSnapshot() const;

  /**
   * @brief Update lifecycle state (write-locked, atomic)
   */
  void updateLifecycleState(const std::string& node_name, const LifecycleState& state);

  /**
   * @brief Update action server state (write-locked, atomic)
   */
  void updateActionServerState(const std::string& action_name, const ActionServerState& state);

  /**
   * @brief Update TF transform state (write-locked, atomic)
   */
  void updateTFState(const std::string& key, const TFState& state);

  /**
   * @brief Update topic state (write-locked, atomic)
   */
  void updateTopicState(const std::string& topic_name, const TopicState& state);

  // Helpers (read from snapshot, thread-safe)
  bool isLifecycleActive(const std::string& node_name) const;
  bool isActionServerReady(const std::string& action_name) const;
  bool isTFTransformAvailable(const std::string& parent, const std::string& child) const;
  bool isTopicReceived(const std::string& topic_name) const;
  rclcpp::Duration getTFAge(const std::string& parent, const std::string& child, const rclcpp::Time& now) const;
  rclcpp::Duration getTopicAge(const std::string& topic_name, const rclcpp::Time& now) const;
};

}  // namespace aehub::nav2

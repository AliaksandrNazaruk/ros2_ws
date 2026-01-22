// Copyright 2026 Boris
//
// Licensed under the Apache License, Version 2.0

#include "aehub_nav2_readiness/world_snapshot.hpp"

namespace aehub::nav2
{

WorldSnapshot WorldSnapshot::takeSnapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  WorldSnapshot copy;
  copy.lifecycle_states = lifecycle_states;
  copy.action_servers = action_servers;
  copy.tf_transforms = tf_transforms;
  copy.topics = topics;
  copy.snapshot_time = snapshot_time;
  return copy;
}

void WorldSnapshot::updateLifecycleState(const std::string& node_name, const LifecycleState& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  lifecycle_states[node_name] = state;
  snapshot_time = state.last_update;
}

void WorldSnapshot::updateActionServerState(const std::string& action_name, const ActionServerState& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  action_servers[action_name] = state;
  snapshot_time = state.last_update;
}

void WorldSnapshot::updateTFState(const std::string& key, const TFState& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  tf_transforms[key] = state;
  snapshot_time = state.last_check;
}

void WorldSnapshot::updateTopicState(const std::string& topic_name, const TopicState& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  topics[topic_name] = state;
  snapshot_time = state.last_check;
}

bool WorldSnapshot::isLifecycleActive(const std::string& node_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = lifecycle_states.find(node_name);
  return it != lifecycle_states.end() && it->second.is_active;
}

bool WorldSnapshot::isActionServerReady(const std::string& action_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = action_servers.find(action_name);
  return it != action_servers.end() && it->second.is_ready;
}

bool WorldSnapshot::isTFTransformAvailable(const std::string& parent, const std::string& child) const
{
  const std::string key = parent + "->" + child;
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tf_transforms.find(key);
  return it != tf_transforms.end() && it->second.exists;
}

bool WorldSnapshot::isTopicReceived(const std::string& topic_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = topics.find(topic_name);
  return it != topics.end() && it->second.received;
}

rclcpp::Duration WorldSnapshot::getTFAge(const std::string& parent, const std::string& child, const rclcpp::Time& now) const
{
  const std::string key = parent + "->" + child;
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = tf_transforms.find(key);
  if (it == tf_transforms.end() || !it->second.exists) {
    return rclcpp::Duration::from_seconds(1e9);
  }
  return now - it->second.transform_time;
}

rclcpp::Duration WorldSnapshot::getTopicAge(const std::string& topic_name, const rclcpp::Time& now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = topics.find(topic_name);
  if (it == topics.end() || !it->second.received) {
    return rclcpp::Duration::from_seconds(1e9);
  }
  return now - it->second.last_message_time;
}

}  // namespace aehub::nav2


// Copyright 2026 Boris
//
// Licensed under the Apache License, Version 2.0

#include "aehub_nav2_readiness/action_sensor.hpp"

namespace aehub::nav2
{

ActionSensor::ActionSensor(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<WorldSnapshot> snapshot,
  const std::vector<std::string>& watched_actions)
: node_(std::move(node)),
  snapshot_(std::move(snapshot)),
  watched_actions_(watched_actions)
{
  initializeActionClient();
}

ActionSensor::ActionSensor(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<WorldSnapshot> snapshot,
  const std::vector<std::string>& watched_actions)
: lifecycle_node_(std::move(node)),
  snapshot_(std::move(snapshot)),
  watched_actions_(watched_actions)
{
  initializeActionClient();
}

void ActionSensor::initializeActionClient()
{
  if ((!node_ && !lifecycle_node_) || !snapshot_) {
    throw std::runtime_error("ActionSensor: node or snapshot is null");
  }

  // We only support NavigateToPose action type here.
  // Action names are passed as relative (namespaced by launch).
  if (node_) {
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      node_, "navigate_to_pose");
  } else {
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      lifecycle_node_, "navigate_to_pose");
  }
}

void ActionSensor::updateSnapshot()
{
  if ((!node_ && !lifecycle_node_) || !snapshot_) {
    return;
  }

  const auto now = node_ ? node_->now() : lifecycle_node_->now();
  auto graph = node_ ? node_->get_node_graph_interface() : lifecycle_node_->get_node_graph_interface();
  if (!graph) {
    return;
  }

  for (const auto& action_name : watched_actions_) {
    WorldSnapshot::ActionServerState st;
    st.last_update = now;

    // Non-blocking existence check via action status topic.
    // Action server creates: <action_name>/_action/status
    const std::string status_topic = action_name + "/_action/status";
    bool exists = false;
    const auto topic_names = graph->get_topic_names_and_types();
    for (const auto & [tn, _types] : topic_names) {
      if (tn == status_topic) {
        exists = true;
        break;
      }
    }

    st.exists = exists;
    st.is_ready = exists;

    snapshot_->updateActionServerState(action_name, st);
  }
}

}  // namespace aehub::nav2


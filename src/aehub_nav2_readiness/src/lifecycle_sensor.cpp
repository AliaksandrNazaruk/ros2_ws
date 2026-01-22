// Copyright 2026 Boris
//
// Licensed under the Apache License, Version 2.0

#include "aehub_nav2_readiness/lifecycle_sensor.hpp"

#include <lifecycle_msgs/msg/state.hpp>

namespace aehub::nav2
{

static bool isActiveState(uint8_t id)
{
  return id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

LifecycleSensor::LifecycleSensor(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<WorldSnapshot> snapshot,
  const std::vector<std::string>& watched_nodes)
: node_(std::move(node)),
  snapshot_(std::move(snapshot)),
  watched_nodes_(watched_nodes)
{
  initializeSubscription();
}

LifecycleSensor::LifecycleSensor(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<WorldSnapshot> snapshot,
  const std::vector<std::string>& watched_nodes)
: lifecycle_node_(std::move(node)),
  snapshot_(std::move(snapshot)),
  watched_nodes_(watched_nodes)
{
  initializeSubscription();
}

void LifecycleSensor::initializeSubscription()
{
  if ((!node_ && !lifecycle_node_) || !snapshot_) {
    throw std::runtime_error("LifecycleSensor: node or snapshot is null");
  }

  transition_subscribers_.clear();
  get_state_clients_.clear();

  // Subscribe to each <node>/transition_event and prepare <node>/get_state client.
  for (const auto& node_name : watched_nodes_) {
    const std::string transition_topic = node_name + "/transition_event";
    if (node_) {
      transition_subscribers_.push_back(
        node_->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
          transition_topic, rclcpp::QoS(10),
          [this](const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
            this->onTransitionEvent(msg);
          }));
    } else {
      transition_subscribers_.push_back(
        lifecycle_node_->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
          transition_topic, rclcpp::QoS(10),
          [this](const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg) {
            this->onTransitionEvent(msg);
          }));
    }

    const std::string service_name = node_name + "/get_state";
    get_state_clients_[node_name] =
      (node_ ? node_->create_client<lifecycle_msgs::srv::GetState>(service_name)
             : lifecycle_node_->create_client<lifecycle_msgs::srv::GetState>(service_name));
  }
}

void LifecycleSensor::onTransitionEvent(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
  if (!msg || !snapshot_) {
    return;
  }

  // TransitionEvent does not carry a node name; but subscription is per-node,
  // so we use goal state as evidence only. The per-node association is handled
  // by also polling get_state in updateSnapshot().
  (void)msg;
}

void LifecycleSensor::updateSnapshot()
{
  if ((!node_ && !lifecycle_node_) || !snapshot_) {
    return;
  }

  for (const auto& node_name : watched_nodes_) {
    auto it = get_state_clients_.find(node_name);
    if (it == get_state_clients_.end() || !it->second) {
      continue;
    }

    auto client = it->second;
    if (!client->service_is_ready()) {
      continue;
    }

    auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto now_fn = [this]() {
      return node_ ? node_->now() : lifecycle_node_->now();
    };
    (void)client->async_send_request(
      req,
      [this, node_name, now_fn](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture fut) {
        auto resp = fut.get();
        if (!resp || !this->snapshot_) {
          return;
        }

        WorldSnapshot::LifecycleState st;
        st.state_id = resp->current_state.id;
        st.state_label = resp->current_state.label;
        st.is_active = isActiveState(st.state_id);
        st.last_update = now_fn();

        this->snapshot_->updateLifecycleState(node_name, st);
      });
  }
}

}  // namespace aehub::nav2


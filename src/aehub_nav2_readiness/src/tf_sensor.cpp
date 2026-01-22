// Copyright 2026 Boris
//
// Licensed under the Apache License, Version 2.0

#include "aehub_nav2_readiness/tf_sensor.hpp"

namespace aehub::nav2
{

TFSensor::TFSensor(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<WorldSnapshot> snapshot,
  const std::vector<TransformChain>& watched_transforms)
: node_(std::move(node)),
  snapshot_(std::move(snapshot)),
  watched_transforms_(watched_transforms)
{
  initializeTF();
}

TFSensor::TFSensor(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<WorldSnapshot> snapshot,
  const std::vector<TransformChain>& watched_transforms)
: lifecycle_node_(std::move(node)),
  snapshot_(std::move(snapshot)),
  watched_transforms_(watched_transforms)
{
  initializeTF();
}

rclcpp::Clock::SharedPtr TFSensor::getClock() const
{
  if (node_) {
    return node_->get_clock();
  }
  if (lifecycle_node_) {
    return lifecycle_node_->get_clock();
  }
  return nullptr;
}

void TFSensor::initializeTF()
{
  auto clock = getClock();
  if (!clock || !snapshot_) {
    throw std::runtime_error("TFSensor: clock or snapshot is null");
  }
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void TFSensor::updateSnapshot()
{
  auto clock = getClock();
  if (!clock || !snapshot_ || !tf_buffer_) {
    return;
  }

  const auto now = clock->now();
  const rclcpp::Time latest{0, 0, clock->get_clock_type()};

  for (const auto& chain : watched_transforms_) {
    WorldSnapshot::TFState st;
    st.last_check = now;

    const bool exists = tf_buffer_->canTransform(
      chain.parent, chain.child, latest, std::chrono::milliseconds(0));

    st.exists = exists;
    st.transform_time = exists ? now : rclcpp::Time(0, 0, clock->get_clock_type());

    snapshot_->updateTFState(chain.key(), st);
  }
}

}  // namespace aehub::nav2


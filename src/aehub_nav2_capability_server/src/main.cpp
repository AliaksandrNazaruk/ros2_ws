// Copyright 2026 Boris
//
// Licensed under the Apache License, Version 2.0

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <aehub_composite_readiness/composite_readiness_gate.hpp>
#include <aehub_nav2_readiness/nav2_readiness_gate.hpp>
#include <aehub_robot_readiness/robot_readiness_gate.hpp>

#include <aehub_nav2_adapter/nav2_adapter_node.hpp>

// Factory from nav2_capability_server_node.cpp
std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_capability_node(
  std::shared_ptr<aehub_nav2_adapter::Nav2AdapterNode> adapter,
  std::shared_ptr<aehub::nav2::ReadinessGate> readiness_gate);

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // A dedicated node context for readiness watchers (subscriptions/timers).
  // This node is added to the executor to ensure readiness snapshots update.
  auto readiness_context = std::make_shared<rclcpp::Node>("aehub_readiness_context");

  // Build composite readiness using the readiness context.
  auto composite = std::make_shared<aehub::readiness::CompositeReadinessGate>();
  auto nav2_gate = std::make_shared<aehub::nav2::Nav2ReadinessGate>(readiness_context);
  auto robot_gate = std::make_shared<aehub::robot::RobotReadinessGate>(readiness_context);
  composite->addGate({"nav2", nav2_gate, true});
  composite->addGate({"robot", robot_gate, true});

  // Adapter uses composite readiness as authority (still in-process).
  auto adapter_node = std::make_shared<aehub_nav2_adapter::Nav2AdapterNode>(rclcpp::NodeOptions(), composite);

  // Create capability node that exposes the action API.
  auto capability_node = make_capability_node(adapter_node, composite);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(readiness_context);
  exec.add_node(adapter_node->get_node_base_interface());
  exec.add_node(capability_node->get_node_base_interface());

  exec.spin();
  rclcpp::shutdown();
  return 0;
}


// Copyright 2026 Boris
//
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <aehub_msgs/action/navigation_execute.hpp>

#include <aehub_nav2_readiness/readiness_gate.hpp>

#include <aehub_nav2_adapter/nav2_adapter_node.hpp>

using NavigationExecute = aehub_msgs::action::NavigationExecute;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigationExecute>;

// Factory from nav2_capability_server_node.cpp
std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_capability_node(
  std::shared_ptr<aehub_nav2_adapter::Nav2AdapterNode> adapter,
  std::shared_ptr<aehub::nav2::ReadinessGate> readiness_gate);

namespace
{

class FakeReadinessGate final : public aehub::nav2::ReadinessGate
{
public:
  explicit FakeReadinessGate(aehub::nav2::ReadinessLevel level, std::string summary)
  : level_(level),
    summary_(std::move(summary))
  {}

  aehub::nav2::ReadinessReport check() override
  {
    aehub::nav2::ReadinessReport r;
    r.overall_level = level_;
    r.summary = summary_;
    return r;
  }

  bool changed() const override { return false; }

  aehub::nav2::ReadinessResult current() const override { return {}; }

  bool waitUntilReady(std::chrono::milliseconds) override { return level_ == aehub::nav2::ReadinessLevel::READY; }

private:
  aehub::nav2::ReadinessLevel level_;
  std::string summary_;
};

NavigationExecute::Goal makeGoal(const std::string & id)
{
  NavigationExecute::Goal g;
  g.command_id = id;
  g.target_id = "t1";
  g.x = 1.0;
  g.y = 2.0;
  g.theta = 0.0;
  return g;
}

}  // namespace

TEST(Nav2CapabilityServer, RejectsGoalWhenNotReady)
{
  rclcpp::init(0, nullptr);

  auto readiness_gate = std::make_shared<FakeReadinessGate>(
    aehub::nav2::ReadinessLevel::NOT_READY, "blocked_by:robot");

  // Adapter is required by the capability node; we don't rely on Nav2 being present in this test.
  auto adapter_node = std::make_shared<aehub_nav2_adapter::Nav2AdapterNode>(
    rclcpp::NodeOptions(), readiness_gate);

  auto capability = make_capability_node(adapter_node, readiness_gate);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(adapter_node->get_node_base_interface());
  exec.add_node(capability->get_node_base_interface());

  // Configure+activate lifecycle so action server is online.
  {
    const auto s = capability->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(s.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }
  {
    const auto s = capability->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(s.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  auto client_node = std::make_shared<rclcpp::Node>("test_client");
  exec.add_node(client_node);

  auto client = rclcpp_action::create_client<NavigationExecute>(
    client_node, "capabilities/navigation/execute");

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));

  auto future_handle = client->async_send_goal(makeGoal("cmd1"));

  // Spin until future resolves or timeout.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    if (future_handle.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      break;
    }
  }

  auto handle = future_handle.get();
  EXPECT_EQ(handle, nullptr) << "Goal should be rejected when readiness is NOT_READY";

  rclcpp::shutdown();
}

TEST(Nav2CapabilityServer, AcceptsGoalWhenReady)
{
  rclcpp::init(0, nullptr);

  auto capability_readiness_gate = std::make_shared<FakeReadinessGate>(
    aehub::nav2::ReadinessLevel::READY, "ok");

  // For this unit test we only verify ACCEPT vs REJECT.
  // We intentionally inject a NOT_READY gate into the adapter so it fails fast
  // without requiring a running Nav2 stack.
  auto adapter_readiness_gate = std::make_shared<FakeReadinessGate>(
    aehub::nav2::ReadinessLevel::NOT_READY, "nav2_not_ready_in_test");

  auto adapter_node = std::make_shared<aehub_nav2_adapter::Nav2AdapterNode>(
    rclcpp::NodeOptions(), adapter_readiness_gate);

  auto capability = make_capability_node(adapter_node, capability_readiness_gate);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(adapter_node->get_node_base_interface());
  exec.add_node(capability->get_node_base_interface());

  {
    const auto s = capability->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(s.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  }
  {
    const auto s = capability->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(s.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  auto client_node = std::make_shared<rclcpp::Node>("test_client_ready");
  exec.add_node(client_node);

  auto client = rclcpp_action::create_client<NavigationExecute>(
    client_node, "capabilities/navigation/execute");

  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));

  auto future_handle = client->async_send_goal(makeGoal("cmd2"));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some();
    if (future_handle.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      break;
    }
  }

  auto handle = future_handle.get();
  ASSERT_NE(handle, nullptr) << "Goal should be accepted when readiness is READY";

  // Ensure the goal terminates cleanly (adapter will fail fast in this test).
  auto future_result = client->async_get_result(handle);
  const auto deadline2 = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline2) {
    exec.spin_some();
    if (future_result.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      break;
    }
  }
  ASSERT_EQ(future_result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


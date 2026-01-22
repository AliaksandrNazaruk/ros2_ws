// Copyright 2026 Boris
//
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <aehub_composite_readiness/composite_readiness_gate.hpp>
#include <aehub_nav2_readiness/nav2_readiness_gate.hpp>
#include <aehub_robot_readiness/robot_readiness_gate.hpp>

using aehub::readiness::CompositeReadinessGate;
using aehub::nav2::Nav2ReadinessGate;
using aehub::robot::RobotReadinessGate;
using aehub::nav2::ReadinessLevel;

TEST(StackSmoke, CompositeIsNotReadyWithoutNav2AndRobotData)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_readiness_context");

  auto composite = std::make_shared<CompositeReadinessGate>();
  auto nav2_gate = std::make_shared<Nav2ReadinessGate>(node);
  auto robot_gate = std::make_shared<RobotReadinessGate>(node);

  composite->addGate({"nav2", nav2_gate, true});
  composite->addGate({"robot", robot_gate, true});

  // No Nav2 stack and no base_controller in this test process => NOT_READY.
  auto report = composite->check();
  EXPECT_EQ(report.overall_level, ReadinessLevel::NOT_READY);
  EXPECT_FALSE(report.isReady());
  EXPECT_TRUE(report.summary.find("blocked_by") != std::string::npos);

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


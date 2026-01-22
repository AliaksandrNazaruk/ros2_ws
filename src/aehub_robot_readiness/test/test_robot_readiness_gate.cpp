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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <aehub_msgs/msg/robot_status.hpp>

#include "aehub_robot_readiness/robot_readiness_gate.hpp"

using aehub::robot::RobotReadinessGate;
using aehub::nav2::ReadinessCapability;
using aehub::nav2::ReadinessLevel;

class RobotReadinessGateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;

  void publishRobotStatus(bool motors_enabled, bool estop_active)
  {
    auto pub = node_->create_publisher<aehub_msgs::msg::RobotStatus>(
      "robot/status", rclcpp::QoS(1).transient_local());

    aehub_msgs::msg::RobotStatus st;
    st.stamp = node_->now();
    st.source = "test";
    st.driver_connected = true;
    st.pose_valid = true;
    st.velocity_valid = true;
    st.motors_enabled_valid = true;
    st.motors_enabled = motors_enabled;
    st.estop_active_valid = true;
    st.estop_active = estop_active;
    st.details = "";

    pub->publish(st);
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    rclcpp::spin_some(node_);
  }
};

// Test RG_01: no odom → NOT_READY
TEST_F(RobotReadinessGateTest, RG_01_NoOdom)
{
  RobotReadinessGate gate(node_);
  publishRobotStatus(true, false);

  // Give some time for subscriptions to initialize
  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto report = gate.check();
  EXPECT_FALSE(report.isReady());
  EXPECT_EQ(report.overall_level, ReadinessLevel::NOT_READY);
  EXPECT_TRUE(report.summary.find("odom") != std::string::npos ||
              report.summary.find("Odometry") != std::string::npos);
}

// Test RG_02: stale odom → NOT_READY
TEST_F(RobotReadinessGateTest, RG_02_StaleOdom)
{
  RobotReadinessGate gate(node_);
  publishRobotStatus(true, false);

  // Publish odom once, then wait for it to become stale
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = node_->now();
  odom_pub->publish(odom_msg);

  // Spin to receive
  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Wait for odom to become stale (>1s)
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));

  auto report = gate.check();
  EXPECT_FALSE(report.isReady());
  EXPECT_EQ(report.overall_level, ReadinessLevel::NOT_READY);
}

// Test RG_03: no cmd_vel consumer → NOT_READY
TEST_F(RobotReadinessGateTest, RG_03_NoCmdVelConsumer)
{
  RobotReadinessGate gate(node_);
  publishRobotStatus(true, false);

  // Publish odom to pass odom check
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = node_->now();
  odom_pub->publish(odom_msg);

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // No cmd_vel subscriber exists
  auto report = gate.check();
  EXPECT_FALSE(report.isReady());
  EXPECT_TRUE(report.summary.find("cmd_vel") != std::string::npos ||
              report.summary.find("velocity") != std::string::npos);
}

// Test RG_04: estop active → NOT_READY
TEST_F(RobotReadinessGateTest, RG_04_EstopActive)
{
  RobotReadinessGate gate(node_);
  publishRobotStatus(true, true);

  // Publish odom
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = node_->now();
  odom_pub->publish(odom_msg);

  // Create cmd_vel subscriber
  auto cmd_vel_sub = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](const geometry_msgs::msg::Twist::SharedPtr) {});

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto report = gate.check();
  EXPECT_FALSE(report.isReady());
  EXPECT_TRUE(report.summary.find("estop") != std::string::npos ||
              report.summary.find("Emergency") != std::string::npos);
}

// Test RG_05: all OK → READY
TEST_F(RobotReadinessGateTest, RG_05_AllOK)
{
  RobotReadinessGate gate(node_);
  publishRobotStatus(true, false);

  // Publish odom
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = node_->now();
  odom_pub->publish(odom_msg);

  // Create cmd_vel subscriber
  auto cmd_vel_sub = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](const geometry_msgs::msg::Twist::SharedPtr) {});

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto report = gate.check();
  EXPECT_TRUE(report.isReady());
  EXPECT_EQ(report.overall_level, ReadinessLevel::READY);
  EXPECT_TRUE(report.isCapabilityReady(ReadinessCapability::MOTION_READY));
}

// Test motors disabled → NOT_READY
TEST_F(RobotReadinessGateTest, MotorsDisabled)
{
  RobotReadinessGate gate(node_);
  publishRobotStatus(false, false);

  // Publish odom
  auto odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = node_->now();
  odom_pub->publish(odom_msg);

  // Create cmd_vel subscriber
  auto cmd_vel_sub = node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [](const geometry_msgs::msg::Twist::SharedPtr) {});

  rclcpp::spin_some(node_);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto report = gate.check();
  EXPECT_FALSE(report.isReady());
  EXPECT_TRUE(report.summary.find("Motors") != std::string::npos ||
              report.summary.find("motors") != std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

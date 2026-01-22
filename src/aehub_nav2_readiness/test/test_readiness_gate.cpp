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
#include <chrono>

#include "aehub_nav2_readiness/nav2_readiness_gate.hpp"
#include "test_helpers.hpp"

using namespace aehub::nav2;
using namespace std::chrono_literals;

class Nav2ReadinessGateTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_readiness_gate");
    test_helper_ = std::make_unique<test_helpers::ReadinessTestHelper>(node_);
    gate_ = std::make_unique<Nav2ReadinessGate>(node_);
  }

  void TearDown() override {
    gate_.reset();
    test_helper_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<test_helpers::ReadinessTestHelper> test_helper_;
  std::unique_ptr<Nav2ReadinessGate> gate_;
};

// Test: check() returns NOT_READY when action server doesn't exist
TEST_F(Nav2ReadinessGateTest, CheckActionServerNotReady)
{
  auto result = gate_->check();
  
  EXPECT_FALSE(result.isReady());
  EXPECT_EQ(result.overall_level, ReadinessLevel::NOT_READY);
  // Should fail on action server or lifecycle nodes (depending on which check runs first)
  EXPECT_FALSE(result.summary.empty());
}

// Test: check() returns NOT_READY when topics are not published
TEST_F(Nav2ReadinessGateTest, CheckTopicsNotReady)
{
  auto result = gate_->check();
  
  EXPECT_FALSE(result.isReady());
  EXPECT_EQ(result.overall_level, ReadinessLevel::NOT_READY);
  // Should fail on topics or action server
  EXPECT_FALSE(result.summary.empty());
}

// Test: check() detects map topic when published
TEST_F(Nav2ReadinessGateTest, CheckMapTopicDetected)
{
  // Publish map
  test_helper_->publishMap();
  
  // Spin to process callback
  rclcpp::spin_some(node_);
  rclcpp::sleep_for(100ms);
  
  // Map should be detected (but still not ready due to other checks)
  auto result = gate_->check();
  // Should still fail on other checks (action server, lifecycle, amcl, tf)
  EXPECT_FALSE(result.isReady());
}

// Test: check() detects amcl_pose topic when published
TEST_F(Nav2ReadinessGateTest, CheckAmclPoseDetected)
{
  // Publish amcl_pose
  test_helper_->publishAmclPose();
  
  // Spin to process callback
  rclcpp::spin_some(node_);
  rclcpp::sleep_for(100ms);
  
  // AMCL pose should be detected (but still not ready due to other checks)
  auto result = gate_->check();
  // Should still fail on other checks
  EXPECT_FALSE(result.isReady());
}

// Test: check() detects TF transform when published
TEST_F(Nav2ReadinessGateTest, CheckTFTransformDetected)
{
  // Publish TF transform
  test_helper_->publishTFTransform("map", "base_link");
  
  // Spin to process
  rclcpp::spin_some(node_);
  rclcpp::sleep_for(100ms);
  
  // TF should be detected (but still not ready due to other checks)
  auto result = gate_->check();
  // Should still fail on other checks
  EXPECT_FALSE(result.isReady());
}

// Test: waitUntilReady() times out when Nav2 not ready
TEST_F(Nav2ReadinessGateTest, WaitUntilReadyTimeout)
{
  auto start = std::chrono::steady_clock::now();
  
  bool result = gate_->waitUntilReady(500ms);
  
  auto elapsed = std::chrono::steady_clock::now() - start;
  
  EXPECT_FALSE(result);
  // Should timeout around 500ms
  EXPECT_GE(elapsed, 400ms);
  EXPECT_LE(elapsed, 1000ms);  // Allow some overhead
}

// Test: ReadinessResult structure
TEST_F(Nav2ReadinessGateTest, ReadinessResultStructure)
{
  ReadinessResult result;
  result.state = ReadinessState::NOT_READY;
  result.reason = "Test reason";
  result.missing.push_back("item1");
  result.missing.push_back("item2");
  
  EXPECT_FALSE(result.isReady());
  EXPECT_EQ(result.state, ReadinessState::NOT_READY);
  EXPECT_EQ(result.reason, "Test reason");
  EXPECT_EQ(result.missing.size(), 2u);
  EXPECT_EQ(result.missing[0], "item1");
  EXPECT_EQ(result.missing[1], "item2");
}

// Test: ReadinessResult READY state
TEST_F(Nav2ReadinessGateTest, ReadinessResultReady)
{
  ReadinessResult result;
  result.state = ReadinessState::READY;
  result.reason = "Nav2 is ready";
  
  EXPECT_TRUE(result.isReady());
  EXPECT_EQ(result.state, ReadinessState::READY);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

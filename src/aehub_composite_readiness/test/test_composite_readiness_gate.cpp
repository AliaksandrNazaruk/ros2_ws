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

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "aehub_composite_readiness/composite_readiness_gate.hpp"
#include "aehub_nav2_readiness/readiness_gate.hpp"
#include "aehub_nav2_readiness/readiness_result.hpp"

using aehub::readiness::CompositeReadinessGate;
using aehub::nav2::CheckPolicy;
using aehub::nav2::FailureClass;
using aehub::nav2::FailureInfo;
using aehub::nav2::FailureSeverity;
using aehub::nav2::ReadinessCapability;
using aehub::nav2::ReadinessGate;
using aehub::nav2::ReadinessLevel;
using aehub::nav2::ReadinessReport;
using aehub::nav2::ReadinessResult;
using aehub::nav2::ReadinessState;
using aehub::nav2::ReadinessFailure;

// Mock gate for testing
class MockReadinessGate : public ReadinessGate
{
public:
  explicit MockReadinessGate(ReadinessLevel level, const std::string & name = "mock")
  : level_(level), name_(name) {}

  ReadinessReport check() override
  {
    ReadinessReport report;
    report.overall_level = level_;
    report.summary = name_ + " gate: " +
      (level_ == ReadinessLevel::READY ? "READY" :
      level_ == ReadinessLevel::NOT_READY ? "NOT_READY" : "DEGRADED");
    report.capabilities[ReadinessCapability::MOTION_READY] =
      (level_ == ReadinessLevel::READY);

    if (level_ != ReadinessLevel::READY) {
      FailureInfo failure;
      failure.failure = ReadinessFailure::UNKNOWN_FAILURE;
      failure.severity = FailureSeverity::HARD;
      failure.failure_class = FailureClass::RECOVERABLE;
      failure.reason = report.summary;
      failure.component = name_;
      report.failures.push_back(failure);
    }

    return report;
  }

  ReadinessResult current() const override
  {
    ReadinessResult result;
    result.state = (level_ == ReadinessLevel::READY) ?
      ReadinessState::READY : ReadinessState::NOT_READY;
    return result;
  }

  bool changed() const override {return false;}
  bool waitUntilReady(std::chrono::milliseconds) override {return false;}

private:
  ReadinessLevel level_;
  std::string name_;
};

class CompositeReadinessGateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// Test: Empty composite → NOT_READY
TEST_F(CompositeReadinessGateTest, EmptyComposite)
{
  CompositeReadinessGate composite;
  auto report = composite.check();

  EXPECT_FALSE(report.isReady());
  EXPECT_EQ(report.overall_level, ReadinessLevel::NOT_READY);
}

// Test: All READY → READY
TEST_F(CompositeReadinessGateTest, AllReady)
{
  CompositeReadinessGate composite;

  auto nav2_gate = std::make_shared<MockReadinessGate>(
    ReadinessLevel::READY, "nav2");
  auto robot_gate = std::make_shared<MockReadinessGate>(
    ReadinessLevel::READY, "robot");

  composite.addGate({"nav2", nav2_gate, true});
  composite.addGate({"robot", robot_gate, true});

  auto report = composite.check();

  EXPECT_TRUE(report.isReady());
  EXPECT_EQ(report.overall_level, ReadinessLevel::READY);
  EXPECT_TRUE(report.isCapabilityReady(ReadinessCapability::MOTION_READY));
}

// Test: ANY NOT_READY → NOT_READY
TEST_F(CompositeReadinessGateTest, AnyNotReady)
{
  CompositeReadinessGate composite;

  auto nav2_gate = std::make_shared<MockReadinessGate>(
    ReadinessLevel::READY, "nav2");
  auto robot_gate = std::make_shared<MockReadinessGate>(
    ReadinessLevel::NOT_READY, "robot");

  composite.addGate({"nav2", nav2_gate, true});
  composite.addGate({"robot", robot_gate, true});

  auto report = composite.check();

  EXPECT_FALSE(report.isReady());
  EXPECT_EQ(report.overall_level, ReadinessLevel::NOT_READY);
  EXPECT_FALSE(report.isCapabilityReady(ReadinessCapability::MOTION_READY));
  EXPECT_EQ(report.failures.size(), 1u);  // Robot gate failure
}

// Test: ANY DEGRADED (no NOT_READY) → DEGRADED
TEST_F(CompositeReadinessGateTest, AnyDegraded)
{
  CompositeReadinessGate composite;

  auto nav2_gate = std::make_shared<MockReadinessGate>(
    ReadinessLevel::READY, "nav2");
  auto robot_gate = std::make_shared<MockReadinessGate>(
    ReadinessLevel::DEGRADED, "robot");

  composite.addGate({"nav2", nav2_gate, true});
  composite.addGate({"robot", robot_gate, true});

  auto report = composite.check();

  EXPECT_FALSE(report.isReady());
  EXPECT_EQ(report.overall_level, ReadinessLevel::DEGRADED);
}

// Test: Failures aggregation
TEST_F(CompositeReadinessGateTest, FailuresAggregation)
{
  CompositeReadinessGate composite;

  auto nav2_gate = std::make_shared<MockReadinessGate>(
    ReadinessLevel::NOT_READY, "nav2");
  auto robot_gate = std::make_shared<MockReadinessGate>(
    ReadinessLevel::NOT_READY, "robot");

  composite.addGate({"nav2", nav2_gate, true});
  composite.addGate({"robot", robot_gate, true});

  auto report = composite.check();

  EXPECT_EQ(report.failures.size(), 2u);  // Both gates have failures
}

// Test: Missing required gate => NOT_READY
TEST_F(CompositeReadinessGateTest, MissingRequiredGate)
{
  CompositeReadinessGate composite;

  auto gate1 = std::make_shared<MockReadinessGate>(
    ReadinessLevel::READY, "gate1");
  composite.addGate({"gate1", gate1, true});

  composite.addGate({"missing_gate", nullptr, true});

  auto report = composite.check();
  EXPECT_FALSE(report.isReady());
  EXPECT_EQ(report.overall_level, ReadinessLevel::NOT_READY);
  EXPECT_TRUE(report.summary.find("missing_gate") != std::string::npos);
}

// Test: Capabilities AND logic
TEST_F(CompositeReadinessGateTest, CapabilitiesAndLogic)
{
  CompositeReadinessGate composite;

  auto gate1 = std::make_shared<MockReadinessGate>(
    ReadinessLevel::READY, "gate1");
  auto gate2 = std::make_shared<MockReadinessGate>(
    ReadinessLevel::NOT_READY, "gate2");

  composite.addGate({"gate1", gate1, true});
  composite.addGate({"gate2", gate2, true});

  auto report = composite.check();

  // Capability should be false because gate2 is NOT_READY
  EXPECT_FALSE(report.isCapabilityReady(ReadinessCapability::MOTION_READY));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

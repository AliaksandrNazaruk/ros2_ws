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

#pragma once

#include <vector>
#include <mutex>
#include <chrono>

#include "aehub_nav2_readiness/readiness_gate.hpp"
#include "aehub_nav2_readiness/readiness_result.hpp"
#include "aehub_composite_readiness/gate_entry.hpp"

namespace aehub::readiness
{

using aehub::nav2::FailureInfo;
using aehub::nav2::FailureSeverity;
using aehub::nav2::FailureClass;
using aehub::nav2::ReadinessCapability;
using aehub::nav2::ReadinessGate;
using aehub::nav2::ReadinessLevel;
using aehub::nav2::ReadinessReport;
using aehub::nav2::ReadinessResult;
using aehub::nav2::ReadinessState;
using aehub::nav2::ReadinessFailure;

/**
 * @brief CompositeReadinessGate - aggregates multiple ReadinessGate instances
 *
 * This is a pure orchestrator/aggregator that:
 * - Does NOT manage gates
 * - Does NOT publish
 * - Does NOT block executors
 * - Does NOT know details about Nav2/robot
 *
 * Aggregation rules:
 * - Level: ANY NOT_READY → NOT_READY, else ANY DEGRADED → DEGRADED, else READY
 * - Failures: union of all gate failures
 * - Capabilities: READY only if ALL gates confirm
 */
class CompositeReadinessGate : public ReadinessGate
{
public:
  CompositeReadinessGate() = default;

  void addGate(const GateEntry & entry);

  ReadinessReport check() override;
  ReadinessResult current() const override;
  bool changed() const override;
  bool waitUntilReady(std::chrono::milliseconds timeout) override;

private:
  ReadinessLevel aggregateLevel(const std::vector<ReadinessReport> & reports) const;

  std::vector<GateEntry> gates_;

  mutable std::mutex mutex_;
  ReadinessResult cached_result_;
  ReadinessReport last_report_;
  ReadinessLevel last_level_{ReadinessLevel::NOT_READY};   // last computed level
  ReadinessLevel prev_level_{ReadinessLevel::NOT_READY};   // previous level (for edge detection)
  bool last_changed_{false};
};

}  // namespace aehub::readiness

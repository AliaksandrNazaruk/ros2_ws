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

#include "aehub_composite_readiness/composite_readiness_gate.hpp"

#include <thread>
#include <sstream>
#include <algorithm>
#include <optional>

using aehub::nav2::FailureClass;
using aehub::nav2::FailureInfo;
using aehub::nav2::FailureSeverity;
using aehub::nav2::ReadinessCapability;
using aehub::nav2::ReadinessFailure;
using aehub::nav2::ReadinessGate;
using aehub::nav2::ReadinessLevel;
using aehub::nav2::ReadinessReport;
using aehub::nav2::ReadinessResult;
using aehub::nav2::ReadinessState;

namespace
{
constexpr std::chrono::milliseconds CHECK_INTERVAL{100};
}  // namespace

void aehub::readiness::CompositeReadinessGate::addGate(const GateEntry & entry)
{
  std::lock_guard<std::mutex> lock(mutex_);
  gates_.push_back(entry);
}

ReadinessReport aehub::readiness::CompositeReadinessGate::check()
{
  struct NamedReport
  {
    std::string name;
    bool required{true};
    std::shared_ptr<aehub::nav2::ReadinessGate> gate;
    std::optional<ReadinessReport> report;
  };

  std::vector<NamedReport> named;
  named.reserve(gates_.size());

  // Collect reports from all gates
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & entry : gates_) {
      NamedReport nr;
      nr.name = entry.name;
      nr.required = entry.required;
      nr.gate = entry.gate;
      if (entry.gate) {
        nr.report = entry.gate->check();
      }
      named.push_back(std::move(nr));
    }
  }

  ReadinessReport out;
  // Snapshot time: best-effort first available report
  out.snapshot_time = named.empty() || !named.front().report.has_value() ?
    rclcpp::Time(0, 0, RCL_ROS_TIME) :
    named.front().report->snapshot_time;

  // Required gate missing => hard NOT_READY
  bool required_missing = false;
  for (const auto & nr : named) {
    if (nr.required && !nr.gate) {
      required_missing = true;
      FailureInfo f;
      f.failure = ReadinessFailure::UNKNOWN_FAILURE;
      f.severity = FailureSeverity::HARD;
      f.failure_class = FailureClass::FATAL;
      f.reason = "required_gate_missing:" + nr.name;
      f.component = "composite";
      out.failures.push_back(std::move(f));
    }
  }

  // Aggregate level from available reports (ignores missing optional gates)
  std::vector<ReadinessReport> reports;
  reports.reserve(named.size());
  for (const auto & nr : named) {
    if (nr.report.has_value()) {
      reports.push_back(*nr.report);
    }
  }
  if (required_missing) {
    out.overall_level = ReadinessLevel::NOT_READY;
  } else if (reports.empty()) {
    // No evidence at all => NOT_READY (conservative default)
    out.overall_level = ReadinessLevel::NOT_READY;
  } else {
    out.overall_level = aggregateLevel(reports);
  }

  // Aggregate capabilities (AND logic: all must be ready)
  // If any required gate is missing -> all capabilities are false.
  if (required_missing || reports.empty()) {
    out.capabilities[ReadinessCapability::TRANSPORT_READY] = false;
    out.capabilities[ReadinessCapability::NAV2_READY] = false;
    out.capabilities[ReadinessCapability::MOTION_READY] = false;
  }
  for (const auto & report : reports) {
    for (const auto & [cap, ok] : report.capabilities) {
      // Initialize as true if first time, then AND
      if (out.capabilities.find(cap) == out.capabilities.end()) {
        out.capabilities[cap] = true;
      }
      out.capabilities[cap] &= ok;
    }
  }

  // Aggregate failures (union of all failures)
  for (const auto & report : reports) {
    out.failures.insert(
      out.failures.end(),
      report.failures.begin(),
      report.failures.end());
  }

  // Summary: keep it short, list which gates are blocking.
  {
    std::vector<std::string> blockers;
    blockers.reserve(named.size());
    for (const auto & nr : named) {
      if (nr.required && !nr.gate) {
        blockers.push_back(nr.name);
        continue;
      }
      if (!nr.report.has_value()) {
        continue;  // optional missing
      }
      if (nr.report->overall_level != ReadinessLevel::READY) {
        blockers.push_back(nr.name);
      }
    }

    std::ostringstream oss;
    if (out.overall_level == ReadinessLevel::READY) {
      oss << "READY";
    } else if (out.overall_level == ReadinessLevel::DEGRADED) {
      oss << "DEGRADED";
    } else {
      oss << "NOT_READY";
    }
    if (!blockers.empty()) {
      oss << " (blocked_by: ";
      for (size_t i = 0; i < blockers.size(); ++i) {
        oss << blockers[i];
        if (i + 1 < blockers.size()) {
          oss << ", ";
        }
      }
      oss << ")";
    }
    out.summary = oss.str();
  }

  // Cache result
  {
    std::lock_guard<std::mutex> lock(mutex_);
    prev_level_ = last_level_;
    last_level_ = out.overall_level;
    last_changed_ = (last_level_ != prev_level_);

    cached_result_.state =
      (out.overall_level == ReadinessLevel::READY) ?
      ReadinessState::READY :
      ReadinessState::NOT_READY;
    cached_result_.reason = out.summary;

    last_report_ = out;
  }

  return out;
}

ReadinessLevel aehub::readiness::CompositeReadinessGate::aggregateLevel(
  const std::vector<ReadinessReport> & reports) const
{
  bool any_degraded = false;

  for (const auto & report : reports) {
    if (report.overall_level == ReadinessLevel::NOT_READY) {
      return ReadinessLevel::NOT_READY;
    }
    if (report.overall_level == ReadinessLevel::DEGRADED) {
      any_degraded = true;
    }
  }

  return any_degraded ? ReadinessLevel::DEGRADED : ReadinessLevel::READY;
}

ReadinessResult aehub::readiness::CompositeReadinessGate::current() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cached_result_;
}

bool aehub::readiness::CompositeReadinessGate::changed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_changed_;
}

bool aehub::readiness::CompositeReadinessGate::waitUntilReady(std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    if (std::chrono::steady_clock::now() - start >= timeout) {
      return false;
    }

    if (check().isReady()) {
      return true;
    }

    std::this_thread::sleep_for(CHECK_INTERVAL);
  }

  return false;
}

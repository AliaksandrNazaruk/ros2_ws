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

#include <chrono>
#include "aehub_nav2_readiness/readiness_result.hpp"

namespace aehub::nav2 {

/**
 * @brief ReadinessGate interface - determines if Nav2 is ready to accept navigation commands
 *
 * This is a pure domain service (not a ROS node). It should:
 * - Not create publishers/subscribers
 * - Not have timers
 * - Not do spin operations
 * - Be called from executor/application layer
 */
class ReadinessGate {
public:
  virtual ~ReadinessGate() = default;

  /**
   * @brief Get current readiness report (snapshot-based, non-blocking)
   * 
   * This is a PURE AGGREGATION over current world snapshot.
   * Never blocks, never waits, never has side effects.
   * 
   * CRITICAL: This method must be called from a spinning executor context.
   * All blocking operations (TF, lifecycle, action) have timeouts to prevent deadlocks.
   * 
   * @return ReadinessReport with overall level (READY/NOT_READY/DEGRADED) and detailed items
   */
  virtual ReadinessReport check() = 0;

  /**
   * @brief Check if readiness state changed since last check
   * 
   * Returns true if overall readiness level changed (READY ↔ NOT_READY ↔ DEGRADED)
   * Useful for debouncing, telemetry, edge detection.
   * 
   * @return true if readiness level changed, false otherwise
   */
  virtual bool changed() const = 0;

  /**
   * @brief Get current cached readiness state (non-blocking)
   * @deprecated Use check() instead
   * @return Current readiness result (legacy format)
   */
  virtual ReadinessResult current() const = 0;

  /**
   * @brief Wait until ready (blocking with timeout)
   * 
   * This method polls check() periodically until ready or timeout.
   * This is the ONLY method that may block/wait.
   * 
   * @param timeout Maximum time to wait
   * @return true if ready, false if timeout
   */
  virtual bool waitUntilReady(
    std::chrono::milliseconds timeout) = 0;
};

}  // namespace aehub::nav2

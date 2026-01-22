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

#include "aehub_nav2_readiness/world_snapshot.hpp"
#include "aehub_nav2_readiness/readiness_result.hpp"
#include <memory>

namespace aehub::nav2 {

/**
 * @brief Base class for individual readiness checks
 *
 * Each check is a PURE FUNCTION over WorldSnapshot:
 * - NO waiting/blocking
 * - NO retries
 * - NO polling loops
 * - NO spin operations
 * - NO side effects
 * - Just instant evaluation over snapshot
 *
 * Checks are evaluated by ReadinessGate against a snapshot that is updated
 * asynchronously by watchers (LifecycleWatcher, TFWatcher, etc.)
 */
class ReadinessCheck {
public:
  virtual ~ReadinessCheck() = default;

  /**
   * @brief Evaluate check against world snapshot (pure function, non-blocking)
   * 
   * This is a PURE FUNCTION - same snapshot → same result
   * 
   * @param snapshot Current world state snapshot (updated by watchers)
   * @param now Current time (for age checks)
   * @return ReadinessItem with level (READY/NOT_READY/DEGRADED) and reason
   */
  virtual ReadinessItem evaluate(
    const WorldSnapshot& snapshot,
    const rclcpp::Time& now) = 0;

  /**
   * @brief Get name of this check (for logging/debugging)
   */
  virtual std::string getName() const = 0;
};

using ReadinessCheckPtr = std::shared_ptr<ReadinessCheck>;

}  // namespace aehub::nav2

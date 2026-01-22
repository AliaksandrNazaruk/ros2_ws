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
#include <string>

namespace aehub::nav2 {

/**
 * @brief ReadinessPolicy - explicit contract for what blocks navigation
 *
 * CRITICAL: Defines WHICH checks block navigation.
 * Not all checks are equal - some are hard requirements, others allow degradation.
 *
 * This makes ReadinessGate deterministic and configurable.
 */
struct ReadinessPolicy {
  // Hard requirements (always block navigation)
  bool require_tf{true};                  ///< TF transforms required (always blocks)
  bool require_action_server{true};       ///< Action server required (always blocks)
  bool require_lifecycle_active{true};    ///< Lifecycle nodes must be ACTIVE (always blocks)

  // Conditional requirements (may allow degradation)
  bool require_map{true};                 ///< Map server required (may be optional in some modes)
  bool require_localization{true};        ///< Localization required (may allow degraded navigation)

  // Soft requirements (degrade but don't block)
  bool allow_degraded_localization{false}; ///< Allow navigation with stale localization

  // Policy name (for logging/debugging)
  std::string name{"default"};

  // Factory methods for common policies
  static ReadinessPolicy strict() {
    ReadinessPolicy policy;
    policy.name = "strict";
    policy.require_tf = true;
    policy.require_action_server = true;
    policy.require_lifecycle_active = true;
    policy.require_map = true;
    policy.require_localization = true;
    policy.allow_degraded_localization = false;
    return policy;
  }

  static ReadinessPolicy degraded() {
    ReadinessPolicy policy;
    policy.name = "degraded";
    policy.require_tf = true;
    policy.require_action_server = true;
    policy.require_lifecycle_active = true;
    policy.require_map = false;  // May work without map in some modes
    policy.require_localization = false;  // Allow degraded navigation
    policy.allow_degraded_localization = true;
    return policy;
  }

  static ReadinessPolicy minimal() {
    ReadinessPolicy policy;
    policy.name = "minimal";
    policy.require_tf = true;
    policy.require_action_server = true;
    policy.require_lifecycle_active = true;
    policy.require_map = false;
    policy.require_localization = false;
    policy.allow_degraded_localization = true;
    return policy;
  }
};

/**
 * @brief TimingPolicy - explicit contract for stability/hysteresis
 *
 * CRITICAL: Defines timing requirements for readiness.
 * Prevents readiness flaps during startup/recovery.
 *
 * READY only if:
 * - State has been stable for stable_required duration
 * - No failures occurred within max_wait window
 */
struct TimingPolicy {
  /// Required duration of stable state before READY (hysteresis)
  std::chrono::milliseconds stable_required{500};  // Default: 500ms stability required

  /// Maximum time to wait for readiness (timeout)
  std::chrono::milliseconds max_wait{5000};  // Default: 5s timeout

  /// Minimum time between readiness checks (debouncing)
  std::chrono::milliseconds min_check_interval{100};  // Default: 100ms between checks

  TimingPolicy() = default;

  TimingPolicy(
    std::chrono::milliseconds stable_ms,
    std::chrono::milliseconds max_wait_ms,
    std::chrono::milliseconds min_interval_ms)
  : stable_required(stable_ms)
  , max_wait(max_wait_ms)
  , min_check_interval(min_interval_ms)
  {}
};

}  // namespace aehub::nav2

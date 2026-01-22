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

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <map>
#include <string>
#include <vector>

namespace aehub::nav2 {

enum class ReadinessLevel {
  READY,           ///< Ready (all checks passed)
  NOT_READY,       ///< Not ready (HARD failures - navigation blocked)
  DEGRADED         ///< Degraded (SOFT failures - navigation possible but degraded)
};

/**
 * @brief Granular readiness levels for different system capabilities
 */
enum class ReadinessCapability {
  TRANSPORT_READY,  ///< Can accept navigation commands (transport layer ready)
  NAV2_READY,       ///< Nav2 stack is operational (lifecycle + action server ready)
  MOTION_READY      ///< Can execute motion (localization + TF + map ready)
};

enum class ReadinessState {
  UNKNOWN,    ///< State not yet checked
  NOT_READY,  ///< Not ready (has failures)
  READY,      ///< Ready to accept commands
  DEGRADED    ///< Degraded but acceptable
};

enum class FailureSeverity {
  HARD,  ///< Hard failure - blocks navigation completely
  SOFT   ///< Soft failure - allows degraded navigation
};

enum class FailureClass {
  TRANSIENT,   ///< Transient failure - temporary, should recover automatically (e.g., TF not yet available)
  RECOVERABLE, ///< Recoverable failure - can be fixed by recovery actions (e.g., AMCL lost)
  FATAL        ///< Fatal failure - requires restart/operator intervention (e.g., Nav2 not running)
};

enum class ReadinessFailure {
  NONE,                    ///< No failure (ready)
  
  // HARD failures (block navigation)
  NAV2_NOT_ACTIVE,         ///< Nav2 lifecycle nodes not active
  ACTION_SERVER_MISSING,   ///< Action server /navigate_to_pose not available
  ACTION_SERVER_NOT_READY, ///< Action server exists but not ready
  MAP_NOT_AVAILABLE,       ///< Map server not active
  TF_CHAIN_INCOMPLETE,     ///< TF transform chain incomplete (map → odom → base_link)
  TF_STALE,                ///< TF transforms too stale (>threshold)

  // HARD failures (robot / motion execution)
  ROBOT_STATUS_MISSING,    ///< RobotStatus not received
  ROBOT_STATUS_STALE,      ///< RobotStatus is stale
  ODOM_MISSING,            ///< /odom not received
  ODOM_STALE,              ///< /odom stale
  CMD_VEL_NO_CONSUMERS,    ///< no subscribers to cmd_vel
  MOTORS_DISABLED,         ///< motors disabled
  ESTOP_ACTIVE,            ///< emergency stop active
  
  // SOFT failures (degraded navigation)
  LOCALIZATION_STALE,      ///< Localization data stale but present
  COSTMAP_STALE,           ///< Costmap not updated recently
  RECOVERY_NOT_ACTIVE,     ///< Recovery server inactive (navigation possible but degraded)
  
  LIFECYCLE_CHECK_FAILED,  ///< Lifecycle state check failed
  UNKNOWN_FAILURE          ///< Unknown failure
};

struct FailureInfo {
  ReadinessFailure failure;
  FailureSeverity severity;
  FailureClass failure_class;  ///< TRANSIENT / RECOVERABLE / FATAL
  std::string reason;
  std::string component;  ///< Which component failed (e.g., "bt_navigator", "map_server")
  
  bool isHardFailure() const { return severity == FailureSeverity::HARD; }
  bool isSoftFailure() const { return severity == FailureSeverity::SOFT; }
  bool isTransient() const { return failure_class == FailureClass::TRANSIENT; }
  bool isRecoverable() const { return failure_class == FailureClass::RECOVERABLE; }
  bool isFatal() const { return failure_class == FailureClass::FATAL; }
};

enum class CheckPolicy {
  IMMEDIATE,  ///< Return current cached state immediately (non-blocking)
  WAIT        ///< Wait for readiness (may block up to timeout)
};

// Note: ReadinessPolicy and TimingPolicy are defined in readiness_policy.hpp

struct ReadinessItem {
  std::string name;              ///< Check name (e.g., "LifecycleCheck", "TFCheck")
  ReadinessLevel level;          ///< READY / NOT_READY / DEGRADED
  std::string reason;            ///< Human-readable reason
  FailureSeverity severity;      ///< HARD or SOFT (if NOT_READY or DEGRADED)
};

struct ReadinessReport {
  ReadinessLevel overall_level{ReadinessLevel::NOT_READY};
  std::map<ReadinessCapability, bool> capabilities;  ///< Capability-level readiness
  
  std::vector<ReadinessItem> items;  ///< Individual check results
  std::vector<FailureInfo> failures; ///< Detailed failure information
  
  // Timestamp of when snapshot was taken
  rclcpp::Time snapshot_time{0, 0, RCL_ROS_TIME};
  
  // Human-readable summary
  std::string summary;

  bool isReady() const {
    return overall_level == ReadinessLevel::READY;
  }

  bool isCapabilityReady(ReadinessCapability cap) const {
    auto it = capabilities.find(cap);
    return it != capabilities.end() && it->second;
  }

  bool hasHardFailure() const {
    for (const auto& failure : failures) {
      if (failure.isHardFailure()) {
        return true;
      }
    }
    return false;
  }

  bool hasSoftFailure() const {
    for (const auto& failure : failures) {
      if (failure.isSoftFailure()) {
        return true;
      }
    }
    return false;
  }

  bool hasTransientFailure() const {
    for (const auto& failure : failures) {
      if (failure.isTransient()) {
        return true;
      }
    }
    return false;
  }

  bool hasFatalFailure() const {
    for (const auto& failure : failures) {
      if (failure.isFatal()) {
        return true;
      }
    }
    return false;
  }
};

// Legacy struct for backward compatibility (maps to ReadinessReport)
struct ReadinessResult {
  ReadinessState state{ReadinessState::UNKNOWN};
  std::vector<ReadinessFailure> failures;
  std::string reason;
  std::vector<std::string> missing;
  std::chrono::system_clock::time_point last_check_time;

  bool isReady() const {
    return state == ReadinessState::READY && failures.empty();
  }

  bool hasFailure(ReadinessFailure failure) const {
    return std::find(failures.begin(), failures.end(), failure) != failures.end();
  }
  
  // Conversion from ReadinessReport
  static ReadinessResult fromReport(const ReadinessReport& report);
};

}  // namespace aehub::nav2

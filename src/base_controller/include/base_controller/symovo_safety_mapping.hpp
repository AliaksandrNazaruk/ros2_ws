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

#include <string>

namespace base_controller
{

struct SafetyMappingConfig
{
  std::string motors_enabled_path;
  std::string estop_active_path;
  bool motors_enabled_invert{false};
  bool estop_active_invert{false};
  bool status_json_required{false};
};

struct SafetyMappingResult
{
  bool motors_enabled_valid{false};
  bool motors_enabled{false};
  bool estop_active_valid{false};
  bool estop_active{false};

  // If status_json_required==true and mapping fails, ok=false and error is set.
  bool ok{true};
  std::string error;
};

// Map safety-related fields from a raw Symovo AMR JSON payload (single AMR object).
// This is a pure function (no ROS, no I/O) to enable unit testing.
SafetyMappingResult mapSafetyFromRawSymovoJson(
  const std::string & raw_amr_json,
  const SafetyMappingConfig & cfg);

}  // namespace base_controller


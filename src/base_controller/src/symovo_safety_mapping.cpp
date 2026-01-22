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

#include "base_controller/symovo_safety_mapping.hpp"

#include <string>

#include <nlohmann/json.hpp>

#include "base_controller/symovo_json_extract.hpp"

namespace base_controller
{
namespace
{

void appendErr(std::string & dst, const std::string & s)
{
  if (dst.empty()) {
    dst = s;
    return;
  }
  dst += ";";
  dst += s;
}

}  // namespace

SafetyMappingResult mapSafetyFromRawSymovoJson(
  const std::string & raw_amr_json,
  const SafetyMappingConfig & cfg)
{
  SafetyMappingResult out;

  if (raw_amr_json.empty()) {
    if (cfg.status_json_required) {
      out.ok = false;
      out.error = "raw_json_empty";
    }
    return out;
  }

  nlohmann::json j;
  try {
    j = nlohmann::json::parse(raw_amr_json);
  } catch (const std::exception & e) {
    if (cfg.status_json_required) {
      out.ok = false;
      out.error = std::string("json_parse_error:") + e.what();
    }
    return out;
  }

  // motors_enabled
  if (cfg.motors_enabled_path.empty()) {
    if (cfg.status_json_required) {
      out.ok = false;
      appendErr(out.error, "motors_enabled_path_empty");
    }
  } else {
    const auto r = extractBool(j, cfg.motors_enabled_path);
    if (r.value.has_value()) {
      out.motors_enabled_valid = true;
      out.motors_enabled = cfg.motors_enabled_invert ? !(*r.value) : *r.value;
    } else if (cfg.status_json_required) {
      out.ok = false;
      appendErr(out.error, "motors_enabled:" + r.error);
    }
  }

  // estop_active
  if (cfg.estop_active_path.empty()) {
    if (cfg.status_json_required) {
      out.ok = false;
      appendErr(out.error, "estop_active_path_empty");
    }
  } else {
    const auto r = extractBool(j, cfg.estop_active_path);
    if (r.value.has_value()) {
      out.estop_active_valid = true;
      out.estop_active = cfg.estop_active_invert ? !(*r.value) : *r.value;
    } else if (cfg.status_json_required) {
      out.ok = false;
      appendErr(out.error, "estop_active:" + r.error);
    }
  }

  return out;
}

}  // namespace base_controller


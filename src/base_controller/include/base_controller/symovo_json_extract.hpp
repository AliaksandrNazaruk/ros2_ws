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

#include <optional>
#include <string>

#include <nlohmann/json.hpp>

namespace base_controller
{

struct BoolExtractResult
{
  std::optional<bool> value;
  std::string error;
};

// Extract a boolean from a JSON document by path.
//
// Supported path formats:
// - JSON pointer: "/a/b/c"
// - Dot path:     "a.b.c"
//
// Supported value types:
// - boolean: true/false
// - number:  0/1 (any non-zero treated as true)
// - string:  "true"/"false"/"1"/"0" (case-insensitive)
//
// Returns {nullopt, "..."} when the path is missing or the value cannot be converted.
BoolExtractResult extractBool(const nlohmann::json & root, const std::string & path);

}  // namespace base_controller


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

#include "base_controller/symovo_json_extract.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace base_controller
{
namespace
{

bool isDigits(const std::string & s)
{
  if (s.empty()) {
    return false;
  }
  for (const char c : s) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return false;
    }
  }
  return true;
}

std::string toLower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

const nlohmann::json * navigateDotPath(const nlohmann::json & root, const std::string & path, std::string & err)
{
  const nlohmann::json * cur = &root;
  std::istringstream ss(path);
  std::string token;
  while (std::getline(ss, token, '.')) {
    if (token.empty()) {
      err = "dot_path_empty_segment";
      return nullptr;
    }
    if (cur->is_object()) {
      if (!cur->contains(token)) {
        err = "missing_field:" + token;
        return nullptr;
      }
      cur = &((*cur)[token]);
      continue;
    }
    if (cur->is_array() && isDigits(token)) {
      const std::size_t idx = static_cast<std::size_t>(std::stoul(token));
      if (idx >= cur->size()) {
        err = "array_index_out_of_range:" + token;
        return nullptr;
      }
      cur = &((*cur)[idx]);
      continue;
    }

    err = std::string("path_type_mismatch_at:") + token + " type=" + cur->type_name();
    return nullptr;
  }
  return cur;
}

BoolExtractResult convertToBool(const nlohmann::json & v)
{
  if (v.is_boolean()) {
    return {v.get<bool>(), ""};
  }
  if (v.is_number_integer() || v.is_number_unsigned()) {
    const long long x = v.get<long long>();
    return {x != 0, ""};
  }
  if (v.is_number_float()) {
    const double x = v.get<double>();
    return {x != 0.0, ""};
  }
  if (v.is_string()) {
    const std::string s = toLower(v.get<std::string>());
    if (s == "true" || s == "1") {
      return {true, ""};
    }
    if (s == "false" || s == "0") {
      return {false, ""};
    }
    return {std::nullopt, "string_not_bool:" + s};
  }
  return {std::nullopt, std::string("unsupported_type:") + v.type_name()};
}

}  // namespace

BoolExtractResult extractBool(const nlohmann::json & root, const std::string & path)
{
  if (path.empty()) {
    return {std::nullopt, "path_empty"};
  }

  const nlohmann::json * v = nullptr;
  std::string err;

  if (!path.empty() && path[0] == '/') {
    try {
      const nlohmann::json::json_pointer ptr(path);
      if (!root.contains(ptr)) {
        return {std::nullopt, "missing_pointer:" + path};
      }
      v = &root.at(ptr);
    } catch (const std::exception & e) {
      return {std::nullopt, std::string("pointer_error:") + e.what()};
    }
  } else {
    v = navigateDotPath(root, path, err);
    if (!v) {
      return {std::nullopt, err.empty() ? "dot_path_error" : err};
    }
  }

  return convertToBool(*v);
}

}  // namespace base_controller


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

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "base_controller/symovo_json_extract.hpp"

TEST(SymovoJsonExtract, JsonPointerAndDotPath)
{
  nlohmann::json j = {
    {"safety", {{"motors_enabled", true}, {"estop_active", false}}},
  };

  {
    const auto r = base_controller::extractBool(j, "/safety/motors_enabled");
    ASSERT_TRUE(r.value.has_value());
    EXPECT_TRUE(*r.value);
    EXPECT_TRUE(r.error.empty());
  }

  {
    const auto r = base_controller::extractBool(j, "safety.estop_active");
    ASSERT_TRUE(r.value.has_value());
    EXPECT_FALSE(*r.value);
    EXPECT_TRUE(r.error.empty());
  }
}

TEST(SymovoJsonExtract, StringAndNumberConversions)
{
  nlohmann::json j = {
    {"a", "true"},
    {"b", "FALSE"},
    {"c", 1},
    {"d", 0},
    {"e", 2.0},
  };

  EXPECT_TRUE(*base_controller::extractBool(j, "a").value);
  EXPECT_FALSE(*base_controller::extractBool(j, "b").value);
  EXPECT_TRUE(*base_controller::extractBool(j, "c").value);
  EXPECT_FALSE(*base_controller::extractBool(j, "d").value);
  EXPECT_TRUE(*base_controller::extractBool(j, "e").value);
}

TEST(SymovoJsonExtract, ArrayIndexInDotPath)
{
  nlohmann::json j = {
    {"arr", nlohmann::json::array({{{"flag", 1}}, {{"flag", 0}}})},
  };

  {
    const auto r = base_controller::extractBool(j, "arr.0.flag");
    ASSERT_TRUE(r.value.has_value());
    EXPECT_TRUE(*r.value);
  }

  {
    const auto r = base_controller::extractBool(j, "arr.1.flag");
    ASSERT_TRUE(r.value.has_value());
    EXPECT_FALSE(*r.value);
  }
}

TEST(SymovoJsonExtract, MissingPathReturnsNullopt)
{
  nlohmann::json j = {{"x", 1}};

  {
    const auto r = base_controller::extractBool(j, "/nope");
    EXPECT_FALSE(r.value.has_value());
    EXPECT_FALSE(r.error.empty());
  }

  {
    const auto r = base_controller::extractBool(j, "nope");
    EXPECT_FALSE(r.value.has_value());
    EXPECT_FALSE(r.error.empty());
  }
}

TEST(SymovoJsonExtract, InversionIsCallerResponsibility)
{
  nlohmann::json j = {{"v", true}};
  const auto r = base_controller::extractBool(j, "v");
  ASSERT_TRUE(r.value.has_value());

  const bool invert = true;
  const bool mapped = invert ? !(*r.value) : *r.value;
  EXPECT_FALSE(mapped);
}


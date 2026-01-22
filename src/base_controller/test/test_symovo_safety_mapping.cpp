// Copyright 2026 Boris
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <gtest/gtest.h>

#include "base_controller/symovo_safety_mapping.hpp"

TEST(SymovoSafetyMapping, OptionalJsonDoesNotFailWhenMissing)
{
  base_controller::SafetyMappingConfig cfg;
  cfg.motors_enabled_path = "safety.motors_enabled";
  cfg.estop_active_path = "safety.estop";
  cfg.status_json_required = false;

  const auto r = base_controller::mapSafetyFromRawSymovoJson("", cfg);
  EXPECT_TRUE(r.ok);
  EXPECT_FALSE(r.motors_enabled_valid);
  EXPECT_FALSE(r.estop_active_valid);
}

TEST(SymovoSafetyMapping, RequiredJsonFailsWhenEmpty)
{
  base_controller::SafetyMappingConfig cfg;
  cfg.motors_enabled_path = "safety.motors_enabled";
  cfg.estop_active_path = "safety.estop";
  cfg.status_json_required = true;

  const auto r = base_controller::mapSafetyFromRawSymovoJson("", cfg);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("raw_json_empty"), std::string::npos);
}

TEST(SymovoSafetyMapping, RequiredJsonFailsWhenParseError)
{
  base_controller::SafetyMappingConfig cfg;
  cfg.motors_enabled_path = "safety.motors_enabled";
  cfg.estop_active_path = "safety.estop";
  cfg.status_json_required = true;

  const auto r = base_controller::mapSafetyFromRawSymovoJson("{not_json", cfg);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("json_parse_error"), std::string::npos);
}

TEST(SymovoSafetyMapping, ExtractsAndInverts)
{
  base_controller::SafetyMappingConfig cfg;
  cfg.motors_enabled_path = "state_flags.drive_ready";
  cfg.estop_active_path = "state_flags.emergency_stop";
  cfg.motors_enabled_invert = true;
  cfg.estop_active_invert = false;
  cfg.status_json_required = true;

  const std::string raw = R"({
    "state_flags": {
      "drive_ready": true,
      "emergency_stop": false
    }
  })";

  const auto r = base_controller::mapSafetyFromRawSymovoJson(raw, cfg);
  EXPECT_TRUE(r.ok) << r.error;

  ASSERT_TRUE(r.motors_enabled_valid);
  EXPECT_FALSE(r.motors_enabled);  // inverted

  ASSERT_TRUE(r.estop_active_valid);
  EXPECT_FALSE(r.estop_active);
}

TEST(SymovoSafetyMapping, RequiredJsonFailsOnMissingPath)
{
  base_controller::SafetyMappingConfig cfg;
  cfg.motors_enabled_path = "missing.field";
  cfg.estop_active_path = "also.missing";
  cfg.status_json_required = true;

  const std::string raw = R"({"x": 1})";
  const auto r = base_controller::mapSafetyFromRawSymovoJson(raw, cfg);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("motors_enabled:"), std::string::npos);
  EXPECT_NE(r.error.find("estop_active:"), std::string::npos);
}


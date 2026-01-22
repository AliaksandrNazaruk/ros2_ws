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
#include <optional>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>
#include "geometry_msgs/msg/twist.hpp"

// Forward declaration
namespace httplib
{
class Client;
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
class SSLClient;
#endif
}  // namespace httplib

namespace base_controller
{

struct Pose
{
  double x;
  double y;
  double theta;
  int map_id;
};

struct Velocity
{
  double x;
  double y;
  double theta;
};

/**
 * @brief Async HTTP client for Symovo API
 * 
 * AE.HUB MVP: Thin adapter only
 * - Sends commands to /v0/agv/{id}/move/speed
 * - Polls status from /v0/agv for pose and velocity
 */
class AsyncDriverClient
{
public:
  explicit AsyncDriverClient(const std::string & endpoint, int amr_id, bool tls_verify = false);
  ~AsyncDriverClient();

  // Submit command to send
  void submit(const geometry_msgs::msg::Twist & cmd);

  // Get current pose (from /v0/agv)
  std::optional<Pose> getPose() const;

  // Get current velocity (from /v0/agv)
  std::optional<Velocity> getVelocity() const;

  // Health check
  bool isHealthy() const;

  // Latest raw JSON payload for the configured AMR (best-effort, debug/telemetry).
  // This is intended to discover real field names (motors/e-stop/etc.) in Symovo API.
  // Returns empty string if not yet available.
  std::string getLastRawAgvJson() const;

  // Timestamp of last raw JSON update (steady clock). Empty if not yet available.
  std::optional<std::chrono::steady_clock::time_point> getLastRawAgvJsonTime() const;

private:
  void workerLoop();
  bool sendCommand(const geometry_msgs::msg::Twist & cmd, httplib::Client * cli);
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  bool sendCommand(const geometry_msgs::msg::Twist & cmd, httplib::SSLClient * cli);
#endif
  void pollStatus(httplib::Client * cli);
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  void pollStatus(httplib::SSLClient * cli);
#endif

  std::string endpoint_;
  int amr_id_;
  bool tls_verify_;
  std::atomic<bool> running_{true};
  std::thread worker_;

  std::mutex mutex_;
  std::optional<geometry_msgs::msg::Twist> pending_cmd_;
  std::atomic<bool> last_ok_{false};

  // API prefix auto-detection:
  // 0 = unknown, 1 = /v0/amr, 2 = /v0/agv
  std::atomic<int> api_mode_{0};

  // Pose and velocity (protected by mutex_)
  mutable std::mutex data_mutex_;
  std::optional<Pose> pose_;
  std::optional<Velocity> velocity_;

  // Raw JSON snapshot (protected by data_mutex_)
  std::string last_raw_agv_json_;
  std::optional<std::chrono::steady_clock::time_point> last_raw_agv_json_time_;
};

}  // namespace base_controller


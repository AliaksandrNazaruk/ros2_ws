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

#include "base_controller/async_driver_client.hpp"

#include "httplib/httplib.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace base_controller
{
namespace
{

enum class ApiMode : int
{
  Unknown = 0,
  Amr = 1,
  Agv = 2
};

const char * apiListPath(ApiMode m)
{
  return (m == ApiMode::Agv) ? "/v0/agv" : "/v0/amr";
}

const char * apiPrefix(ApiMode m)
{
  return (m == ApiMode::Agv) ? "/v0/agv" : "/v0/amr";
}

ApiMode modeFromInt(int v)
{
  if (v == static_cast<int>(ApiMode::Agv)) {
    return ApiMode::Agv;
  }
  if (v == static_cast<int>(ApiMode::Amr)) {
    return ApiMode::Amr;
  }
  return ApiMode::Unknown;
}

}  // namespace

AsyncDriverClient::AsyncDriverClient(const std::string & endpoint, int amr_id, bool tls_verify)
: endpoint_(endpoint), amr_id_(amr_id), tls_verify_(tls_verify)
{
  worker_ = std::thread(&AsyncDriverClient::workerLoop, this);
}

AsyncDriverClient::~AsyncDriverClient()
{
  running_ = false;
  if (worker_.joinable()) {
    worker_.join();
  }
}

void AsyncDriverClient::submit(const geometry_msgs::msg::Twist & cmd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_cmd_ = cmd;
}

std::optional<Pose> AsyncDriverClient::getPose() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return pose_;
}

std::optional<Velocity> AsyncDriverClient::getVelocity() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return velocity_;
}

bool AsyncDriverClient::isHealthy() const
{
  return last_ok_.load();
}

std::string AsyncDriverClient::getLastRawAgvJson() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_raw_agv_json_;
}

std::optional<std::chrono::steady_clock::time_point> AsyncDriverClient::getLastRawAgvJsonTime() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return last_raw_agv_json_time_;
}

void AsyncDriverClient::workerLoop()
{
  // Determine if endpoint uses HTTPS
  bool is_https = endpoint_.find("https://") == 0;
  std::string host;
  int port = is_https ? 443 : 80;
  
  // Extract host and port from endpoint
  if (is_https) {
    host = endpoint_.substr(8);  // Remove "https://"
  } else if (endpoint_.find("http://") == 0) {
    host = endpoint_.substr(7);  // Remove "http://"
  } else {
    host = endpoint_;
  }
  
  // Extract port if present
  size_t colon_pos = host.find(':');
  if (colon_pos != std::string::npos) {
    port = std::stoi(host.substr(colon_pos + 1));
    host = host.substr(0, colon_pos);
  }
  
  // Create appropriate client
  // Note: SSLClient and Client are not related by inheritance, so we use separate variables
  std::unique_ptr<httplib::Client> http_cli;
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  std::unique_ptr<httplib::SSLClient> ssl_cli;
#endif
  
  if (is_https) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
    ssl_cli = std::make_unique<httplib::SSLClient>(host, port);
    ssl_cli->enable_server_certificate_verification(tls_verify_);
    ssl_cli->set_connection_timeout(0, 200000);  // 200 ms
    ssl_cli->set_read_timeout(0, 200000);
    std::cerr << "[AsyncDriverClient] Using SSLClient for HTTPS connection" << std::endl;
#else
    // Fallback to HTTP if SSL not supported
    std::cerr << "[AsyncDriverClient] WARNING: CPPHTTPLIB_OPENSSL_SUPPORT not defined, using HTTP client for HTTPS endpoint!" << std::endl;
    http_cli = std::make_unique<httplib::Client>(host, port);
    http_cli->set_connection_timeout(0, 200000);  // 200 ms
    http_cli->set_read_timeout(0, 200000);
#endif
  } else {
    http_cli = std::make_unique<httplib::Client>(host, port);
    http_cli->set_connection_timeout(0, 200000);  // 200 ms
    http_cli->set_read_timeout(0, 200000);
  }
  
  // Log connection details (both stdout and stderr to ensure visibility)
  std::cout << "[AsyncDriverClient] Connecting to " << (is_https ? "https" : "http") 
            << "://" << host << ":" << port << " (amr_id=" << amr_id_ << ")" << std::endl;
  std::cerr << "[AsyncDriverClient] Connecting to " << (is_https ? "https" : "http") 
            << "://" << host << ":" << port << " (amr_id=" << amr_id_ << ")" << std::endl;

  while (running_) {
    // Send pending command
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pending_cmd_) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
        if (is_https && ssl_cli) {
          last_ok_ = sendCommand(*pending_cmd_, ssl_cli.get());
        } else if (http_cli) {
          last_ok_ = sendCommand(*pending_cmd_, http_cli.get());
        }
#else
        if (http_cli) {
          last_ok_ = sendCommand(*pending_cmd_, http_cli.get());
        }
#endif
        pending_cmd_.reset();
      }
    }

    // Poll status for pose and velocity
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
    if (is_https && ssl_cli) {
      pollStatus(ssl_cli.get());
    } else if (http_cli) {
      pollStatus(http_cli.get());
    }
#else
    if (http_cli) {
      pollStatus(http_cli.get());
    }
#endif

    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20 Hz
  }
}

bool AsyncDriverClient::sendCommand(const geometry_msgs::msg::Twist & cmd, httplib::Client * cli)
{
  nlohmann::json payload = {
    {"speed", cmd.linear.x},
    {"angular_speed", cmd.angular.z},
    {"duration", 0.05}
  };

  const auto do_put = [&](ApiMode m) -> httplib::Result {
    const std::string path =
      std::string(apiPrefix(m)) + "/" + std::to_string(amr_id_) + "/move/speed";
    return cli->Put(path.c_str(), payload.dump(), "application/json");
  };

  ApiMode m = modeFromInt(api_mode_.load());
  httplib::Result res;
  if (m == ApiMode::Unknown) {
    // Try AMR first, then AGV (real robot sometimes exposes /v0/agv).
    res = do_put(ApiMode::Amr);
    if (!(res && (res->status == 202 || res->status == 200))) {
      res = do_put(ApiMode::Agv);
      if (res && (res->status == 202 || res->status == 200)) {
        api_mode_.store(static_cast<int>(ApiMode::Agv));
      }
    } else {
      api_mode_.store(static_cast<int>(ApiMode::Amr));
    }
  } else {
    res = do_put(m);
  }

  // According to API, /v0/agv/{id}/move/speed returns 202 (Accepted)
  return res && (res->status == 202 || res->status == 200);
}

void AsyncDriverClient::pollStatus(httplib::Client * cli)
{
  const auto do_get = [&](ApiMode m) -> httplib::Result {
    return cli->Get(apiListPath(m));
  };

  ApiMode m = modeFromInt(api_mode_.load());
  httplib::Result res;
  if (m == ApiMode::Unknown) {
    res = do_get(ApiMode::Amr);
    if (!(res && res->status == 200)) {
      res = do_get(ApiMode::Agv);
      if (res && res->status == 200) {
        api_mode_.store(static_cast<int>(ApiMode::Agv));
      }
    } else {
      api_mode_.store(static_cast<int>(ApiMode::Amr));
    }
  } else {
    res = do_get(m);
  }

  if (!res) {
    last_ok_.store(false);
    // Connection failed - will retry
    static int connection_fail_count = 0;
    if (connection_fail_count++ % 100 == 0) {
      const auto mm = modeFromInt(api_mode_.load());
      std::cerr << "[AsyncDriverClient] Connection failed to " << endpoint_ << apiListPath(mm) << std::endl;
    }
    return;
  }
  
  if (res->status != 200) {
    last_ok_.store(false);
    // Log error status with response body for debugging (always log first error, then every 100th)
    static int error_count = 0;
    static bool first_error_logged = false;
    
    // Always log to both stdout and stderr for first error to ensure visibility
    if (!first_error_logged) {
      std::cerr << "========================================" << std::endl;
      std::cerr << "[AsyncDriverClient] FIRST HTTP ERROR DETECTED!" << std::endl;
      std::cerr << "========================================" << std::endl;
      const auto mm = modeFromInt(api_mode_.load());
      std::cerr << "[AsyncDriverClient] HTTP error: status=" << res->status
                << " for " << endpoint_ << apiListPath(mm) << std::endl;
      std::cerr << "[AsyncDriverClient] Response body length: " << res->body.length() << " bytes" << std::endl;
      
      if (!res->body.empty()) {
        std::cerr << "[AsyncDriverClient] Response body: " << res->body.substr(0, 1000) << std::endl;
      } else {
        std::cerr << "[AsyncDriverClient] Response body: (empty)" << std::endl;
      }
      
      // Log headers
      std::cerr << "[AsyncDriverClient] Response headers (" << res->headers.size() << "):" << std::endl;
      if (res->headers.empty()) {
        std::cerr << "  (no headers)" << std::endl;
      } else {
        for (const auto& header : res->headers) {
          std::cerr << "  " << header.first << ": " << header.second << std::endl;
        }
      }
      std::cerr << "========================================" << std::endl;
      first_error_logged = true;
    } else if (error_count % 100 == 0) {
      std::cerr << "[AsyncDriverClient] HTTP error: status=" << res->status 
                << " (error count: " << error_count << ")" << std::endl;
    }
    error_count++;
    return;
  }
  
  // Log first successful response
  static bool first_success_logged = false;
  if (!first_success_logged) {
    std::cout << "[AsyncDriverClient] Successfully connected to API, response body length: " << res->body.length() << std::endl;
    first_success_logged = true;
  }

  try {
    auto json_data = nlohmann::json::parse(res->body);
    
    if (!json_data.is_array() || json_data.empty()) {
      last_ok_.store(false);
      static int empty_count = 0;
      if (empty_count++ % 100 == 0) {
        std::cerr << "[AsyncDriverClient] JSON is not array or empty" << std::endl;
      }
      return;
    }

    // Find vehicle with configured ID
    nlohmann::json amr_data = nullptr;
    
    for (const auto & v : json_data) {
      if (v.contains("id") && v["id"].get<int>() == amr_id_) {
        amr_data = v;
        break;
      }
    }

    if (amr_data.is_null()) {
      last_ok_.store(false);
      static int not_found_count = 0;
      if (not_found_count++ % 100 == 0) {
        std::cerr << "[AsyncDriverClient] vehicle with id=" << amr_id_ << " not found in response" << std::endl;
      }
      return;
    }

    // Cache raw JSON for field discovery / readiness integration.
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_raw_agv_json_ = amr_data.dump();
      last_raw_agv_json_time_ = std::chrono::steady_clock::now();
    }
    last_ok_.store(true);

    // Parse pose
    if (amr_data.contains("pose")) {
      auto pose_json = amr_data["pose"];
      Pose pose;
      pose.x = pose_json.value("x", 0.0);
      pose.y = pose_json.value("y", 0.0);
      pose.theta = pose_json.value("theta", 0.0);
      pose.map_id = pose_json.value("map_id", 0);

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pose_ = pose;
      }
      
      // Log first successful update and every significant change
      static bool first_pose_logged = false;
      static double last_x = 0.0, last_y = 0.0;
      if (!first_pose_logged || (std::abs(pose.x - last_x) > 0.01 || std::abs(pose.y - last_y) > 0.01)) {
        std::cout << "[AsyncDriverClient] Parsed pose: x=" << pose.x 
                  << ", y=" << pose.y << ", theta=" << pose.theta << std::endl;
        first_pose_logged = true;
        last_x = pose.x;
        last_y = pose.y;
      }
    } else {
      static int no_pose_count = 0;
      if (no_pose_count++ % 100 == 0) {
        std::cerr << "[AsyncDriverClient] AMR data does not contain 'pose' field" << std::endl;
      }
    }

    // Parse velocity
    if (amr_data.contains("velocity")) {
      auto vel_json = amr_data["velocity"];
      Velocity velocity;
      velocity.x = vel_json.value("x", 0.0);
      velocity.y = vel_json.value("y", 0.0);
      velocity.theta = vel_json.value("theta", 0.0);

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        velocity_ = velocity;
      }
    } else {
      static int no_velocity_count = 0;
      if (no_velocity_count++ % 100 == 0) {
        std::cerr << "[AsyncDriverClient] AMR data does not contain 'velocity' field" << std::endl;
      }
    }
  } catch (const nlohmann::json::exception & e) {
    last_ok_.store(false);
    // Log JSON parsing errors (but not too frequently)
    static int parse_error_count = 0;
    if (parse_error_count++ % 100 == 0) {
      std::cerr << "[AsyncDriverClient] JSON parse error: " << e.what() << std::endl;
    }
  }
}

#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
// Overload for SSLClient
bool AsyncDriverClient::sendCommand(const geometry_msgs::msg::Twist & cmd, httplib::SSLClient * cli)
{
  nlohmann::json payload = {
    {"speed", cmd.linear.x},
    {"angular_speed", cmd.angular.z},
    {"duration", 0.05}
  };

  const auto do_put = [&](ApiMode m) -> httplib::Result {
    const std::string path =
      std::string(apiPrefix(m)) + "/" + std::to_string(amr_id_) + "/move/speed";
    return cli->Put(path.c_str(), payload.dump(), "application/json");
  };

  ApiMode m = modeFromInt(api_mode_.load());
  httplib::Result res;
  if (m == ApiMode::Unknown) {
    res = do_put(ApiMode::Amr);
    if (!(res && (res->status == 202 || res->status == 200))) {
      res = do_put(ApiMode::Agv);
      if (res && (res->status == 202 || res->status == 200)) {
        api_mode_.store(static_cast<int>(ApiMode::Agv));
      }
    } else {
      api_mode_.store(static_cast<int>(ApiMode::Amr));
    }
  } else {
    res = do_put(m);
  }

  return res && (res->status == 202 || res->status == 200);
}

void AsyncDriverClient::pollStatus(httplib::SSLClient * cli)
{
  const auto do_get = [&](ApiMode m) -> httplib::Result {
    return cli->Get(apiListPath(m));
  };

  ApiMode m = modeFromInt(api_mode_.load());
  httplib::Result res;
  if (m == ApiMode::Unknown) {
    res = do_get(ApiMode::Amr);
    if (!(res && res->status == 200)) {
      res = do_get(ApiMode::Agv);
      if (res && res->status == 200) {
        api_mode_.store(static_cast<int>(ApiMode::Agv));
      }
    } else {
      api_mode_.store(static_cast<int>(ApiMode::Amr));
    }
  } else {
    res = do_get(m);
  }

  if (!res) {
    last_ok_.store(false);
    static int connection_fail_count = 0;
    if (connection_fail_count++ % 100 == 0) {
      const auto mm = modeFromInt(api_mode_.load());
      std::cerr << "[AsyncDriverClient] Connection failed to " << endpoint_ << apiListPath(mm) << std::endl;
    }
    return;
  }
  
  if (res->status != 200) {
    last_ok_.store(false);
    static int error_count = 0;
    static bool first_error_logged = false;
    
    if (!first_error_logged) {
      std::cerr << "========================================" << std::endl;
      std::cerr << "[AsyncDriverClient] FIRST HTTP ERROR DETECTED (SSL)!" << std::endl;
      std::cerr << "========================================" << std::endl;
      const auto mm = modeFromInt(api_mode_.load());
      std::cerr << "[AsyncDriverClient] HTTP error: status=" << res->status
                << " for " << endpoint_ << apiListPath(mm) << std::endl;
      std::cerr << "[AsyncDriverClient] Response body length: " << res->body.length() << " bytes" << std::endl;
      
      if (!res->body.empty()) {
        std::cerr << "[AsyncDriverClient] Response body: " << res->body.substr(0, 1000) << std::endl;
      } else {
        std::cerr << "[AsyncDriverClient] Response body: (empty)" << std::endl;
      }
      
      std::cerr << "[AsyncDriverClient] Response headers (" << res->headers.size() << "):" << std::endl;
      if (res->headers.empty()) {
        std::cerr << "  (no headers)" << std::endl;
      } else {
        for (const auto& header : res->headers) {
          std::cerr << "  " << header.first << ": " << header.second << std::endl;
        }
      }
      std::cerr << "========================================" << std::endl;
      first_error_logged = true;
    } else if (error_count % 100 == 0) {
      std::cerr << "[AsyncDriverClient] HTTP error: status=" << res->status 
                << " (error count: " << error_count << ")" << std::endl;
    }
    error_count++;
    return;
  }
  
  static bool first_success_logged = false;
  if (!first_success_logged) {
    std::cout << "[AsyncDriverClient] Successfully connected to API via SSL, response body length: " << res->body.length() << std::endl;
    first_success_logged = true;
  }

  try {
    auto json_data = nlohmann::json::parse(res->body);
    
    if (!json_data.is_array() || json_data.empty()) {
      last_ok_.store(false);
      static int empty_count = 0;
      if (empty_count++ % 100 == 0) {
        std::cerr << "[AsyncDriverClient] JSON is not array or empty" << std::endl;
      }
      return;
    }

    nlohmann::json amr_data = nullptr;
    
    for (const auto & v : json_data) {
      if (v.contains("id") && v["id"].get<int>() == amr_id_) {
        amr_data = v;
        break;
      }
    }

    if (amr_data.is_null()) {
      last_ok_.store(false);
      static int not_found_count = 0;
      if (not_found_count++ % 100 == 0) {
        std::cerr << "[AsyncDriverClient] vehicle with id=" << amr_id_ << " not found in response" << std::endl;
      }
      return;
    }

    // Cache raw JSON for field discovery / readiness integration.
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_raw_agv_json_ = amr_data.dump();
      last_raw_agv_json_time_ = std::chrono::steady_clock::now();
    }
    last_ok_.store(true);

    if (amr_data.contains("pose")) {
      auto pose_json = amr_data["pose"];
      Pose pose;
      pose.x = pose_json.value("x", 0.0);
      pose.y = pose_json.value("y", 0.0);
      pose.theta = pose_json.value("theta", 0.0);
      pose.map_id = pose_json.value("map_id", 0);

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pose_ = pose;
      }
      
      static bool first_pose_logged = false;
      static double last_x = 0.0, last_y = 0.0;
      if (!first_pose_logged || (std::abs(pose.x - last_x) > 0.01 || std::abs(pose.y - last_y) > 0.01)) {
        std::cout << "[AsyncDriverClient] Parsed pose: x=" << pose.x 
                  << ", y=" << pose.y << ", theta=" << pose.theta << std::endl;
        first_pose_logged = true;
        last_x = pose.x;
        last_y = pose.y;
      }
    }

    if (amr_data.contains("velocity")) {
      auto vel_json = amr_data["velocity"];
      Velocity velocity;
      velocity.x = vel_json.value("x", 0.0);
      velocity.y = vel_json.value("y", 0.0);
      velocity.theta = vel_json.value("theta", 0.0);

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        velocity_ = velocity;
      }
    }
  } catch (const nlohmann::json::exception & e) {
    last_ok_.store(false);
    static int parse_error_count = 0;
    if (parse_error_count++ % 100 == 0) {
      std::cerr << "[AsyncDriverClient] JSON parse error: " << e.what() << std::endl;
    }
  }
}
#endif

}  // namespace base_controller


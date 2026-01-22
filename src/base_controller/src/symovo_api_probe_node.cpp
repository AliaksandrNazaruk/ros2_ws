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

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include "httplib/httplib.h"

namespace
{

struct Endpoint
{
  bool https{false};
  std::string host;
  int port{0};
};

Endpoint parseEndpoint(const std::string & endpoint)
{
  Endpoint ep;
  ep.https = endpoint.rfind("https://", 0) == 0;
  std::string host = endpoint;
  if (ep.https) {
    host = endpoint.substr(std::string("https://").size());
    ep.port = 443;
  } else if (endpoint.rfind("http://", 0) == 0) {
    host = endpoint.substr(std::string("http://").size());
    ep.port = 80;
  } else {
    ep.port = 80;
  }

  const auto colon = host.find(':');
  if (colon != std::string::npos) {
    ep.host = host.substr(0, colon);
    ep.port = std::stoi(host.substr(colon + 1));
  } else {
    ep.host = host;
  }
  return ep;
}

// Returns raw JSON string for the configured AMR/AGV object.
// The real robot API sometimes exposes GET /v0/agv instead of GET /v0/amr,
// so we try both.
std::string fetchVehicleJson(
  const rclcpp::Logger & logger,
  const std::string & endpoint,
  int vehicle_id,
  bool tls_verify,
  int timeout_ms,
  bool print_full_list)
{
  const auto ep = parseEndpoint(endpoint);
  RCLCPP_INFO(logger, "Probing Symovo API at %s://%s:%d (vehicle_id=%d)",
    ep.https ? "https" : "http", ep.host.c_str(), ep.port, vehicle_id);

  std::unique_ptr<httplib::Client> http;
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  std::unique_ptr<httplib::SSLClient> https;
#endif

  if (ep.https) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
    https = std::make_unique<httplib::SSLClient>(ep.host, ep.port);
    https->enable_server_certificate_verification(tls_verify);
    https->set_connection_timeout(0, timeout_ms * 1000);
    https->set_read_timeout(0, timeout_ms * 1000);
#else
    RCLCPP_ERROR(logger, "HTTPS endpoint but CPPHTTPLIB_OPENSSL_SUPPORT is not enabled");
    return "";
#endif
  } else {
    http = std::make_unique<httplib::Client>(ep.host, ep.port);
    http->set_connection_timeout(0, timeout_ms * 1000);
    http->set_read_timeout(0, timeout_ms * 1000);
  }

  auto fetchList = [&](const std::string & path) -> std::optional<std::string> {
    std::string body;
    int status = 0;
    bool have_response = false;

    if (ep.https) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
      const auto res = https->Get(path.c_str());
      if (res) {
        have_response = true;
        status = res->status;
        body = res->body;
      }
#endif
    } else {
      const auto res = http->Get(path.c_str());
      if (res) {
        have_response = true;
        status = res->status;
        body = res->body;
      }
    }

    if (!have_response) {
      return std::nullopt;
    }
    if (status != 200) {
      return std::nullopt;
    }
    return body;
  };

  std::string list_path;
  std::optional<std::string> list_body;
  for (const auto & p : {std::string("/v0/amr"), std::string("/v0/agv")}) {
    const auto b = fetchList(p);
    if (b.has_value()) {
      list_path = p;
      list_body = b;
      break;
    }
  }

  if (!list_body.has_value()) {
    RCLCPP_ERROR(logger, "GET /v0/amr and /v0/agv both failed (no 200 response)");
    return "";
  }

  if (print_full_list) {
    return *list_body;
  }

  nlohmann::json arr;
  try {
    arr = nlohmann::json::parse(*list_body);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to parse JSON array: %s", e.what());
    return "";
  }

  if (!arr.is_array()) {
    RCLCPP_ERROR(logger, "Expected JSON array from %s, got %s", list_path.c_str(), arr.type_name());
    return "";
  }

  for (const auto & v : arr) {
    if (v.is_object() && v.contains("id") && v["id"].is_number_integer() &&
      v["id"].get<int>() == vehicle_id)
    {
      RCLCPP_INFO(logger, "Selected object from %s (id=%d)", list_path.c_str(), vehicle_id);
      return v.dump();
    }
  }

  RCLCPP_ERROR(logger, "vehicle id=%d not found in %s response", vehicle_id, list_path.c_str());
  return "";
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("symovo_api_probe");

  const auto endpoint = node->declare_parameter<std::string>("driver_endpoint", "http://127.0.0.1");
  // Keep compatibility with base_controller params: amr_id.
  // Some real robots expose /v0/agv, so we also accept agv_id.
  const auto amr_id = node->declare_parameter<int>("amr_id", 0);
  const auto agv_id = node->declare_parameter<int>("agv_id", -1);
  const auto tls_verify = node->declare_parameter<bool>("tls_verify", false);
  const auto timeout_ms = node->declare_parameter<int>("timeout_ms", 200);
  const auto print_full_list = node->declare_parameter<bool>("print_full_list", false);

  const int vehicle_id = (agv_id >= 0) ? agv_id : amr_id;
  const auto raw = fetchVehicleJson(
    node->get_logger(),
    endpoint,
    vehicle_id,
    tls_verify,
    timeout_ms,
    print_full_list);

  if (raw.empty()) {
    rclcpp::shutdown();
    return 1;
  }

  // Print to stdout for easy copy-paste into mapping config.
  std::cout << raw << std::endl;
  RCLCPP_INFO(node->get_logger(), "Probe OK (printed %zu bytes)", raw.size());

  rclcpp::shutdown();
  return 0;
}


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

#include "aehub_nav2_readiness/lifecycle_checker.hpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <future>

namespace aehub::nav2 {

LifecycleChecker::LifecycleChecker(NodeVariant node)
: node_(node)
{
  // No need to create a default client - we create per-node clients in callGetStateService
}

bool LifecycleChecker::checkNodeState(const std::string& node_name, NodeState& result)
{
  result.node_name = node_name;
  
  uint8_t state_id = 0;
  if (!callGetStateService(node_name, state_id)) {
    result.state_id = 0;
    result.is_active = false;
    return false;
  }

  result.state_id = state_id;
  result.is_active = (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  return true;
}

std::vector<LifecycleChecker::NodeState> LifecycleChecker::checkNodes(
  const std::vector<std::string>& node_names)
{
  std::vector<NodeState> results;
  results.reserve(node_names.size());

  for (const auto& node_name : node_names) {
    NodeState state;
    checkNodeState(node_name, state);
    results.push_back(state);
  }

  return results;
}

namespace {
std::string normalizeNodeName(const std::string & name)
{
  if (name.empty()) return name;
  return (name.front() == '/') ? name : ("/" + name);
}
}  // namespace

bool LifecycleChecker::callGetStateService(
  const std::string& node_name,
  uint8_t& state_id)
{
  // Normalize node name to absolute path
  const auto abs_node_name = normalizeNodeName(node_name);
  const std::string service_name = abs_node_name + "/get_state";

  // Check if client can be created for this service
  if (!rclcpp::ok()) {
    return false;
  }

  // Create a temporary client for this specific node's service
  // Use std::visit to handle both Node and LifecycleNode types
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client;
  std::visit(
    [&service_name, &client](auto&& n) {
      client = n->template create_client<lifecycle_msgs::srv::GetState>(service_name);
    },
    node_);

  if (!client->wait_for_service(std::chrono::milliseconds(100))) {
    return false;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

  auto future = client->async_send_request(request);
  
  // Use wait_for instead of spin_until_future_complete to avoid deadlocks
  // Assumes external executor is spinning
  if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
    return false;
  }

  auto response = future.get();
  if (!response) {
    return false;
  }

  state_id = response->current_state.id;
  return true;
}

}  // namespace aehub::nav2

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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <string>
#include <vector>
#include <memory>
#include <variant>

namespace aehub::nav2 {

class LifecycleChecker {
public:
  using NodeVariant = std::variant<
    rclcpp::Node::SharedPtr,
    rclcpp_lifecycle::LifecycleNode::SharedPtr>;

  explicit LifecycleChecker(NodeVariant node);

  struct NodeState {
    std::string node_name;
    uint8_t state_id;
    bool is_active;
  };

  bool checkNodeState(const std::string& node_name, NodeState& result);
  std::vector<NodeState> checkNodes(const std::vector<std::string>& node_names);

private:
  NodeVariant node_;

  bool callGetStateService(
    const std::string& node_name,
    uint8_t& state_id);
};

}  // namespace aehub::nav2

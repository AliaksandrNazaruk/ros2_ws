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

#include "aehub_robot_readiness/cmd_vel_watcher.hpp"

namespace aehub::robot
{

CmdVelWatcher::CmdVelWatcher(const rclcpp::Node::SharedPtr & node)
: node_(node)
{
  if (!node_) {
    throw std::runtime_error("CmdVelWatcher: node is null");
  }
}

bool CmdVelWatcher::hasConsumer() const
{
  if (!node_) {
    return false;
  }

  try {
    // Relative name (namespaced by launch): /robot/<id>/cmd_vel
    const std::string topic_name = "cmd_vel";

    // Graph introspection: who subscribes to cmd_vel?
    // NOTE: count_subscribers() only works for topics published by THIS node.
    // We must use graph API here.
    auto graph = node_->get_node_graph_interface();
    if (!graph) {
      return false;
    }

    const auto infos = graph->get_subscriptions_info_by_topic(topic_name);
    return !infos.empty();
  } catch (const std::exception & e) {
    // Topic might not exist yet, or graph introspection failed
    return false;
  }
}

}  // namespace aehub::robot

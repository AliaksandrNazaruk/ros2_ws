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

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "aehub_nav2_adapter/nav2_adapter_node.hpp"

using namespace std::chrono_literals;

namespace
{
geometry_msgs::msg::PoseStamped makePose()
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = 1.0;
  p.pose.position.y = 2.0;
  p.pose.orientation.w = 1.0;
  return p;
}
}  // namespace

TEST(Nav2AdapterUnit, NavigateToPoseFailsWhenNotConfiguredOrActive)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<aehub_nav2_adapter::Nav2AdapterNode>(rclcpp::NodeOptions());

  // Not configured/active yet -> not IDLE
  EXPECT_FALSE(node->navigateToPose("cmd_1", makePose()));

  rclcpp::shutdown();
}

TEST(Nav2AdapterUnit, NavigateToPoseFailsWithNav2UnavailableWithoutServer)
{
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<aehub_nav2_adapter::Nav2AdapterNode>(rclcpp::NodeOptions());
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());

  ASSERT_TRUE(node->configure());
  ASSERT_TRUE(node->activate());
  ASSERT_EQ(node->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  std::promise<std::string> failed_promise;
  auto failed_future = failed_promise.get_future();

  aehub_nav2_adapter::Nav2Events events;
  events.onFailed = [&failed_promise](const std::string&, const std::string& error) {
    try {
      failed_promise.set_value(error);
    } catch (const std::future_error&) {
      // ignore
    }
  };
  node->setEvents(std::move(events));

  const bool sent = node->navigateToPose("cmd_2", makePose());
  EXPECT_FALSE(sent);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline &&
         failed_future.wait_for(0ms) != std::future_status::ready)
  {
    exec.spin_some(10ms);
  }

  ASSERT_EQ(failed_future.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(failed_future.get(), "nav2_unavailable");

  rclcpp::shutdown();
}

TEST(Nav2AdapterUnit, CancelIsIdempotentWithoutGoal)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<aehub_nav2_adapter::Nav2AdapterNode>(rclcpp::NodeOptions());

  EXPECT_TRUE(node->cancelActiveGoal("no_goal"));
  EXPECT_TRUE(node->cancelActiveGoal("no_goal_again"));

  rclcpp::shutdown();
}


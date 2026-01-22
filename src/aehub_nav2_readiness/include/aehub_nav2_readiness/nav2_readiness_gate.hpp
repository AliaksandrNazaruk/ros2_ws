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

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "aehub_nav2_readiness/readiness_gate.hpp"
#include "aehub_nav2_readiness/readiness_result.hpp"
#include "aehub_nav2_readiness/world_snapshot.hpp"
#include "aehub_nav2_readiness/lifecycle_sensor.hpp"
#include "aehub_nav2_readiness/action_sensor.hpp"
#include "aehub_nav2_readiness/tf_sensor.hpp"

namespace aehub::nav2
{

class Nav2ReadinessGate : public ReadinessGate
{
public:
  explicit Nav2ReadinessGate(
    const rclcpp::Node::SharedPtr & node,
    const std::string& global_frame = "map",
    const std::string& robot_base_frame = "base_link");
  explicit Nav2ReadinessGate(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & lifecycle_node,
    const std::string& global_frame = "map",
    const std::string& robot_base_frame = "base_link");

  ReadinessReport check() override;
  
  /**
   * @brief Wait until ready (blocking with timeout)
   * 
   * ⚠️ WARNING: Do NOT call this method inside LifecycleNode callbacks.
   * This method blocks and can cause deadlocks in single-threaded executors.
   * Use only in main() or external supervisor threads.
   * For lifecycle integration, use check() + timers instead.
   */
  bool waitUntilReady(std::chrono::milliseconds timeout) override;
  
  ReadinessResult current() const override;
  bool changed() const override;

private:
  void init();

  // internal checks
  bool checkLifecycle(ReadinessResult & result);
  bool checkActionServer(ReadinessResult & result);
  bool checkTF(ReadinessResult & result);
  bool checkTopics(ReadinessResult & result);

  rclcpp::Time now() const;
  rclcpp::Clock::SharedPtr getClock() const;

  // node context
  rclcpp::Node::SharedPtr node_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_node_;

  // tf (no action client - use graph introspection instead)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Snapshot + sensors (non-blocking observers)
  std::shared_ptr<WorldSnapshot> snapshot_;
  std::unique_ptr<LifecycleSensor> lifecycle_sensor_;
  std::unique_ptr<ActionSensor> action_sensor_;
  std::unique_ptr<TFSensor> tf_sensor_;
  rclcpp::TimerBase::SharedPtr sensors_timer_;

  // topic evidence
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscriber_;
  // (stored in snapshot_->topics)

  // cached result
  mutable std::mutex cache_mutex_;
  ReadinessResult cached_result_;
  ReadinessReport last_report_;
  rclcpp::Time last_check_time_{0, 0, RCL_ROS_TIME};
  ReadinessLevel last_reported_level_{ReadinessLevel::NOT_READY};

  // TF frame configuration
  std::string global_frame_;
  std::string robot_base_frame_;

  // callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};

}  // namespace aehub::nav2

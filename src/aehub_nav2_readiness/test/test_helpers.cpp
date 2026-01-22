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

#include "test_helpers.hpp"

#include <geometry_msgs/msg/quaternion.hpp>

namespace test_helpers {

ReadinessTestHelper::ReadinessTestHelper(rclcpp::Node::SharedPtr node)
: node_(node)
{
  map_publisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local());
  
  amcl_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", rclcpp::QoS(1));
  
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void ReadinessTestHelper::publishMap()
{
  auto map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map_msg->header.stamp = node_->now();
  map_msg->header.frame_id = "map";
  map_msg->info.width = 10;
  map_msg->info.height = 10;
  map_msg->info.resolution = 0.1;
  map_msg->info.origin.position.x = 0.0;
  map_msg->info.origin.position.y = 0.0;
  map_msg->info.origin.orientation.w = 1.0;
  map_msg->data.resize(100, 0);
  
  map_publisher_->publish(*map_msg);
}

void ReadinessTestHelper::publishAmclPose()
{
  auto pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose_msg->header.stamp = node_->now();
  pose_msg->header.frame_id = "map";
  pose_msg->pose.pose.position.x = 0.0;
  pose_msg->pose.pose.position.y = 0.0;
  pose_msg->pose.pose.position.z = 0.0;
  pose_msg->pose.pose.orientation.w = 1.0;
  pose_msg->pose.covariance.fill(0.0);
  
  amcl_pose_publisher_->publish(*pose_msg);
}

void ReadinessTestHelper::publishTFTransform(
  const std::string& parent,
  const std::string& child)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  
  tf_broadcaster_->sendTransform(transform);
}

}  // namespace test_helpers

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
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace aehub_nav2_adapter
{

/**
 * @brief Pure virtual interface for Nav2 adapter capability layer
 *
 * This interface provides a clean abstraction for Nav2 navigation operations,
 * encapsulating all Nav2-specific knowledge and providing a stable API
 * for the application layer.
 */
class Nav2Adapter
{
public:
  virtual ~Nav2Adapter() = default;

  // Lifecycle
  /**
   * @brief Configure the adapter
   * @return true if configuration successful, false otherwise
   */
  virtual bool configure() = 0;

  /**
   * @brief Activate the adapter
   * @return true if activation successful, false otherwise
   */
  virtual bool activate() = 0;

  /**
   * @brief Deactivate the adapter
   */
  virtual void deactivate() = 0;

  /**
   * @brief Cleanup the adapter
   */
  virtual void cleanup() = 0;

  // Commands
  /**
   * @brief Send navigation goal to Nav2
   * @param command_id Unique command identifier
   * @param target Target pose for navigation
   * @return true if goal sent successfully, false otherwise
   */
  virtual bool navigateToPose(
    const std::string& command_id,
    const geometry_msgs::msg::PoseStamped& target) = 0;

  /**
   * @brief Cancel the active navigation goal
   * @param reason Optional reason for cancellation
   * @return true if cancel request sent or no active goal, false on error
   */
  virtual bool cancelActiveGoal(
    const std::string& reason) = 0;

  // State
  /**
   * @brief Check if there is an active navigation goal
   * @return true if a goal is active, false otherwise
   */
  virtual bool hasActiveGoal() const = 0;
};

}  // namespace aehub_nav2_adapter

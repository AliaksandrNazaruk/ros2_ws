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

#include <functional>
#include <string>

namespace aehub_nav2_adapter
{

/**
 * @brief Event callbacks for Nav2 adapter
 *
 * The executor subscribes to these callbacks to receive notifications
 * about navigation goal state changes. The adapter does NOT publish
 * ROS topics - it uses callbacks for upward communication.
 */
struct Nav2Events
{
  /**
   * @brief Callback when goal is accepted by Nav2
   * @param command_id Command identifier
   */
  std::function<void(const std::string&)> onAccepted;

  /**
   * @brief Callback when goal succeeds
   * @param command_id Command identifier
   */
  std::function<void(const std::string&)> onSucceeded;

  /**
   * @brief Callback when goal fails
   * @param command_id Command identifier
   * @param error Error message
   */
  std::function<void(const std::string&, const std::string&)> onFailed;

  /**
   * @brief Callback when goal is canceled
   * @param command_id Command identifier
   */
  std::function<void(const std::string&)> onCanceled;
};

}  // namespace aehub_nav2_adapter

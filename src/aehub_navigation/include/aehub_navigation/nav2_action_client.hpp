#pragma once

#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace aehub_navigation
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

/**
 * @brief Nav2 Action Client for NavigateToPose
 * 
 * AE.HUB MVP requirement: Nav2 is NOT autonomous.
 * Nav2 is a subordinate executor of AE.HUB commands.
 */
class Nav2ActionClient : public rclcpp::Node
{
public:
  Nav2ActionClient();
  ~Nav2ActionClient();

  /**
   * @brief Send navigation goal
   * @param pose Target pose
   * @param target_id Target ID (for tracking)
   * @return true if goal sent successfully
   */
  bool sendGoal(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_id = "");

  /**
   * @brief Cancel current active goal
   * @return true if cancel request sent
   */
  bool cancelCurrentGoal();

  /**
   * @brief Check if goal is active
   * @return true if navigating
   */
  bool isNavigating() const;

  /**
   * @brief Set feedback callback
   */
  void setFeedbackCallback(
    std::function<void(const GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback>)> callback);

  /**
   * @brief Set result callback
   */
  void setResultCallback(
    std::function<void(const GoalHandleNav::WrappedResult &)> callback);

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  
  GoalHandleNav::SharedPtr current_goal_handle_;
  bool is_navigating_{false};
  
  std::function<void(const GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback>)> feedback_callback_;
  std::function<void(const GoalHandleNav::WrappedResult &)> result_callback_;

  void feedbackCallback(
    GoalHandleNav::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  void resultCallback(const GoalHandleNav::WrappedResult & result);
};

}  // namespace aehub_navigation


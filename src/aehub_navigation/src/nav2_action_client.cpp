#include "aehub_navigation/nav2_action_client.hpp"
#include <memory>

namespace aehub_navigation
{

Nav2ActionClient::Nav2ActionClient()
: Node("nav2_action_client")
{
  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    this,
    "navigate_to_pose");

  RCLCPP_INFO(get_logger(), "Nav2ActionClient initialized");
}

Nav2ActionClient::~Nav2ActionClient()
{
}

bool Nav2ActionClient::sendGoal(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_id)
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Action server not available");
    return false;
  }

  // Cancel current goal if active
  if (is_navigating_) {
    cancelCurrentGoal();
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = pose;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  
  send_goal_options.feedback_callback = std::bind(
    &Nav2ActionClient::feedbackCallback,
    this,
    std::placeholders::_1,
    std::placeholders::_2);

  send_goal_options.result_callback = std::bind(
    &Nav2ActionClient::resultCallback,
    this,
    std::placeholders::_1);

  RCLCPP_INFO(
    get_logger(),
    "Sending goal to (%.2f, %.2f, %.2f) [target_id: %s]",
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.orientation.z,
    target_id.c_str());

  auto future = action_client_->async_send_goal(goal_msg, send_goal_options);
  
  // Wait for goal to be accepted
  if (rclcpp::spin_until_future_complete(this->shared_from_this(), future, std::chrono::seconds(5)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to send goal");
    return false;
  }

  current_goal_handle_ = future.get();
  if (!current_goal_handle_) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected");
    return false;
  }

  is_navigating_ = true;
  return true;
}

bool Nav2ActionClient::cancelCurrentGoal()
{
  if (!is_navigating_ || !current_goal_handle_) {
    return false;
  }

  RCLCPP_INFO(get_logger(), "Cancelling current goal");
  
  auto future = action_client_->async_cancel_goal(current_goal_handle_);
  
  if (rclcpp::spin_until_future_complete(this->shared_from_this(), future, std::chrono::seconds(2)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "Failed to cancel goal");
    return false;
  }

  is_navigating_ = false;
  return true;
}

bool Nav2ActionClient::isNavigating() const
{
  return is_navigating_;
}

void Nav2ActionClient::setFeedbackCallback(
  std::function<void(const GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback>)> callback)
{
  feedback_callback_ = callback;
}

void Nav2ActionClient::setResultCallback(
  std::function<void(const GoalHandleNav::WrappedResult &)> callback)
{
  result_callback_ = callback;
}

void Nav2ActionClient::feedbackCallback(
  GoalHandleNav::SharedPtr goal_handle,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  (void)goal_handle;
  
  if (feedback_callback_) {
    feedback_callback_(goal_handle, feedback);
  }
}

void Nav2ActionClient::resultCallback(const GoalHandleNav::WrappedResult & result)
{
  is_navigating_ = false;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Goal succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(get_logger(), "Goal aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(get_logger(), "Goal canceled");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      break;
  }

  if (result_callback_) {
    result_callback_(result);
  }
}

}  // namespace aehub_navigation


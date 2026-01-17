// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav2_msgs:msg/MissedWaypoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav2_msgs/msg/missed_waypoint.hpp"


#ifndef NAV2_MSGS__MSG__DETAIL__MISSED_WAYPOINT__BUILDER_HPP_
#define NAV2_MSGS__MSG__DETAIL__MISSED_WAYPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nav2_msgs/msg/detail/missed_waypoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nav2_msgs
{

namespace msg
{

namespace builder
{

class Init_MissedWaypoint_error_code
{
public:
  explicit Init_MissedWaypoint_error_code(::nav2_msgs::msg::MissedWaypoint & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::msg::MissedWaypoint error_code(::nav2_msgs::msg::MissedWaypoint::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::msg::MissedWaypoint msg_;
};

class Init_MissedWaypoint_goal
{
public:
  explicit Init_MissedWaypoint_goal(::nav2_msgs::msg::MissedWaypoint & msg)
  : msg_(msg)
  {}
  Init_MissedWaypoint_error_code goal(::nav2_msgs::msg::MissedWaypoint::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_MissedWaypoint_error_code(msg_);
  }

private:
  ::nav2_msgs::msg::MissedWaypoint msg_;
};

class Init_MissedWaypoint_index
{
public:
  Init_MissedWaypoint_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MissedWaypoint_goal index(::nav2_msgs::msg::MissedWaypoint::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_MissedWaypoint_goal(msg_);
  }

private:
  ::nav2_msgs::msg::MissedWaypoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::msg::MissedWaypoint>()
{
  return nav2_msgs::msg::builder::Init_MissedWaypoint_index();
}

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__MSG__DETAIL__MISSED_WAYPOINT__BUILDER_HPP_

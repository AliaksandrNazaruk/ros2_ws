// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nav2_msgs:srv/GetCost.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav2_msgs/srv/get_cost.hpp"


#ifndef NAV2_MSGS__SRV__DETAIL__GET_COST__BUILDER_HPP_
#define NAV2_MSGS__SRV__DETAIL__GET_COST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nav2_msgs/srv/detail/get_cost__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nav2_msgs
{

namespace srv
{

namespace builder
{

class Init_GetCost_Request_theta
{
public:
  explicit Init_GetCost_Request_theta(::nav2_msgs::srv::GetCost_Request & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::srv::GetCost_Request theta(::nav2_msgs::srv::GetCost_Request::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::srv::GetCost_Request msg_;
};

class Init_GetCost_Request_y
{
public:
  explicit Init_GetCost_Request_y(::nav2_msgs::srv::GetCost_Request & msg)
  : msg_(msg)
  {}
  Init_GetCost_Request_theta y(::nav2_msgs::srv::GetCost_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GetCost_Request_theta(msg_);
  }

private:
  ::nav2_msgs::srv::GetCost_Request msg_;
};

class Init_GetCost_Request_x
{
public:
  explicit Init_GetCost_Request_x(::nav2_msgs::srv::GetCost_Request & msg)
  : msg_(msg)
  {}
  Init_GetCost_Request_y x(::nav2_msgs::srv::GetCost_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GetCost_Request_y(msg_);
  }

private:
  ::nav2_msgs::srv::GetCost_Request msg_;
};

class Init_GetCost_Request_use_footprint
{
public:
  Init_GetCost_Request_use_footprint()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetCost_Request_x use_footprint(::nav2_msgs::srv::GetCost_Request::_use_footprint_type arg)
  {
    msg_.use_footprint = std::move(arg);
    return Init_GetCost_Request_x(msg_);
  }

private:
  ::nav2_msgs::srv::GetCost_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::srv::GetCost_Request>()
{
  return nav2_msgs::srv::builder::Init_GetCost_Request_use_footprint();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace srv
{

namespace builder
{

class Init_GetCost_Response_cost
{
public:
  Init_GetCost_Response_cost()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::nav2_msgs::srv::GetCost_Response cost(::nav2_msgs::srv::GetCost_Response::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::srv::GetCost_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::srv::GetCost_Response>()
{
  return nav2_msgs::srv::builder::Init_GetCost_Response_cost();
}

}  // namespace nav2_msgs


namespace nav2_msgs
{

namespace srv
{

namespace builder
{

class Init_GetCost_Event_response
{
public:
  explicit Init_GetCost_Event_response(::nav2_msgs::srv::GetCost_Event & msg)
  : msg_(msg)
  {}
  ::nav2_msgs::srv::GetCost_Event response(::nav2_msgs::srv::GetCost_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nav2_msgs::srv::GetCost_Event msg_;
};

class Init_GetCost_Event_request
{
public:
  explicit Init_GetCost_Event_request(::nav2_msgs::srv::GetCost_Event & msg)
  : msg_(msg)
  {}
  Init_GetCost_Event_response request(::nav2_msgs::srv::GetCost_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetCost_Event_response(msg_);
  }

private:
  ::nav2_msgs::srv::GetCost_Event msg_;
};

class Init_GetCost_Event_info
{
public:
  Init_GetCost_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetCost_Event_request info(::nav2_msgs::srv::GetCost_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetCost_Event_request(msg_);
  }

private:
  ::nav2_msgs::srv::GetCost_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::nav2_msgs::srv::GetCost_Event>()
{
  return nav2_msgs::srv::builder::Init_GetCost_Event_info();
}

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__SRV__DETAIL__GET_COST__BUILDER_HPP_

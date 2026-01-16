// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nav2_msgs:srv/GetCost.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav2_msgs/srv/get_cost.hpp"


#ifndef NAV2_MSGS__SRV__DETAIL__GET_COST__TRAITS_HPP_
#define NAV2_MSGS__SRV__DETAIL__GET_COST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nav2_msgs/srv/detail/get_cost__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace nav2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetCost_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: use_footprint
  {
    out << "use_footprint: ";
    rosidl_generator_traits::value_to_yaml(msg.use_footprint, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetCost_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: use_footprint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_footprint: ";
    rosidl_generator_traits::value_to_yaml(msg.use_footprint, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetCost_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::srv::GetCost_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::srv::GetCost_Request & msg)
{
  return nav2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::srv::GetCost_Request>()
{
  return "nav2_msgs::srv::GetCost_Request";
}

template<>
inline const char * name<nav2_msgs::srv::GetCost_Request>()
{
  return "nav2_msgs/srv/GetCost_Request";
}

template<>
struct has_fixed_size<nav2_msgs::srv::GetCost_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<nav2_msgs::srv::GetCost_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<nav2_msgs::srv::GetCost_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace nav2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetCost_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: cost
  {
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetCost_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cost
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cost: ";
    rosidl_generator_traits::value_to_yaml(msg.cost, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetCost_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::srv::GetCost_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::srv::GetCost_Response & msg)
{
  return nav2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::srv::GetCost_Response>()
{
  return "nav2_msgs::srv::GetCost_Response";
}

template<>
inline const char * name<nav2_msgs::srv::GetCost_Response>()
{
  return "nav2_msgs/srv/GetCost_Response";
}

template<>
struct has_fixed_size<nav2_msgs::srv::GetCost_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<nav2_msgs::srv::GetCost_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<nav2_msgs::srv::GetCost_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace nav2_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetCost_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetCost_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetCost_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace nav2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nav2_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nav2_msgs::srv::GetCost_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  nav2_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nav2_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const nav2_msgs::srv::GetCost_Event & msg)
{
  return nav2_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<nav2_msgs::srv::GetCost_Event>()
{
  return "nav2_msgs::srv::GetCost_Event";
}

template<>
inline const char * name<nav2_msgs::srv::GetCost_Event>()
{
  return "nav2_msgs/srv/GetCost_Event";
}

template<>
struct has_fixed_size<nav2_msgs::srv::GetCost_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<nav2_msgs::srv::GetCost_Event>
  : std::integral_constant<bool, has_bounded_size<nav2_msgs::srv::GetCost_Request>::value && has_bounded_size<nav2_msgs::srv::GetCost_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<nav2_msgs::srv::GetCost_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<nav2_msgs::srv::GetCost>()
{
  return "nav2_msgs::srv::GetCost";
}

template<>
inline const char * name<nav2_msgs::srv::GetCost>()
{
  return "nav2_msgs/srv/GetCost";
}

template<>
struct has_fixed_size<nav2_msgs::srv::GetCost>
  : std::integral_constant<
    bool,
    has_fixed_size<nav2_msgs::srv::GetCost_Request>::value &&
    has_fixed_size<nav2_msgs::srv::GetCost_Response>::value
  >
{
};

template<>
struct has_bounded_size<nav2_msgs::srv::GetCost>
  : std::integral_constant<
    bool,
    has_bounded_size<nav2_msgs::srv::GetCost_Request>::value &&
    has_bounded_size<nav2_msgs::srv::GetCost_Response>::value
  >
{
};

template<>
struct is_service<nav2_msgs::srv::GetCost>
  : std::true_type
{
};

template<>
struct is_service_request<nav2_msgs::srv::GetCost_Request>
  : std::true_type
{
};

template<>
struct is_service_response<nav2_msgs::srv::GetCost_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // NAV2_MSGS__SRV__DETAIL__GET_COST__TRAITS_HPP_

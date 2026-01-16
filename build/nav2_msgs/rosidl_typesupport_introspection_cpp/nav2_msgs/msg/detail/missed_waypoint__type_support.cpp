// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from nav2_msgs:msg/MissedWaypoint.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "nav2_msgs/msg/detail/missed_waypoint__functions.h"
#include "nav2_msgs/msg/detail/missed_waypoint__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace nav2_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MissedWaypoint_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) nav2_msgs::msg::MissedWaypoint(_init);
}

void MissedWaypoint_fini_function(void * message_memory)
{
  auto typed_message = static_cast<nav2_msgs::msg::MissedWaypoint *>(message_memory);
  typed_message->~MissedWaypoint();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MissedWaypoint_message_member_array[3] = {
  {
    "index",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nav2_msgs::msg::MissedWaypoint, index),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "goal",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nav2_msgs::msg::MissedWaypoint, goal),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "error_code",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(nav2_msgs::msg::MissedWaypoint, error_code),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MissedWaypoint_message_members = {
  "nav2_msgs::msg",  // message namespace
  "MissedWaypoint",  // message name
  3,  // number of fields
  sizeof(nav2_msgs::msg::MissedWaypoint),
  false,  // has_any_key_member_
  MissedWaypoint_message_member_array,  // message members
  MissedWaypoint_init_function,  // function to initialize message memory (memory has to be allocated)
  MissedWaypoint_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MissedWaypoint_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MissedWaypoint_message_members,
  get_message_typesupport_handle_function,
  &nav2_msgs__msg__MissedWaypoint__get_type_hash,
  &nav2_msgs__msg__MissedWaypoint__get_type_description,
  &nav2_msgs__msg__MissedWaypoint__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace nav2_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<nav2_msgs::msg::MissedWaypoint>()
{
  return &::nav2_msgs::msg::rosidl_typesupport_introspection_cpp::MissedWaypoint_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, nav2_msgs, msg, MissedWaypoint)() {
  return &::nav2_msgs::msg::rosidl_typesupport_introspection_cpp::MissedWaypoint_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

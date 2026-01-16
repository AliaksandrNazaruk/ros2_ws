// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nav2_msgs:msg/MissedWaypoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav2_msgs/msg/missed_waypoint.h"


#ifndef NAV2_MSGS__MSG__DETAIL__MISSED_WAYPOINT__STRUCT_H_
#define NAV2_MSGS__MSG__DETAIL__MISSED_WAYPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'goal'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in msg/MissedWaypoint in the package nav2_msgs.
typedef struct nav2_msgs__msg__MissedWaypoint
{
  uint32_t index;
  geometry_msgs__msg__PoseStamped goal;
  uint16_t error_code;
} nav2_msgs__msg__MissedWaypoint;

// Struct for a sequence of nav2_msgs__msg__MissedWaypoint.
typedef struct nav2_msgs__msg__MissedWaypoint__Sequence
{
  nav2_msgs__msg__MissedWaypoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nav2_msgs__msg__MissedWaypoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAV2_MSGS__MSG__DETAIL__MISSED_WAYPOINT__STRUCT_H_

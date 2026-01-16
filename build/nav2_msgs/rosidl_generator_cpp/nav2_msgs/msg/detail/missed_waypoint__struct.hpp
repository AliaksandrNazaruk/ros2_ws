// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nav2_msgs:msg/MissedWaypoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nav2_msgs/msg/missed_waypoint.hpp"


#ifndef NAV2_MSGS__MSG__DETAIL__MISSED_WAYPOINT__STRUCT_HPP_
#define NAV2_MSGS__MSG__DETAIL__MISSED_WAYPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'goal'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__nav2_msgs__msg__MissedWaypoint __attribute__((deprecated))
#else
# define DEPRECATED__nav2_msgs__msg__MissedWaypoint __declspec(deprecated)
#endif

namespace nav2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MissedWaypoint_
{
  using Type = MissedWaypoint_<ContainerAllocator>;

  explicit MissedWaypoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0ul;
      this->error_code = 0;
    }
  }

  explicit MissedWaypoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index = 0ul;
      this->error_code = 0;
    }
  }

  // field types and members
  using _index_type =
    uint32_t;
  _index_type index;
  using _goal_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _goal_type goal;
  using _error_code_type =
    uint16_t;
  _error_code_type error_code;

  // setters for named parameter idiom
  Type & set__index(
    const uint32_t & _arg)
  {
    this->index = _arg;
    return *this;
  }
  Type & set__goal(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint16_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nav2_msgs::msg::MissedWaypoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const nav2_msgs::msg::MissedWaypoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nav2_msgs::msg::MissedWaypoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nav2_msgs::msg::MissedWaypoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::msg::MissedWaypoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::msg::MissedWaypoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nav2_msgs::msg::MissedWaypoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nav2_msgs::msg::MissedWaypoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nav2_msgs::msg::MissedWaypoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nav2_msgs::msg::MissedWaypoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nav2_msgs__msg__MissedWaypoint
    std::shared_ptr<nav2_msgs::msg::MissedWaypoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nav2_msgs__msg__MissedWaypoint
    std::shared_ptr<nav2_msgs::msg::MissedWaypoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MissedWaypoint_ & other) const
  {
    if (this->index != other.index) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    return true;
  }
  bool operator!=(const MissedWaypoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MissedWaypoint_

// alias to use template instance with default allocator
using MissedWaypoint =
  nav2_msgs::msg::MissedWaypoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nav2_msgs

#endif  // NAV2_MSGS__MSG__DETAIL__MISSED_WAYPOINT__STRUCT_HPP_

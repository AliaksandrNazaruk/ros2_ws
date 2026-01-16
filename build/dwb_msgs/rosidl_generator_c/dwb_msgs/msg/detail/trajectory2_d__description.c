// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dwb_msgs:msg/Trajectory2D.idl
// generated code does not contain a copyright notice

#include "dwb_msgs/msg/detail/trajectory2_d__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dwb_msgs
const rosidl_type_hash_t *
dwb_msgs__msg__Trajectory2D__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5a, 0x27, 0x6e, 0x6e, 0x04, 0xdb, 0x51, 0x77,
      0xd4, 0x9f, 0x26, 0x72, 0xaa, 0x6d, 0x88, 0x3a,
      0x6c, 0x5d, 0x99, 0x4c, 0xfe, 0xd3, 0x1c, 0x94,
      0xdb, 0xa2, 0xd6, 0x69, 0x70, 0xba, 0x8f, 0x4a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "nav_2d_msgs/msg/detail/twist2_d__functions.h"
#include "builtin_interfaces/msg/detail/duration__functions.h"
#include "geometry_msgs/msg/detail/pose2_d__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Duration__EXPECTED_HASH = {1, {
    0xe8, 0xd0, 0x09, 0xf6, 0x59, 0x81, 0x6f, 0x75,
    0x8b, 0x75, 0x33, 0x4e, 0xe1, 0xa9, 0xca, 0x5b,
    0x5c, 0x0b, 0x85, 0x98, 0x43, 0x26, 0x1f, 0x14,
    0xc7, 0xf9, 0x37, 0x34, 0x95, 0x99, 0xd9, 0x3b,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Pose2D__EXPECTED_HASH = {1, {
    0xd6, 0x8e, 0xfa, 0x5b, 0x46, 0xe7, 0x0f, 0x7b,
    0x16, 0xca, 0x23, 0x08, 0x54, 0x74, 0xfd, 0xac,
    0x5a, 0x44, 0xb6, 0x38, 0x78, 0x3e, 0xc4, 0x2f,
    0x66, 0x1d, 0xa6, 0x4d, 0xa4, 0x72, 0x4c, 0xcc,
  }};
static const rosidl_type_hash_t nav_2d_msgs__msg__Twist2D__EXPECTED_HASH = {1, {
    0x33, 0xa3, 0xa4, 0x7d, 0x7b, 0x3a, 0xaa, 0xd5,
    0x37, 0x6a, 0x48, 0x25, 0x05, 0x97, 0x09, 0x3f,
    0xeb, 0x4b, 0x92, 0xab, 0x87, 0x06, 0xc9, 0x6d,
    0xca, 0x69, 0xf4, 0x7d, 0x9a, 0x98, 0x07, 0xd0,
  }};
#endif

static char dwb_msgs__msg__Trajectory2D__TYPE_NAME[] = "dwb_msgs/msg/Trajectory2D";
static char builtin_interfaces__msg__Duration__TYPE_NAME[] = "builtin_interfaces/msg/Duration";
static char geometry_msgs__msg__Pose2D__TYPE_NAME[] = "geometry_msgs/msg/Pose2D";
static char nav_2d_msgs__msg__Twist2D__TYPE_NAME[] = "nav_2d_msgs/msg/Twist2D";

// Define type names, field names, and default values
static char dwb_msgs__msg__Trajectory2D__FIELD_NAME__velocity[] = "velocity";
static char dwb_msgs__msg__Trajectory2D__FIELD_NAME__time_offsets[] = "time_offsets";
static char dwb_msgs__msg__Trajectory2D__FIELD_NAME__poses[] = "poses";

static rosidl_runtime_c__type_description__Field dwb_msgs__msg__Trajectory2D__FIELDS[] = {
  {
    {dwb_msgs__msg__Trajectory2D__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nav_2d_msgs__msg__Twist2D__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__msg__Trajectory2D__FIELD_NAME__time_offsets, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__msg__Trajectory2D__FIELD_NAME__poses, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {geometry_msgs__msg__Pose2D__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dwb_msgs__msg__Trajectory2D__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose2D__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {nav_2d_msgs__msg__Twist2D__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dwb_msgs__msg__Trajectory2D__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dwb_msgs__msg__Trajectory2D__TYPE_NAME, 25, 25},
      {dwb_msgs__msg__Trajectory2D__FIELDS, 3, 3},
    },
    {dwb_msgs__msg__Trajectory2D__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Duration__EXPECTED_HASH, builtin_interfaces__msg__Duration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Duration__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose2D__EXPECTED_HASH, geometry_msgs__msg__Pose2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = geometry_msgs__msg__Pose2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Twist2D__EXPECTED_HASH, nav_2d_msgs__msg__Twist2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = nav_2d_msgs__msg__Twist2D__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# For a given velocity command, the poses that the robot will go to in the allotted time.\n"
  "\n"
  "# Input Velocity\n"
  "nav_2d_msgs/Twist2D velocity\n"
  "# Time difference between first and last poses\n"
  "builtin_interfaces/Duration[] time_offsets\n"
  "# Poses the robot will go to, given our kinematic model\n"
  "geometry_msgs/Pose2D[] poses";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dwb_msgs__msg__Trajectory2D__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dwb_msgs__msg__Trajectory2D__TYPE_NAME, 25, 25},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 312, 312},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dwb_msgs__msg__Trajectory2D__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dwb_msgs__msg__Trajectory2D__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *geometry_msgs__msg__Pose2D__get_individual_type_description_source(NULL);
    sources[3] = *nav_2d_msgs__msg__Twist2D__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

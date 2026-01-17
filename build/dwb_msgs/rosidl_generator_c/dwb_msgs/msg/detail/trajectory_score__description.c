// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dwb_msgs:msg/TrajectoryScore.idl
// generated code does not contain a copyright notice

#include "dwb_msgs/msg/detail/trajectory_score__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dwb_msgs
const rosidl_type_hash_t *
dwb_msgs__msg__TrajectoryScore__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x80, 0xbd, 0x18, 0x29, 0xf8, 0x87, 0x2d, 0x38,
      0xc2, 0x2b, 0x97, 0x6c, 0x3a, 0x2d, 0x60, 0x40,
      0xeb, 0xd1, 0x16, 0x2b, 0x3a, 0x6a, 0x4b, 0xf6,
      0x49, 0xa0, 0x8d, 0x79, 0x59, 0xf8, 0x44, 0x65,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "geometry_msgs/msg/detail/pose2_d__functions.h"
#include "dwb_msgs/msg/detail/critic_score__functions.h"
#include "builtin_interfaces/msg/detail/duration__functions.h"
#include "nav_2d_msgs/msg/detail/twist2_d__functions.h"
#include "dwb_msgs/msg/detail/trajectory2_d__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Duration__EXPECTED_HASH = {1, {
    0xe8, 0xd0, 0x09, 0xf6, 0x59, 0x81, 0x6f, 0x75,
    0x8b, 0x75, 0x33, 0x4e, 0xe1, 0xa9, 0xca, 0x5b,
    0x5c, 0x0b, 0x85, 0x98, 0x43, 0x26, 0x1f, 0x14,
    0xc7, 0xf9, 0x37, 0x34, 0x95, 0x99, 0xd9, 0x3b,
  }};
static const rosidl_type_hash_t dwb_msgs__msg__CriticScore__EXPECTED_HASH = {1, {
    0x4d, 0xfb, 0x08, 0x3c, 0x8d, 0x33, 0xfd, 0xe7,
    0xd1, 0xe1, 0x48, 0x23, 0x81, 0x50, 0x7f, 0xf3,
    0x33, 0xb8, 0x62, 0x33, 0x23, 0x88, 0x80, 0xa8,
    0x96, 0x82, 0x10, 0x69, 0x2c, 0xdd, 0xab, 0x6f,
  }};
static const rosidl_type_hash_t dwb_msgs__msg__Trajectory2D__EXPECTED_HASH = {1, {
    0x5a, 0x27, 0x6e, 0x6e, 0x04, 0xdb, 0x51, 0x77,
    0xd4, 0x9f, 0x26, 0x72, 0xaa, 0x6d, 0x88, 0x3a,
    0x6c, 0x5d, 0x99, 0x4c, 0xfe, 0xd3, 0x1c, 0x94,
    0xdb, 0xa2, 0xd6, 0x69, 0x70, 0xba, 0x8f, 0x4a,
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

static char dwb_msgs__msg__TrajectoryScore__TYPE_NAME[] = "dwb_msgs/msg/TrajectoryScore";
static char builtin_interfaces__msg__Duration__TYPE_NAME[] = "builtin_interfaces/msg/Duration";
static char dwb_msgs__msg__CriticScore__TYPE_NAME[] = "dwb_msgs/msg/CriticScore";
static char dwb_msgs__msg__Trajectory2D__TYPE_NAME[] = "dwb_msgs/msg/Trajectory2D";
static char geometry_msgs__msg__Pose2D__TYPE_NAME[] = "geometry_msgs/msg/Pose2D";
static char nav_2d_msgs__msg__Twist2D__TYPE_NAME[] = "nav_2d_msgs/msg/Twist2D";

// Define type names, field names, and default values
static char dwb_msgs__msg__TrajectoryScore__FIELD_NAME__traj[] = "traj";
static char dwb_msgs__msg__TrajectoryScore__FIELD_NAME__scores[] = "scores";
static char dwb_msgs__msg__TrajectoryScore__FIELD_NAME__total[] = "total";

static rosidl_runtime_c__type_description__Field dwb_msgs__msg__TrajectoryScore__FIELDS[] = {
  {
    {dwb_msgs__msg__TrajectoryScore__FIELD_NAME__traj, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dwb_msgs__msg__Trajectory2D__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__msg__TrajectoryScore__FIELD_NAME__scores, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {dwb_msgs__msg__CriticScore__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__msg__TrajectoryScore__FIELD_NAME__total, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dwb_msgs__msg__TrajectoryScore__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__msg__CriticScore__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__msg__Trajectory2D__TYPE_NAME, 25, 25},
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
dwb_msgs__msg__TrajectoryScore__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dwb_msgs__msg__TrajectoryScore__TYPE_NAME, 28, 28},
      {dwb_msgs__msg__TrajectoryScore__FIELDS, 3, 3},
    },
    {dwb_msgs__msg__TrajectoryScore__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Duration__EXPECTED_HASH, builtin_interfaces__msg__Duration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Duration__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&dwb_msgs__msg__CriticScore__EXPECTED_HASH, dwb_msgs__msg__CriticScore__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = dwb_msgs__msg__CriticScore__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&dwb_msgs__msg__Trajectory2D__EXPECTED_HASH, dwb_msgs__msg__Trajectory2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = dwb_msgs__msg__Trajectory2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose2D__EXPECTED_HASH, geometry_msgs__msg__Pose2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Pose2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Twist2D__EXPECTED_HASH, nav_2d_msgs__msg__Twist2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = nav_2d_msgs__msg__Twist2D__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Complete scoring for a given twist.\n"
  "\n"
  "# The trajectory being scored\n"
  "Trajectory2D traj\n"
  "# The Scores for each of the critics employed\n"
  "CriticScore[] scores\n"
  "# Convenience member that totals the critic scores\n"
  "float32 total";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dwb_msgs__msg__TrajectoryScore__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dwb_msgs__msg__TrajectoryScore__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 219, 219},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dwb_msgs__msg__TrajectoryScore__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dwb_msgs__msg__TrajectoryScore__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *dwb_msgs__msg__CriticScore__get_individual_type_description_source(NULL);
    sources[3] = *dwb_msgs__msg__Trajectory2D__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Pose2D__get_individual_type_description_source(NULL);
    sources[5] = *nav_2d_msgs__msg__Twist2D__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

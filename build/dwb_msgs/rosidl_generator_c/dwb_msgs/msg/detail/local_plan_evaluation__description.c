// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dwb_msgs:msg/LocalPlanEvaluation.idl
// generated code does not contain a copyright notice

#include "dwb_msgs/msg/detail/local_plan_evaluation__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dwb_msgs
const rosidl_type_hash_t *
dwb_msgs__msg__LocalPlanEvaluation__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6b, 0xb7, 0xaf, 0x7f, 0x92, 0x59, 0x1f, 0xcb,
      0x4e, 0x63, 0x7d, 0xd0, 0x80, 0x3c, 0xa5, 0x9a,
      0x0b, 0x31, 0x25, 0xa0, 0x96, 0x2e, 0xeb, 0x99,
      0x19, 0xb5, 0x5c, 0xd6, 0x8d, 0xb3, 0xe9, 0x7b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "geometry_msgs/msg/detail/pose2_d__functions.h"
#include "dwb_msgs/msg/detail/critic_score__functions.h"
#include "builtin_interfaces/msg/detail/duration__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "nav_2d_msgs/msg/detail/twist2_d__functions.h"
#include "dwb_msgs/msg/detail/trajectory2_d__functions.h"
#include "dwb_msgs/msg/detail/trajectory_score__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Duration__EXPECTED_HASH = {1, {
    0xe8, 0xd0, 0x09, 0xf6, 0x59, 0x81, 0x6f, 0x75,
    0x8b, 0x75, 0x33, 0x4e, 0xe1, 0xa9, 0xca, 0x5b,
    0x5c, 0x0b, 0x85, 0x98, 0x43, 0x26, 0x1f, 0x14,
    0xc7, 0xf9, 0x37, 0x34, 0x95, 0x99, 0xd9, 0x3b,
  }};
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
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
static const rosidl_type_hash_t dwb_msgs__msg__TrajectoryScore__EXPECTED_HASH = {1, {
    0x80, 0xbd, 0x18, 0x29, 0xf8, 0x87, 0x2d, 0x38,
    0xc2, 0x2b, 0x97, 0x6c, 0x3a, 0x2d, 0x60, 0x40,
    0xeb, 0xd1, 0x16, 0x2b, 0x3a, 0x6a, 0x4b, 0xf6,
    0x49, 0xa0, 0x8d, 0x79, 0x59, 0xf8, 0x44, 0x65,
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
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char dwb_msgs__msg__LocalPlanEvaluation__TYPE_NAME[] = "dwb_msgs/msg/LocalPlanEvaluation";
static char builtin_interfaces__msg__Duration__TYPE_NAME[] = "builtin_interfaces/msg/Duration";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char dwb_msgs__msg__CriticScore__TYPE_NAME[] = "dwb_msgs/msg/CriticScore";
static char dwb_msgs__msg__Trajectory2D__TYPE_NAME[] = "dwb_msgs/msg/Trajectory2D";
static char dwb_msgs__msg__TrajectoryScore__TYPE_NAME[] = "dwb_msgs/msg/TrajectoryScore";
static char geometry_msgs__msg__Pose2D__TYPE_NAME[] = "geometry_msgs/msg/Pose2D";
static char nav_2d_msgs__msg__Twist2D__TYPE_NAME[] = "nav_2d_msgs/msg/Twist2D";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char dwb_msgs__msg__LocalPlanEvaluation__FIELD_NAME__header[] = "header";
static char dwb_msgs__msg__LocalPlanEvaluation__FIELD_NAME__twists[] = "twists";
static char dwb_msgs__msg__LocalPlanEvaluation__FIELD_NAME__best_index[] = "best_index";
static char dwb_msgs__msg__LocalPlanEvaluation__FIELD_NAME__worst_index[] = "worst_index";

static rosidl_runtime_c__type_description__Field dwb_msgs__msg__LocalPlanEvaluation__FIELDS[] = {
  {
    {dwb_msgs__msg__LocalPlanEvaluation__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__msg__LocalPlanEvaluation__FIELD_NAME__twists, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {dwb_msgs__msg__TrajectoryScore__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__msg__LocalPlanEvaluation__FIELD_NAME__best_index, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__msg__LocalPlanEvaluation__FIELD_NAME__worst_index, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dwb_msgs__msg__LocalPlanEvaluation__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
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
    {dwb_msgs__msg__TrajectoryScore__TYPE_NAME, 28, 28},
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
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dwb_msgs__msg__LocalPlanEvaluation__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dwb_msgs__msg__LocalPlanEvaluation__TYPE_NAME, 32, 32},
      {dwb_msgs__msg__LocalPlanEvaluation__FIELDS, 4, 4},
    },
    {dwb_msgs__msg__LocalPlanEvaluation__REFERENCED_TYPE_DESCRIPTIONS, 8, 8},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Duration__EXPECTED_HASH, builtin_interfaces__msg__Duration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Duration__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&dwb_msgs__msg__CriticScore__EXPECTED_HASH, dwb_msgs__msg__CriticScore__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = dwb_msgs__msg__CriticScore__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&dwb_msgs__msg__Trajectory2D__EXPECTED_HASH, dwb_msgs__msg__Trajectory2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = dwb_msgs__msg__Trajectory2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&dwb_msgs__msg__TrajectoryScore__EXPECTED_HASH, dwb_msgs__msg__TrajectoryScore__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = dwb_msgs__msg__TrajectoryScore__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose2D__EXPECTED_HASH, geometry_msgs__msg__Pose2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Pose2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Twist2D__EXPECTED_HASH, nav_2d_msgs__msg__Twist2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = nav_2d_msgs__msg__Twist2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Full Scoring for running the local planner\n"
  "\n"
  "# Header, used for timestamp\n"
  "std_msgs/Header header\n"
  "# All the trajectories evaluated and their scores\n"
  "TrajectoryScore[] twists\n"
  "# Convenience index of the best (lowest) score in the twists array\n"
  "uint16 best_index\n"
  "# Convenience index of the worst (highest) score in the twists array. Useful for scaling.\n"
  "uint16 worst_index";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dwb_msgs__msg__LocalPlanEvaluation__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dwb_msgs__msg__LocalPlanEvaluation__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 367, 367},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dwb_msgs__msg__LocalPlanEvaluation__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[9];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 9, 9};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dwb_msgs__msg__LocalPlanEvaluation__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *dwb_msgs__msg__CriticScore__get_individual_type_description_source(NULL);
    sources[4] = *dwb_msgs__msg__Trajectory2D__get_individual_type_description_source(NULL);
    sources[5] = *dwb_msgs__msg__TrajectoryScore__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Pose2D__get_individual_type_description_source(NULL);
    sources[7] = *nav_2d_msgs__msg__Twist2D__get_individual_type_description_source(NULL);
    sources[8] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

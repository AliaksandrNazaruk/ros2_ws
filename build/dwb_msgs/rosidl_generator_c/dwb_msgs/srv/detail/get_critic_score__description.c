// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from dwb_msgs:srv/GetCriticScore.idl
// generated code does not contain a copyright notice

#include "dwb_msgs/srv/detail/get_critic_score__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_dwb_msgs
const rosidl_type_hash_t *
dwb_msgs__srv__GetCriticScore__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x35, 0xc5, 0x4b, 0x4f, 0x70, 0xe9, 0x21, 0x36,
      0x37, 0x42, 0xca, 0x68, 0xfa, 0x34, 0xf6, 0x98,
      0xfd, 0xd7, 0x34, 0x70, 0xbe, 0x3d, 0xae, 0xca,
      0x33, 0x1c, 0x32, 0xca, 0x3e, 0xef, 0x98, 0xdc,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dwb_msgs
const rosidl_type_hash_t *
dwb_msgs__srv__GetCriticScore_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x68, 0x8a, 0x7f, 0x50, 0xbc, 0xff, 0x51, 0xd0,
      0x35, 0xdd, 0x3f, 0xb9, 0x85, 0x18, 0xe8, 0x58,
      0x6e, 0xc9, 0xb8, 0x01, 0xc7, 0xa3, 0xc7, 0xd4,
      0x5c, 0x77, 0x39, 0xc9, 0xb6, 0x4c, 0x89, 0x02,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dwb_msgs
const rosidl_type_hash_t *
dwb_msgs__srv__GetCriticScore_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xec, 0x8e, 0x53, 0xec, 0x07, 0x6b, 0x94, 0x3c,
      0x42, 0x08, 0x9b, 0x3b, 0x8a, 0xbe, 0x97, 0x2b,
      0xf4, 0x49, 0x2c, 0x7c, 0xb5, 0x23, 0x6a, 0x3f,
      0xbb, 0x0f, 0xa3, 0x84, 0x98, 0x52, 0x21, 0x82,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_dwb_msgs
const rosidl_type_hash_t *
dwb_msgs__srv__GetCriticScore_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcd, 0x98, 0xc3, 0x75, 0xec, 0xad, 0xe1, 0xb5,
      0x16, 0x69, 0xd4, 0x11, 0xaa, 0xf0, 0xb0, 0x34,
      0x54, 0x7b, 0xea, 0x11, 0x11, 0xc3, 0x1f, 0xe5,
      0x73, 0x61, 0xdc, 0x93, 0x6e, 0x5c, 0x2c, 0xbd,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "nav_2d_msgs/msg/detail/path2_d__functions.h"
#include "geometry_msgs/msg/detail/pose2_d__functions.h"
#include "dwb_msgs/msg/detail/critic_score__functions.h"
#include "builtin_interfaces/msg/detail/duration__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "nav_2d_msgs/msg/detail/twist2_d__functions.h"
#include "dwb_msgs/msg/detail/trajectory2_d__functions.h"
#include "nav_2d_msgs/msg/detail/pose2_d_stamped__functions.h"

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
static const rosidl_type_hash_t geometry_msgs__msg__Pose2D__EXPECTED_HASH = {1, {
    0xd6, 0x8e, 0xfa, 0x5b, 0x46, 0xe7, 0x0f, 0x7b,
    0x16, 0xca, 0x23, 0x08, 0x54, 0x74, 0xfd, 0xac,
    0x5a, 0x44, 0xb6, 0x38, 0x78, 0x3e, 0xc4, 0x2f,
    0x66, 0x1d, 0xa6, 0x4d, 0xa4, 0x72, 0x4c, 0xcc,
  }};
static const rosidl_type_hash_t nav_2d_msgs__msg__Path2D__EXPECTED_HASH = {1, {
    0xfb, 0xf4, 0x2e, 0xfc, 0x1f, 0xea, 0xfb, 0x28,
    0xb0, 0xbe, 0xd4, 0x2b, 0xaf, 0xcc, 0xce, 0x7b,
    0xd4, 0x10, 0xbd, 0x10, 0xa7, 0x35, 0x4d, 0x29,
    0x8f, 0x36, 0xe1, 0x0f, 0x9c, 0x34, 0xa7, 0xcf,
  }};
static const rosidl_type_hash_t nav_2d_msgs__msg__Pose2DStamped__EXPECTED_HASH = {1, {
    0x06, 0xb7, 0x49, 0xe5, 0xc4, 0x8a, 0xa6, 0x34,
    0x42, 0x89, 0x62, 0x24, 0x7e, 0xef, 0xf9, 0xde,
    0x8d, 0x83, 0x93, 0xb2, 0x18, 0x3d, 0xaf, 0xa9,
    0x67, 0x42, 0x17, 0x48, 0x71, 0xbb, 0xb3, 0x2e,
  }};
static const rosidl_type_hash_t nav_2d_msgs__msg__Twist2D__EXPECTED_HASH = {1, {
    0x33, 0xa3, 0xa4, 0x7d, 0x7b, 0x3a, 0xaa, 0xd5,
    0x37, 0x6a, 0x48, 0x25, 0x05, 0x97, 0x09, 0x3f,
    0xeb, 0x4b, 0x92, 0xab, 0x87, 0x06, 0xc9, 0x6d,
    0xca, 0x69, 0xf4, 0x7d, 0x9a, 0x98, 0x07, 0xd0,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char dwb_msgs__srv__GetCriticScore__TYPE_NAME[] = "dwb_msgs/srv/GetCriticScore";
static char builtin_interfaces__msg__Duration__TYPE_NAME[] = "builtin_interfaces/msg/Duration";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char dwb_msgs__msg__CriticScore__TYPE_NAME[] = "dwb_msgs/msg/CriticScore";
static char dwb_msgs__msg__Trajectory2D__TYPE_NAME[] = "dwb_msgs/msg/Trajectory2D";
static char dwb_msgs__srv__GetCriticScore_Event__TYPE_NAME[] = "dwb_msgs/srv/GetCriticScore_Event";
static char dwb_msgs__srv__GetCriticScore_Request__TYPE_NAME[] = "dwb_msgs/srv/GetCriticScore_Request";
static char dwb_msgs__srv__GetCriticScore_Response__TYPE_NAME[] = "dwb_msgs/srv/GetCriticScore_Response";
static char geometry_msgs__msg__Pose2D__TYPE_NAME[] = "geometry_msgs/msg/Pose2D";
static char nav_2d_msgs__msg__Path2D__TYPE_NAME[] = "nav_2d_msgs/msg/Path2D";
static char nav_2d_msgs__msg__Pose2DStamped__TYPE_NAME[] = "nav_2d_msgs/msg/Pose2DStamped";
static char nav_2d_msgs__msg__Twist2D__TYPE_NAME[] = "nav_2d_msgs/msg/Twist2D";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char dwb_msgs__srv__GetCriticScore__FIELD_NAME__request_message[] = "request_message";
static char dwb_msgs__srv__GetCriticScore__FIELD_NAME__response_message[] = "response_message";
static char dwb_msgs__srv__GetCriticScore__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field dwb_msgs__srv__GetCriticScore__FIELDS[] = {
  {
    {dwb_msgs__srv__GetCriticScore__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dwb_msgs__srv__GetCriticScore_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dwb_msgs__srv__GetCriticScore_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dwb_msgs__srv__GetCriticScore_Event__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dwb_msgs__srv__GetCriticScore__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
    {dwb_msgs__srv__GetCriticScore_Event__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose2D__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {nav_2d_msgs__msg__Path2D__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {nav_2d_msgs__msg__Pose2DStamped__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {nav_2d_msgs__msg__Twist2D__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dwb_msgs__srv__GetCriticScore__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dwb_msgs__srv__GetCriticScore__TYPE_NAME, 27, 27},
      {dwb_msgs__srv__GetCriticScore__FIELDS, 3, 3},
    },
    {dwb_msgs__srv__GetCriticScore__REFERENCED_TYPE_DESCRIPTIONS, 13, 13},
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
    description.referenced_type_descriptions.data[4].fields = dwb_msgs__srv__GetCriticScore_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = dwb_msgs__srv__GetCriticScore_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[6].fields = dwb_msgs__srv__GetCriticScore_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose2D__EXPECTED_HASH, geometry_msgs__msg__Pose2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = geometry_msgs__msg__Pose2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Path2D__EXPECTED_HASH, nav_2d_msgs__msg__Path2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = nav_2d_msgs__msg__Path2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Pose2DStamped__EXPECTED_HASH, nav_2d_msgs__msg__Pose2DStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = nav_2d_msgs__msg__Pose2DStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Twist2D__EXPECTED_HASH, nav_2d_msgs__msg__Twist2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[10].fields = nav_2d_msgs__msg__Twist2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[11].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[12].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dwb_msgs__srv__GetCriticScore_Request__FIELD_NAME__pose[] = "pose";
static char dwb_msgs__srv__GetCriticScore_Request__FIELD_NAME__velocity[] = "velocity";
static char dwb_msgs__srv__GetCriticScore_Request__FIELD_NAME__global_plan[] = "global_plan";
static char dwb_msgs__srv__GetCriticScore_Request__FIELD_NAME__traj[] = "traj";
static char dwb_msgs__srv__GetCriticScore_Request__FIELD_NAME__critic_name[] = "critic_name";

static rosidl_runtime_c__type_description__Field dwb_msgs__srv__GetCriticScore_Request__FIELDS[] = {
  {
    {dwb_msgs__srv__GetCriticScore_Request__FIELD_NAME__pose, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nav_2d_msgs__msg__Pose2DStamped__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore_Request__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nav_2d_msgs__msg__Twist2D__TYPE_NAME, 23, 23},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore_Request__FIELD_NAME__global_plan, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nav_2d_msgs__msg__Path2D__TYPE_NAME, 22, 22},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore_Request__FIELD_NAME__traj, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dwb_msgs__msg__Trajectory2D__TYPE_NAME, 25, 25},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore_Request__FIELD_NAME__critic_name, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dwb_msgs__srv__GetCriticScore_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Duration__TYPE_NAME, 31, 31},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
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
    {nav_2d_msgs__msg__Path2D__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {nav_2d_msgs__msg__Pose2DStamped__TYPE_NAME, 29, 29},
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
dwb_msgs__srv__GetCriticScore_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dwb_msgs__srv__GetCriticScore_Request__TYPE_NAME, 35, 35},
      {dwb_msgs__srv__GetCriticScore_Request__FIELDS, 5, 5},
    },
    {dwb_msgs__srv__GetCriticScore_Request__REFERENCED_TYPE_DESCRIPTIONS, 8, 8},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Duration__EXPECTED_HASH, builtin_interfaces__msg__Duration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Duration__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&dwb_msgs__msg__Trajectory2D__EXPECTED_HASH, dwb_msgs__msg__Trajectory2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = dwb_msgs__msg__Trajectory2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose2D__EXPECTED_HASH, geometry_msgs__msg__Pose2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Pose2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Path2D__EXPECTED_HASH, nav_2d_msgs__msg__Path2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = nav_2d_msgs__msg__Path2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Pose2DStamped__EXPECTED_HASH, nav_2d_msgs__msg__Pose2DStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = nav_2d_msgs__msg__Pose2DStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Twist2D__EXPECTED_HASH, nav_2d_msgs__msg__Twist2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = nav_2d_msgs__msg__Twist2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dwb_msgs__srv__GetCriticScore_Response__FIELD_NAME__score[] = "score";

static rosidl_runtime_c__type_description__Field dwb_msgs__srv__GetCriticScore_Response__FIELDS[] = {
  {
    {dwb_msgs__srv__GetCriticScore_Response__FIELD_NAME__score, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {dwb_msgs__msg__CriticScore__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dwb_msgs__srv__GetCriticScore_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {dwb_msgs__msg__CriticScore__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dwb_msgs__srv__GetCriticScore_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dwb_msgs__srv__GetCriticScore_Response__TYPE_NAME, 36, 36},
      {dwb_msgs__srv__GetCriticScore_Response__FIELDS, 1, 1},
    },
    {dwb_msgs__srv__GetCriticScore_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&dwb_msgs__msg__CriticScore__EXPECTED_HASH, dwb_msgs__msg__CriticScore__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = dwb_msgs__msg__CriticScore__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char dwb_msgs__srv__GetCriticScore_Event__FIELD_NAME__info[] = "info";
static char dwb_msgs__srv__GetCriticScore_Event__FIELD_NAME__request[] = "request";
static char dwb_msgs__srv__GetCriticScore_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field dwb_msgs__srv__GetCriticScore_Event__FIELDS[] = {
  {
    {dwb_msgs__srv__GetCriticScore_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {dwb_msgs__srv__GetCriticScore_Request__TYPE_NAME, 35, 35},
    },
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {dwb_msgs__srv__GetCriticScore_Response__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription dwb_msgs__srv__GetCriticScore_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
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
    {dwb_msgs__srv__GetCriticScore_Request__TYPE_NAME, 35, 35},
    {NULL, 0, 0},
  },
  {
    {dwb_msgs__srv__GetCriticScore_Response__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose2D__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {nav_2d_msgs__msg__Path2D__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {nav_2d_msgs__msg__Pose2DStamped__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {nav_2d_msgs__msg__Twist2D__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
dwb_msgs__srv__GetCriticScore_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {dwb_msgs__srv__GetCriticScore_Event__TYPE_NAME, 33, 33},
      {dwb_msgs__srv__GetCriticScore_Event__FIELDS, 3, 3},
    },
    {dwb_msgs__srv__GetCriticScore_Event__REFERENCED_TYPE_DESCRIPTIONS, 12, 12},
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
    description.referenced_type_descriptions.data[4].fields = dwb_msgs__srv__GetCriticScore_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = dwb_msgs__srv__GetCriticScore_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose2D__EXPECTED_HASH, geometry_msgs__msg__Pose2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = geometry_msgs__msg__Pose2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Path2D__EXPECTED_HASH, nav_2d_msgs__msg__Path2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = nav_2d_msgs__msg__Path2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Pose2DStamped__EXPECTED_HASH, nav_2d_msgs__msg__Pose2DStamped__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = nav_2d_msgs__msg__Pose2DStamped__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nav_2d_msgs__msg__Twist2D__EXPECTED_HASH, nav_2d_msgs__msg__Twist2D__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = nav_2d_msgs__msg__Twist2D__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[10].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[11].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "nav_2d_msgs/Pose2DStamped pose\n"
  "nav_2d_msgs/Twist2D velocity\n"
  "nav_2d_msgs/Path2D global_plan\n"
  "Trajectory2D traj\n"
  "string critic_name\n"
  "---\n"
  "CriticScore score";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
dwb_msgs__srv__GetCriticScore__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dwb_msgs__srv__GetCriticScore__TYPE_NAME, 27, 27},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 150, 150},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dwb_msgs__srv__GetCriticScore_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dwb_msgs__srv__GetCriticScore_Request__TYPE_NAME, 35, 35},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dwb_msgs__srv__GetCriticScore_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dwb_msgs__srv__GetCriticScore_Response__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
dwb_msgs__srv__GetCriticScore_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {dwb_msgs__srv__GetCriticScore_Event__TYPE_NAME, 33, 33},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dwb_msgs__srv__GetCriticScore__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[14];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 14, 14};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dwb_msgs__srv__GetCriticScore__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *dwb_msgs__msg__CriticScore__get_individual_type_description_source(NULL);
    sources[4] = *dwb_msgs__msg__Trajectory2D__get_individual_type_description_source(NULL);
    sources[5] = *dwb_msgs__srv__GetCriticScore_Event__get_individual_type_description_source(NULL);
    sources[6] = *dwb_msgs__srv__GetCriticScore_Request__get_individual_type_description_source(NULL);
    sources[7] = *dwb_msgs__srv__GetCriticScore_Response__get_individual_type_description_source(NULL);
    sources[8] = *geometry_msgs__msg__Pose2D__get_individual_type_description_source(NULL);
    sources[9] = *nav_2d_msgs__msg__Path2D__get_individual_type_description_source(NULL);
    sources[10] = *nav_2d_msgs__msg__Pose2DStamped__get_individual_type_description_source(NULL);
    sources[11] = *nav_2d_msgs__msg__Twist2D__get_individual_type_description_source(NULL);
    sources[12] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[13] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dwb_msgs__srv__GetCriticScore_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[9];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 9, 9};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dwb_msgs__srv__GetCriticScore_Request__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *dwb_msgs__msg__Trajectory2D__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Pose2D__get_individual_type_description_source(NULL);
    sources[5] = *nav_2d_msgs__msg__Path2D__get_individual_type_description_source(NULL);
    sources[6] = *nav_2d_msgs__msg__Pose2DStamped__get_individual_type_description_source(NULL);
    sources[7] = *nav_2d_msgs__msg__Twist2D__get_individual_type_description_source(NULL);
    sources[8] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dwb_msgs__srv__GetCriticScore_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dwb_msgs__srv__GetCriticScore_Response__get_individual_type_description_source(NULL),
    sources[1] = *dwb_msgs__msg__CriticScore__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
dwb_msgs__srv__GetCriticScore_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[13];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 13, 13};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *dwb_msgs__srv__GetCriticScore_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Duration__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *dwb_msgs__msg__CriticScore__get_individual_type_description_source(NULL);
    sources[4] = *dwb_msgs__msg__Trajectory2D__get_individual_type_description_source(NULL);
    sources[5] = *dwb_msgs__srv__GetCriticScore_Request__get_individual_type_description_source(NULL);
    sources[6] = *dwb_msgs__srv__GetCriticScore_Response__get_individual_type_description_source(NULL);
    sources[7] = *geometry_msgs__msg__Pose2D__get_individual_type_description_source(NULL);
    sources[8] = *nav_2d_msgs__msg__Path2D__get_individual_type_description_source(NULL);
    sources[9] = *nav_2d_msgs__msg__Pose2DStamped__get_individual_type_description_source(NULL);
    sources[10] = *nav_2d_msgs__msg__Twist2D__get_individual_type_description_source(NULL);
    sources[11] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[12] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

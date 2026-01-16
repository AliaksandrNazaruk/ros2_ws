// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from nav2_msgs:srv/GetCost.idl
// generated code does not contain a copyright notice

#include "nav2_msgs/srv/detail/get_cost__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_nav2_msgs
const rosidl_type_hash_t *
nav2_msgs__srv__GetCost__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbe, 0x1b, 0xfe, 0x25, 0x18, 0x0a, 0x91, 0x5c,
      0x2e, 0xb1, 0x84, 0x17, 0x2e, 0xcc, 0xf9, 0xea,
      0x00, 0x42, 0x07, 0x32, 0xe5, 0xf2, 0xc3, 0xf9,
      0xe6, 0xd6, 0x07, 0x54, 0xc4, 0x1f, 0x49, 0x9a,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_nav2_msgs
const rosidl_type_hash_t *
nav2_msgs__srv__GetCost_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x86, 0x15, 0x8e, 0x1b, 0x66, 0x92, 0xed, 0x88,
      0x8f, 0x20, 0x39, 0xf5, 0xec, 0xc7, 0xc9, 0x14,
      0xb3, 0x48, 0x47, 0x15, 0xe1, 0xfd, 0xc4, 0x5f,
      0x4e, 0xe7, 0xc3, 0xb4, 0x67, 0x52, 0x22, 0x93,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_nav2_msgs
const rosidl_type_hash_t *
nav2_msgs__srv__GetCost_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3f, 0xa6, 0xcb, 0xc5, 0x21, 0x20, 0x3b, 0x88,
      0xbf, 0x45, 0x5a, 0x70, 0x4d, 0x4d, 0x94, 0x3e,
      0xc0, 0x25, 0xe2, 0xbb, 0x0b, 0xe5, 0xcf, 0x72,
      0xdf, 0x7e, 0x4e, 0xdf, 0x40, 0xa1, 0xf7, 0x90,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_nav2_msgs
const rosidl_type_hash_t *
nav2_msgs__srv__GetCost_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdf, 0x2b, 0xf1, 0x8a, 0xbb, 0xb5, 0x04, 0x9b,
      0xaa, 0xf4, 0x44, 0x4c, 0x79, 0xa4, 0xbf, 0x12,
      0x8e, 0xec, 0x50, 0x3d, 0x61, 0x3f, 0xdd, 0x5a,
      0xf5, 0x3e, 0xe5, 0xc4, 0xf2, 0xcb, 0xfb, 0x6f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char nav2_msgs__srv__GetCost__TYPE_NAME[] = "nav2_msgs/srv/GetCost";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char nav2_msgs__srv__GetCost_Event__TYPE_NAME[] = "nav2_msgs/srv/GetCost_Event";
static char nav2_msgs__srv__GetCost_Request__TYPE_NAME[] = "nav2_msgs/srv/GetCost_Request";
static char nav2_msgs__srv__GetCost_Response__TYPE_NAME[] = "nav2_msgs/srv/GetCost_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char nav2_msgs__srv__GetCost__FIELD_NAME__request_message[] = "request_message";
static char nav2_msgs__srv__GetCost__FIELD_NAME__response_message[] = "response_message";
static char nav2_msgs__srv__GetCost__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field nav2_msgs__srv__GetCost__FIELDS[] = {
  {
    {nav2_msgs__srv__GetCost__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nav2_msgs__srv__GetCost_Request__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nav2_msgs__srv__GetCost_Response__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nav2_msgs__srv__GetCost_Event__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription nav2_msgs__srv__GetCost__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost_Event__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost_Request__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost_Response__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nav2_msgs__srv__GetCost__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nav2_msgs__srv__GetCost__TYPE_NAME, 21, 21},
      {nav2_msgs__srv__GetCost__FIELDS, 3, 3},
    },
    {nav2_msgs__srv__GetCost__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = nav2_msgs__srv__GetCost_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = nav2_msgs__srv__GetCost_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = nav2_msgs__srv__GetCost_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char nav2_msgs__srv__GetCost_Request__FIELD_NAME__use_footprint[] = "use_footprint";
static char nav2_msgs__srv__GetCost_Request__FIELD_NAME__x[] = "x";
static char nav2_msgs__srv__GetCost_Request__FIELD_NAME__y[] = "y";
static char nav2_msgs__srv__GetCost_Request__FIELD_NAME__theta[] = "theta";

static rosidl_runtime_c__type_description__Field nav2_msgs__srv__GetCost_Request__FIELDS[] = {
  {
    {nav2_msgs__srv__GetCost_Request__FIELD_NAME__use_footprint, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost_Request__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost_Request__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost_Request__FIELD_NAME__theta, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nav2_msgs__srv__GetCost_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nav2_msgs__srv__GetCost_Request__TYPE_NAME, 29, 29},
      {nav2_msgs__srv__GetCost_Request__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char nav2_msgs__srv__GetCost_Response__FIELD_NAME__cost[] = "cost";

static rosidl_runtime_c__type_description__Field nav2_msgs__srv__GetCost_Response__FIELDS[] = {
  {
    {nav2_msgs__srv__GetCost_Response__FIELD_NAME__cost, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nav2_msgs__srv__GetCost_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nav2_msgs__srv__GetCost_Response__TYPE_NAME, 30, 30},
      {nav2_msgs__srv__GetCost_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char nav2_msgs__srv__GetCost_Event__FIELD_NAME__info[] = "info";
static char nav2_msgs__srv__GetCost_Event__FIELD_NAME__request[] = "request";
static char nav2_msgs__srv__GetCost_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field nav2_msgs__srv__GetCost_Event__FIELDS[] = {
  {
    {nav2_msgs__srv__GetCost_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {nav2_msgs__srv__GetCost_Request__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {nav2_msgs__srv__GetCost_Response__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription nav2_msgs__srv__GetCost_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost_Request__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
  {
    {nav2_msgs__srv__GetCost_Response__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nav2_msgs__srv__GetCost_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nav2_msgs__srv__GetCost_Event__TYPE_NAME, 27, 27},
      {nav2_msgs__srv__GetCost_Event__FIELDS, 3, 3},
    },
    {nav2_msgs__srv__GetCost_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = nav2_msgs__srv__GetCost_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = nav2_msgs__srv__GetCost_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Get costmap cost at given point\n"
  "\n"
  "bool use_footprint\n"
  "float32 x\n"
  "float32 y\n"
  "float32 theta\n"
  "---\n"
  "float32 cost";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
nav2_msgs__srv__GetCost__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nav2_msgs__srv__GetCost__TYPE_NAME, 21, 21},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 104, 104},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
nav2_msgs__srv__GetCost_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nav2_msgs__srv__GetCost_Request__TYPE_NAME, 29, 29},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
nav2_msgs__srv__GetCost_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nav2_msgs__srv__GetCost_Response__TYPE_NAME, 30, 30},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
nav2_msgs__srv__GetCost_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nav2_msgs__srv__GetCost_Event__TYPE_NAME, 27, 27},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nav2_msgs__srv__GetCost__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nav2_msgs__srv__GetCost__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *nav2_msgs__srv__GetCost_Event__get_individual_type_description_source(NULL);
    sources[3] = *nav2_msgs__srv__GetCost_Request__get_individual_type_description_source(NULL);
    sources[4] = *nav2_msgs__srv__GetCost_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nav2_msgs__srv__GetCost_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nav2_msgs__srv__GetCost_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nav2_msgs__srv__GetCost_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nav2_msgs__srv__GetCost_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nav2_msgs__srv__GetCost_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nav2_msgs__srv__GetCost_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *nav2_msgs__srv__GetCost_Request__get_individual_type_description_source(NULL);
    sources[3] = *nav2_msgs__srv__GetCost_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

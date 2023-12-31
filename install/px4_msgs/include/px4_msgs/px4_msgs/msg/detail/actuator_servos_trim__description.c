// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ActuatorServosTrim.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/actuator_servos_trim__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ActuatorServosTrim__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x88, 0x90, 0x20, 0x9a, 0x65, 0xae, 0x4d, 0xc9,
      0xe2, 0xf3, 0xe9, 0x97, 0xeb, 0x77, 0x96, 0xb5,
      0xaa, 0x10, 0xf9, 0xb6, 0xf4, 0x95, 0x13, 0x1e,
      0xd7, 0xfb, 0x66, 0xa6, 0xee, 0xc0, 0xf0, 0x57,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ActuatorServosTrim__TYPE_NAME[] = "px4_msgs/msg/ActuatorServosTrim";

// Define type names, field names, and default values
static char px4_msgs__msg__ActuatorServosTrim__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ActuatorServosTrim__FIELD_NAME__trim[] = "trim";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ActuatorServosTrim__FIELDS[] = {
  {
    {px4_msgs__msg__ActuatorServosTrim__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActuatorServosTrim__FIELD_NAME__trim, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      8,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__ActuatorServosTrim__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ActuatorServosTrim__TYPE_NAME, 31, 31},
      {px4_msgs__msg__ActuatorServosTrim__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Servo trims, added as offset to servo outputs\n"
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint8 NUM_CONTROLS = 8\n"
  "float32[8] trim    # range: [-1, 1]";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ActuatorServosTrim__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ActuatorServosTrim__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 168, 168},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ActuatorServosTrim__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ActuatorServosTrim__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

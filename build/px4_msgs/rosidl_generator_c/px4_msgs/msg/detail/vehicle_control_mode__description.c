// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleControlMode.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_control_mode__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleControlMode__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x53, 0xf6, 0xa6, 0xa7, 0xe1, 0x70, 0x72, 0x4c,
      0xa2, 0x8a, 0x1b, 0xdd, 0x0e, 0xcd, 0x22, 0xf2,
      0x0d, 0x10, 0x3d, 0x64, 0xda, 0x4a, 0xed, 0x01,
      0xb3, 0x8e, 0x94, 0xb6, 0x31, 0x79, 0xbb, 0x00,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleControlMode__TYPE_NAME[] = "px4_msgs/msg/VehicleControlMode";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_armed[] = "flag_armed";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_multicopter_position_control_enabled[] = "flag_multicopter_position_control_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_manual_enabled[] = "flag_control_manual_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_auto_enabled[] = "flag_control_auto_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_offboard_enabled[] = "flag_control_offboard_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_rates_enabled[] = "flag_control_rates_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_attitude_enabled[] = "flag_control_attitude_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_acceleration_enabled[] = "flag_control_acceleration_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_velocity_enabled[] = "flag_control_velocity_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_position_enabled[] = "flag_control_position_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_altitude_enabled[] = "flag_control_altitude_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_climb_rate_enabled[] = "flag_control_climb_rate_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_termination_enabled[] = "flag_control_termination_enabled";
static char px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_allocation_enabled[] = "flag_control_allocation_enabled";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleControlMode__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_armed, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_multicopter_position_control_enabled, 41, 41},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_manual_enabled, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_auto_enabled, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_offboard_enabled, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_rates_enabled, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_attitude_enabled, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_acceleration_enabled, 33, 33},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_velocity_enabled, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_position_enabled, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_altitude_enabled, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_climb_rate_enabled, 31, 31},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_termination_enabled, 32, 32},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleControlMode__FIELD_NAME__flag_control_allocation_enabled, 31, 31},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__VehicleControlMode__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleControlMode__TYPE_NAME, 31, 31},
      {px4_msgs__msg__VehicleControlMode__FIELDS, 15, 15},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "bool flag_armed\\t\\t\\t# synonym for actuator_armed.armed\n"
  "\n"
  "bool flag_multicopter_position_control_enabled\n"
  "\n"
  "bool flag_control_manual_enabled\\t\\t# true if manual input is mixed in\n"
  "bool flag_control_auto_enabled\\t\\t\\t# true if onboard autopilot should act\n"
  "bool flag_control_offboard_enabled\\t\\t# true if offboard control should be used\n"
  "bool flag_control_rates_enabled\\t\\t\\t# true if rates are stabilized\n"
  "bool flag_control_attitude_enabled\\t\\t# true if attitude stabilization is mixed in\n"
  "bool flag_control_acceleration_enabled\\t\\t# true if acceleration is controlled\n"
  "bool flag_control_velocity_enabled\\t\\t# true if horizontal velocity (implies direction) is controlled\n"
  "bool flag_control_position_enabled\\t\\t# true if position is controlled\n"
  "bool flag_control_altitude_enabled\\t\\t# true if altitude is controlled\n"
  "bool flag_control_climb_rate_enabled\\t\\t# true if climb rate is controlled\n"
  "bool flag_control_termination_enabled\\t\\t# true if flighttermination is enabled\n"
  "bool flag_control_allocation_enabled\\t\\t# true if control allocation is enabled";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleControlMode__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleControlMode__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1070, 1070},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleControlMode__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleControlMode__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

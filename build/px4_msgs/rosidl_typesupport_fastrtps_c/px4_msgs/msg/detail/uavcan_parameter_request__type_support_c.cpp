// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/UavcanParameterRequest.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/uavcan_parameter_request__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/uavcan_parameter_request__struct.h"
#include "px4_msgs/msg/detail/uavcan_parameter_request__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _UavcanParameterRequest__ros_msg_type = px4_msgs__msg__UavcanParameterRequest;

static bool _UavcanParameterRequest__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UavcanParameterRequest__ros_msg_type * ros_message = static_cast<const _UavcanParameterRequest__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: message_type
  {
    cdr << ros_message->message_type;
  }

  // Field name: node_id
  {
    cdr << ros_message->node_id;
  }

  // Field name: param_id
  {
    size_t size = 17;
    auto array_ptr = ros_message->param_id;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: param_index
  {
    cdr << ros_message->param_index;
  }

  // Field name: param_type
  {
    cdr << ros_message->param_type;
  }

  // Field name: int_value
  {
    cdr << ros_message->int_value;
  }

  // Field name: real_value
  {
    cdr << ros_message->real_value;
  }

  return true;
}

static bool _UavcanParameterRequest__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UavcanParameterRequest__ros_msg_type * ros_message = static_cast<_UavcanParameterRequest__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: message_type
  {
    cdr >> ros_message->message_type;
  }

  // Field name: node_id
  {
    cdr >> ros_message->node_id;
  }

  // Field name: param_id
  {
    size_t size = 17;
    auto array_ptr = ros_message->param_id;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: param_index
  {
    cdr >> ros_message->param_index;
  }

  // Field name: param_type
  {
    cdr >> ros_message->param_type;
  }

  // Field name: int_value
  {
    cdr >> ros_message->int_value;
  }

  // Field name: real_value
  {
    cdr >> ros_message->real_value;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__UavcanParameterRequest(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UavcanParameterRequest__ros_msg_type * ros_message = static_cast<const _UavcanParameterRequest__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name message_type
  {
    size_t item_size = sizeof(ros_message->message_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name node_id
  {
    size_t item_size = sizeof(ros_message->node_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name param_id
  {
    size_t array_size = 17;
    auto array_ptr = ros_message->param_id;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name param_index
  {
    size_t item_size = sizeof(ros_message->param_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name param_type
  {
    size_t item_size = sizeof(ros_message->param_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name int_value
  {
    size_t item_size = sizeof(ros_message->int_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name real_value
  {
    size_t item_size = sizeof(ros_message->real_value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UavcanParameterRequest__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__UavcanParameterRequest(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__UavcanParameterRequest(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: message_type
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: node_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: param_id
  {
    size_t array_size = 17;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: param_index
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: param_type
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: int_value
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: real_value
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _UavcanParameterRequest__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__UavcanParameterRequest(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UavcanParameterRequest = {
  "px4_msgs::msg",
  "UavcanParameterRequest",
  _UavcanParameterRequest__cdr_serialize,
  _UavcanParameterRequest__cdr_deserialize,
  _UavcanParameterRequest__get_serialized_size,
  _UavcanParameterRequest__max_serialized_size
};

static rosidl_message_type_support_t _UavcanParameterRequest__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UavcanParameterRequest,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__UavcanParameterRequest__get_type_hash,
  &px4_msgs__msg__UavcanParameterRequest__get_type_description,
  &px4_msgs__msg__UavcanParameterRequest__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, UavcanParameterRequest)() {
  return &_UavcanParameterRequest__type_support;
}

#if defined(__cplusplus)
}
#endif
// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quadrotor_msgs:msg/PolyTraj.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quadrotor_msgs/msg/detail/poly_traj__rosidl_typesupport_introspection_c.h"
#include "quadrotor_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quadrotor_msgs/msg/detail/poly_traj__functions.h"
#include "quadrotor_msgs/msg/detail/poly_traj__struct.h"


// Include directives for member types
// Member `start_time`
#include "builtin_interfaces/msg/time.h"
// Member `start_time`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `hover_p`
// Member `coef_x`
// Member `coef_y`
// Member `coef_z`
// Member `duration`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quadrotor_msgs__msg__PolyTraj__init(message_memory);
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_fini_function(void * message_memory)
{
  quadrotor_msgs__msg__PolyTraj__fini(message_memory);
}

size_t quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__size_function__PolyTraj__hover_p(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__hover_p(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__hover_p(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__fetch_function__PolyTraj__hover_p(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__hover_p(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__assign_function__PolyTraj__hover_p(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__hover_p(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__resize_function__PolyTraj__hover_p(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__size_function__PolyTraj__coef_x(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__coef_x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__coef_x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__fetch_function__PolyTraj__coef_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__coef_x(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__assign_function__PolyTraj__coef_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__coef_x(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__resize_function__PolyTraj__coef_x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__size_function__PolyTraj__coef_y(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__coef_y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__coef_y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__fetch_function__PolyTraj__coef_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__coef_y(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__assign_function__PolyTraj__coef_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__coef_y(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__resize_function__PolyTraj__coef_y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__size_function__PolyTraj__coef_z(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__coef_z(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__coef_z(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__fetch_function__PolyTraj__coef_z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__coef_z(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__assign_function__PolyTraj__coef_z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__coef_z(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__resize_function__PolyTraj__coef_z(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__size_function__PolyTraj__duration(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__duration(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__duration(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__fetch_function__PolyTraj__duration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__duration(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__assign_function__PolyTraj__duration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__duration(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__resize_function__PolyTraj__duration(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_message_member_array[11] = {
  {
    "drone_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, drone_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "traj_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, traj_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, start_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hover",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, hover),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hover_p",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, hover_p),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__size_function__PolyTraj__hover_p,  // size() function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__hover_p,  // get_const(index) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__hover_p,  // get(index) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__fetch_function__PolyTraj__hover_p,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__assign_function__PolyTraj__hover_p,  // assign(index, value) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__resize_function__PolyTraj__hover_p  // resize(index) function pointer
  },
  {
    "order",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, order),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "coef_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, coef_x),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__size_function__PolyTraj__coef_x,  // size() function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__coef_x,  // get_const(index) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__coef_x,  // get(index) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__fetch_function__PolyTraj__coef_x,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__assign_function__PolyTraj__coef_x,  // assign(index, value) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__resize_function__PolyTraj__coef_x  // resize(index) function pointer
  },
  {
    "coef_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, coef_y),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__size_function__PolyTraj__coef_y,  // size() function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__coef_y,  // get_const(index) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__coef_y,  // get(index) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__fetch_function__PolyTraj__coef_y,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__assign_function__PolyTraj__coef_y,  // assign(index, value) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__resize_function__PolyTraj__coef_y  // resize(index) function pointer
  },
  {
    "coef_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, coef_z),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__size_function__PolyTraj__coef_z,  // size() function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__coef_z,  // get_const(index) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__coef_z,  // get(index) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__fetch_function__PolyTraj__coef_z,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__assign_function__PolyTraj__coef_z,  // assign(index, value) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__resize_function__PolyTraj__coef_z  // resize(index) function pointer
  },
  {
    "duration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolyTraj, duration),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__size_function__PolyTraj__duration,  // size() function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_const_function__PolyTraj__duration,  // get_const(index) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__get_function__PolyTraj__duration,  // get(index) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__fetch_function__PolyTraj__duration,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__assign_function__PolyTraj__duration,  // assign(index, value) function pointer
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__resize_function__PolyTraj__duration  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_message_members = {
  "quadrotor_msgs__msg",  // message namespace
  "PolyTraj",  // message name
  11,  // number of fields
  sizeof(quadrotor_msgs__msg__PolyTraj),
  quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_message_member_array,  // message members
  quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_init_function,  // function to initialize message memory (memory has to be allocated)
  quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_message_type_support_handle = {
  0,
  &quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quadrotor_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quadrotor_msgs, msg, PolyTraj)() {
  quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_message_type_support_handle.typesupport_identifier) {
    quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quadrotor_msgs__msg__PolyTraj__rosidl_typesupport_introspection_c__PolyTraj_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

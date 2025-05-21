// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from decomp_ros_msgs:msg/PolyhedronArray.idl
// generated code does not contain a copyright notice

#ifndef DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__STRUCT_H_
#define DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'polyhedrons'
#include "decomp_ros_msgs/msg/detail/polyhedron__struct.h"

/// Struct defined in msg/PolyhedronArray in the package decomp_ros_msgs.
typedef struct decomp_ros_msgs__msg__PolyhedronArray
{
  std_msgs__msg__Header header;
  decomp_ros_msgs__msg__Polyhedron__Sequence polyhedrons;
} decomp_ros_msgs__msg__PolyhedronArray;

// Struct for a sequence of decomp_ros_msgs__msg__PolyhedronArray.
typedef struct decomp_ros_msgs__msg__PolyhedronArray__Sequence
{
  decomp_ros_msgs__msg__PolyhedronArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} decomp_ros_msgs__msg__PolyhedronArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DECOMP_ROS_MSGS__MSG__DETAIL__POLYHEDRON_ARRAY__STRUCT_H_

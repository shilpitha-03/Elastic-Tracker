// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from quadrotor_msgs:msg/Px4ctrlDebug.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__FUNCTIONS_H_
#define QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "quadrotor_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "quadrotor_msgs/msg/detail/px4ctrl_debug__struct.h"

/// Initialize msg/Px4ctrlDebug message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * quadrotor_msgs__msg__Px4ctrlDebug
 * )) before or use
 * quadrotor_msgs__msg__Px4ctrlDebug__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__Px4ctrlDebug__init(quadrotor_msgs__msg__Px4ctrlDebug * msg);

/// Finalize msg/Px4ctrlDebug message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
void
quadrotor_msgs__msg__Px4ctrlDebug__fini(quadrotor_msgs__msg__Px4ctrlDebug * msg);

/// Create msg/Px4ctrlDebug message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * quadrotor_msgs__msg__Px4ctrlDebug__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
quadrotor_msgs__msg__Px4ctrlDebug *
quadrotor_msgs__msg__Px4ctrlDebug__create();

/// Destroy msg/Px4ctrlDebug message.
/**
 * It calls
 * quadrotor_msgs__msg__Px4ctrlDebug__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
void
quadrotor_msgs__msg__Px4ctrlDebug__destroy(quadrotor_msgs__msg__Px4ctrlDebug * msg);

/// Check for msg/Px4ctrlDebug message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__Px4ctrlDebug__are_equal(const quadrotor_msgs__msg__Px4ctrlDebug * lhs, const quadrotor_msgs__msg__Px4ctrlDebug * rhs);

/// Copy a msg/Px4ctrlDebug message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__Px4ctrlDebug__copy(
  const quadrotor_msgs__msg__Px4ctrlDebug * input,
  quadrotor_msgs__msg__Px4ctrlDebug * output);

/// Initialize array of msg/Px4ctrlDebug messages.
/**
 * It allocates the memory for the number of elements and calls
 * quadrotor_msgs__msg__Px4ctrlDebug__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__init(quadrotor_msgs__msg__Px4ctrlDebug__Sequence * array, size_t size);

/// Finalize array of msg/Px4ctrlDebug messages.
/**
 * It calls
 * quadrotor_msgs__msg__Px4ctrlDebug__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
void
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__fini(quadrotor_msgs__msg__Px4ctrlDebug__Sequence * array);

/// Create array of msg/Px4ctrlDebug messages.
/**
 * It allocates the memory for the array and calls
 * quadrotor_msgs__msg__Px4ctrlDebug__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
quadrotor_msgs__msg__Px4ctrlDebug__Sequence *
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__create(size_t size);

/// Destroy array of msg/Px4ctrlDebug messages.
/**
 * It calls
 * quadrotor_msgs__msg__Px4ctrlDebug__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
void
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__destroy(quadrotor_msgs__msg__Px4ctrlDebug__Sequence * array);

/// Check for msg/Px4ctrlDebug message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__are_equal(const quadrotor_msgs__msg__Px4ctrlDebug__Sequence * lhs, const quadrotor_msgs__msg__Px4ctrlDebug__Sequence * rhs);

/// Copy an array of msg/Px4ctrlDebug messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_quadrotor_msgs
bool
quadrotor_msgs__msg__Px4ctrlDebug__Sequence__copy(
  const quadrotor_msgs__msg__Px4ctrlDebug__Sequence * input,
  quadrotor_msgs__msg__Px4ctrlDebug__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__PX4CTRL_DEBUG__FUNCTIONS_H_

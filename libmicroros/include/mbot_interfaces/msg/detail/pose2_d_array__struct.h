// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbot_interfaces:msg/Pose2DArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbot_interfaces/msg/pose2_d_array.h"


#ifndef MBOT_INTERFACES__MSG__DETAIL__POSE2_D_ARRAY__STRUCT_H_
#define MBOT_INTERFACES__MSG__DETAIL__POSE2_D_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'poses'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"

/// Struct defined in msg/Pose2DArray in the package mbot_interfaces.
typedef struct mbot_interfaces__msg__Pose2DArray
{
  geometry_msgs__msg__Pose2D__Sequence poses;
} mbot_interfaces__msg__Pose2DArray;

// Struct for a sequence of mbot_interfaces__msg__Pose2DArray.
typedef struct mbot_interfaces__msg__Pose2DArray__Sequence
{
  mbot_interfaces__msg__Pose2DArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbot_interfaces__msg__Pose2DArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBOT_INTERFACES__MSG__DETAIL__POSE2_D_ARRAY__STRUCT_H_

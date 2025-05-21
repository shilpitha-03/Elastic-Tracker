// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from quadrotor_msgs:msg/AuxCommand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_HPP_
#define QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__quadrotor_msgs__msg__AuxCommand __attribute__((deprecated))
#else
# define DEPRECATED__quadrotor_msgs__msg__AuxCommand __declspec(deprecated)
#endif

namespace quadrotor_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AuxCommand_
{
  using Type = AuxCommand_<ContainerAllocator>;

  explicit AuxCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_yaw = 0.0;
      this->kf_correction = 0.0;
      std::fill<typename std::array<double, 2>::iterator, double>(this->angle_corrections.begin(), this->angle_corrections.end(), 0.0);
      this->enable_motors = false;
      this->use_external_yaw = false;
    }
  }

  explicit AuxCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : angle_corrections(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->current_yaw = 0.0;
      this->kf_correction = 0.0;
      std::fill<typename std::array<double, 2>::iterator, double>(this->angle_corrections.begin(), this->angle_corrections.end(), 0.0);
      this->enable_motors = false;
      this->use_external_yaw = false;
    }
  }

  // field types and members
  using _current_yaw_type =
    double;
  _current_yaw_type current_yaw;
  using _kf_correction_type =
    double;
  _kf_correction_type kf_correction;
  using _angle_corrections_type =
    std::array<double, 2>;
  _angle_corrections_type angle_corrections;
  using _enable_motors_type =
    bool;
  _enable_motors_type enable_motors;
  using _use_external_yaw_type =
    bool;
  _use_external_yaw_type use_external_yaw;

  // setters for named parameter idiom
  Type & set__current_yaw(
    const double & _arg)
  {
    this->current_yaw = _arg;
    return *this;
  }
  Type & set__kf_correction(
    const double & _arg)
  {
    this->kf_correction = _arg;
    return *this;
  }
  Type & set__angle_corrections(
    const std::array<double, 2> & _arg)
  {
    this->angle_corrections = _arg;
    return *this;
  }
  Type & set__enable_motors(
    const bool & _arg)
  {
    this->enable_motors = _arg;
    return *this;
  }
  Type & set__use_external_yaw(
    const bool & _arg)
  {
    this->use_external_yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    quadrotor_msgs::msg::AuxCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const quadrotor_msgs::msg::AuxCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::AuxCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<quadrotor_msgs::msg::AuxCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::AuxCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::AuxCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      quadrotor_msgs::msg::AuxCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<quadrotor_msgs::msg::AuxCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::AuxCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<quadrotor_msgs::msg::AuxCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__quadrotor_msgs__msg__AuxCommand
    std::shared_ptr<quadrotor_msgs::msg::AuxCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__quadrotor_msgs__msg__AuxCommand
    std::shared_ptr<quadrotor_msgs::msg::AuxCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AuxCommand_ & other) const
  {
    if (this->current_yaw != other.current_yaw) {
      return false;
    }
    if (this->kf_correction != other.kf_correction) {
      return false;
    }
    if (this->angle_corrections != other.angle_corrections) {
      return false;
    }
    if (this->enable_motors != other.enable_motors) {
      return false;
    }
    if (this->use_external_yaw != other.use_external_yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const AuxCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AuxCommand_

// alias to use template instance with default allocator
using AuxCommand =
  quadrotor_msgs::msg::AuxCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace quadrotor_msgs

#endif  // QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_HPP_

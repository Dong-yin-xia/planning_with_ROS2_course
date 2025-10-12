// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:msg/LocalTrajectoryPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__TRAITS_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/msg/detail/local_trajectory_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'path_point'
#include "base_msgs/msg/detail/local_path_point__traits.hpp"
// Member 'speed_point'
#include "base_msgs/msg/detail/local_speeds_point__traits.hpp"

namespace base_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LocalTrajectoryPoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: path_point
  {
    out << "path_point: ";
    to_flow_style_yaml(msg.path_point, out);
    out << ", ";
  }

  // member: speed_point
  {
    out << "speed_point: ";
    to_flow_style_yaml(msg.speed_point, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LocalTrajectoryPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: path_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path_point:\n";
    to_block_style_yaml(msg.path_point, out, indentation + 2);
  }

  // member: speed_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_point:\n";
    to_block_style_yaml(msg.speed_point, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LocalTrajectoryPoint & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace base_msgs

namespace rosidl_generator_traits
{

[[deprecated("use base_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const base_msgs::msg::LocalTrajectoryPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::msg::LocalTrajectoryPoint & msg)
{
  return base_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::msg::LocalTrajectoryPoint>()
{
  return "base_msgs::msg::LocalTrajectoryPoint";
}

template<>
inline const char * name<base_msgs::msg::LocalTrajectoryPoint>()
{
  return "base_msgs/msg/LocalTrajectoryPoint";
}

template<>
struct has_fixed_size<base_msgs::msg::LocalTrajectoryPoint>
  : std::integral_constant<bool, has_fixed_size<base_msgs::msg::LocalPathPoint>::value && has_fixed_size<base_msgs::msg::LocalSpeedsPoint>::value> {};

template<>
struct has_bounded_size<base_msgs::msg::LocalTrajectoryPoint>
  : std::integral_constant<bool, has_bounded_size<base_msgs::msg::LocalPathPoint>::value && has_bounded_size<base_msgs::msg::LocalSpeedsPoint>::value> {};

template<>
struct is_message<base_msgs::msg::LocalTrajectoryPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__TRAITS_HPP_

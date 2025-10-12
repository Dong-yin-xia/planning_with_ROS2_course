// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:msg/ReferlinePoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__TRAITS_HPP_
#define BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/msg/detail/referline_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace base_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ReferlinePoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: rs
  {
    out << "rs: ";
    rosidl_generator_traits::value_to_yaml(msg.rs, out);
    out << ", ";
  }

  // member: rtheta
  {
    out << "rtheta: ";
    rosidl_generator_traits::value_to_yaml(msg.rtheta, out);
    out << ", ";
  }

  // member: rkappa
  {
    out << "rkappa: ";
    rosidl_generator_traits::value_to_yaml(msg.rkappa, out);
    out << ", ";
  }

  // member: rdkappa
  {
    out << "rdkappa: ";
    rosidl_generator_traits::value_to_yaml(msg.rdkappa, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ReferlinePoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: rs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rs: ";
    rosidl_generator_traits::value_to_yaml(msg.rs, out);
    out << "\n";
  }

  // member: rtheta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rtheta: ";
    rosidl_generator_traits::value_to_yaml(msg.rtheta, out);
    out << "\n";
  }

  // member: rkappa
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rkappa: ";
    rosidl_generator_traits::value_to_yaml(msg.rkappa, out);
    out << "\n";
  }

  // member: rdkappa
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rdkappa: ";
    rosidl_generator_traits::value_to_yaml(msg.rdkappa, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ReferlinePoint & msg, bool use_flow_style = false)
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
  const base_msgs::msg::ReferlinePoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::msg::ReferlinePoint & msg)
{
  return base_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::msg::ReferlinePoint>()
{
  return "base_msgs::msg::ReferlinePoint";
}

template<>
inline const char * name<base_msgs::msg::ReferlinePoint>()
{
  return "base_msgs/msg/ReferlinePoint";
}

template<>
struct has_fixed_size<base_msgs::msg::ReferlinePoint>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<base_msgs::msg::ReferlinePoint>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<base_msgs::msg::ReferlinePoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__TRAITS_HPP_

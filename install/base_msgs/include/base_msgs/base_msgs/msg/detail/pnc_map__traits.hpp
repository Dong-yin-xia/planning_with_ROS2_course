// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:msg/PNCMap.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__PNC_MAP__TRAITS_HPP_
#define BASE_MSGS__MSG__DETAIL__PNC_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/msg/detail/pnc_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'left_boundary'
// Member 'right_boundary'
// Member 'midline'
#include "visualization_msgs/msg/detail/marker__traits.hpp"

namespace base_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PNCMap & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: road_length
  {
    out << "road_length: ";
    rosidl_generator_traits::value_to_yaml(msg.road_length, out);
    out << ", ";
  }

  // member: road_half_width
  {
    out << "road_half_width: ";
    rosidl_generator_traits::value_to_yaml(msg.road_half_width, out);
    out << ", ";
  }

  // member: left_boundary
  {
    out << "left_boundary: ";
    to_flow_style_yaml(msg.left_boundary, out);
    out << ", ";
  }

  // member: right_boundary
  {
    out << "right_boundary: ";
    to_flow_style_yaml(msg.right_boundary, out);
    out << ", ";
  }

  // member: midline
  {
    out << "midline: ";
    to_flow_style_yaml(msg.midline, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PNCMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: road_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "road_length: ";
    rosidl_generator_traits::value_to_yaml(msg.road_length, out);
    out << "\n";
  }

  // member: road_half_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "road_half_width: ";
    rosidl_generator_traits::value_to_yaml(msg.road_half_width, out);
    out << "\n";
  }

  // member: left_boundary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_boundary:\n";
    to_block_style_yaml(msg.left_boundary, out, indentation + 2);
  }

  // member: right_boundary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_boundary:\n";
    to_block_style_yaml(msg.right_boundary, out, indentation + 2);
  }

  // member: midline
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "midline:\n";
    to_block_style_yaml(msg.midline, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PNCMap & msg, bool use_flow_style = false)
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
  const base_msgs::msg::PNCMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::msg::PNCMap & msg)
{
  return base_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::msg::PNCMap>()
{
  return "base_msgs::msg::PNCMap";
}

template<>
inline const char * name<base_msgs::msg::PNCMap>()
{
  return "base_msgs/msg/PNCMap";
}

template<>
struct has_fixed_size<base_msgs::msg::PNCMap>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<visualization_msgs::msg::Marker>::value> {};

template<>
struct has_bounded_size<base_msgs::msg::PNCMap>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<visualization_msgs::msg::Marker>::value> {};

template<>
struct is_message<base_msgs::msg::PNCMap>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__MSG__DETAIL__PNC_MAP__TRAITS_HPP_

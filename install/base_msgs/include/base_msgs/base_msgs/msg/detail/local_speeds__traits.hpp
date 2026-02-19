// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:msg/LocalSpeeds.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS__TRAITS_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/msg/detail/local_speeds__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'local_speeds'
#include "base_msgs/msg/detail/local_speeds_point__traits.hpp"

namespace base_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LocalSpeeds & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: local_speeds
  {
    if (msg.local_speeds.size() == 0) {
      out << "local_speeds: []";
    } else {
      out << "local_speeds: [";
      size_t pending_items = msg.local_speeds.size();
      for (auto item : msg.local_speeds) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LocalSpeeds & msg,
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

  // member: local_speeds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.local_speeds.size() == 0) {
      out << "local_speeds: []\n";
    } else {
      out << "local_speeds:\n";
      for (auto item : msg.local_speeds) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LocalSpeeds & msg, bool use_flow_style = false)
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
  const base_msgs::msg::LocalSpeeds & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::msg::LocalSpeeds & msg)
{
  return base_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::msg::LocalSpeeds>()
{
  return "base_msgs::msg::LocalSpeeds";
}

template<>
inline const char * name<base_msgs::msg::LocalSpeeds>()
{
  return "base_msgs/msg/LocalSpeeds";
}

template<>
struct has_fixed_size<base_msgs::msg::LocalSpeeds>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<base_msgs::msg::LocalSpeeds>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<base_msgs::msg::LocalSpeeds>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS__TRAITS_HPP_

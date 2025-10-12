// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:msg/LocalPath.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_PATH__TRAITS_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_PATH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/msg/detail/local_path__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'local_path'
#include "base_msgs/msg/detail/local_path_point__traits.hpp"

namespace base_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LocalPath & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: local_path
  {
    if (msg.local_path.size() == 0) {
      out << "local_path: []";
    } else {
      out << "local_path: [";
      size_t pending_items = msg.local_path.size();
      for (auto item : msg.local_path) {
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
  const LocalPath & msg,
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

  // member: local_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.local_path.size() == 0) {
      out << "local_path: []\n";
    } else {
      out << "local_path:\n";
      for (auto item : msg.local_path) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LocalPath & msg, bool use_flow_style = false)
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
  const base_msgs::msg::LocalPath & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::msg::LocalPath & msg)
{
  return base_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::msg::LocalPath>()
{
  return "base_msgs::msg::LocalPath";
}

template<>
inline const char * name<base_msgs::msg::LocalPath>()
{
  return "base_msgs/msg/LocalPath";
}

template<>
struct has_fixed_size<base_msgs::msg::LocalPath>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<base_msgs::msg::LocalPath>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<base_msgs::msg::LocalPath>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_PATH__TRAITS_HPP_

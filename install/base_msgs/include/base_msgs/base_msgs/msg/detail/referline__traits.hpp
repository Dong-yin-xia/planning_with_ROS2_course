// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:msg/Referline.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__REFERLINE__TRAITS_HPP_
#define BASE_MSGS__MSG__DETAIL__REFERLINE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/msg/detail/referline__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'refer_line'
#include "base_msgs/msg/detail/referline_point__traits.hpp"

namespace base_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Referline & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: refer_line
  {
    if (msg.refer_line.size() == 0) {
      out << "refer_line: []";
    } else {
      out << "refer_line: [";
      size_t pending_items = msg.refer_line.size();
      for (auto item : msg.refer_line) {
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
  const Referline & msg,
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

  // member: refer_line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.refer_line.size() == 0) {
      out << "refer_line: []\n";
    } else {
      out << "refer_line:\n";
      for (auto item : msg.refer_line) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Referline & msg, bool use_flow_style = false)
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
  const base_msgs::msg::Referline & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::msg::Referline & msg)
{
  return base_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::msg::Referline>()
{
  return "base_msgs::msg::Referline";
}

template<>
inline const char * name<base_msgs::msg::Referline>()
{
  return "base_msgs/msg/Referline";
}

template<>
struct has_fixed_size<base_msgs::msg::Referline>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<base_msgs::msg::Referline>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<base_msgs::msg::Referline>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__MSG__DETAIL__REFERLINE__TRAITS_HPP_

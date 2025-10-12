// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:msg/PlotInfo.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__PLOT_INFO__TRAITS_HPP_
#define BASE_MSGS__MSG__DETAIL__PLOT_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/msg/detail/plot_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'trajectory_info'
#include "base_msgs/msg/detail/local_trajectory__traits.hpp"
// Member 'obs_info'
#include "base_msgs/msg/detail/obs_info__traits.hpp"

namespace base_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PlotInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: trajectory_info
  {
    out << "trajectory_info: ";
    to_flow_style_yaml(msg.trajectory_info, out);
    out << ", ";
  }

  // member: obs_info
  {
    if (msg.obs_info.size() == 0) {
      out << "obs_info: []";
    } else {
      out << "obs_info: [";
      size_t pending_items = msg.obs_info.size();
      for (auto item : msg.obs_info) {
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
  const PlotInfo & msg,
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

  // member: trajectory_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trajectory_info:\n";
    to_block_style_yaml(msg.trajectory_info, out, indentation + 2);
  }

  // member: obs_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.obs_info.size() == 0) {
      out << "obs_info: []\n";
    } else {
      out << "obs_info:\n";
      for (auto item : msg.obs_info) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlotInfo & msg, bool use_flow_style = false)
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
  const base_msgs::msg::PlotInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::msg::PlotInfo & msg)
{
  return base_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::msg::PlotInfo>()
{
  return "base_msgs::msg::PlotInfo";
}

template<>
inline const char * name<base_msgs::msg::PlotInfo>()
{
  return "base_msgs/msg/PlotInfo";
}

template<>
struct has_fixed_size<base_msgs::msg::PlotInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<base_msgs::msg::PlotInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<base_msgs::msg::PlotInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__MSG__DETAIL__PLOT_INFO__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:msg/ObsInfo.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__OBS_INFO__TRAITS_HPP_
#define BASE_MSGS__MSG__DETAIL__OBS_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/msg/detail/obs_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace base_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObsInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: obs_length
  {
    out << "obs_length: ";
    rosidl_generator_traits::value_to_yaml(msg.obs_length, out);
    out << ", ";
  }

  // member: obs_width
  {
    out << "obs_width: ";
    rosidl_generator_traits::value_to_yaml(msg.obs_width, out);
    out << ", ";
  }

  // member: l
  {
    out << "l: ";
    rosidl_generator_traits::value_to_yaml(msg.l, out);
    out << ", ";
  }

  // member: s
  {
    out << "s: ";
    rosidl_generator_traits::value_to_yaml(msg.s, out);
    out << ", ";
  }

  // member: s_2path
  {
    out << "s_2path: ";
    rosidl_generator_traits::value_to_yaml(msg.s_2path, out);
    out << ", ";
  }

  // member: ds_dt_2path
  {
    out << "ds_dt_2path: ";
    rosidl_generator_traits::value_to_yaml(msg.ds_dt_2path, out);
    out << ", ";
  }

  // member: t_in
  {
    out << "t_in: ";
    rosidl_generator_traits::value_to_yaml(msg.t_in, out);
    out << ", ";
  }

  // member: t_out
  {
    out << "t_out: ";
    rosidl_generator_traits::value_to_yaml(msg.t_out, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObsInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: obs_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obs_length: ";
    rosidl_generator_traits::value_to_yaml(msg.obs_length, out);
    out << "\n";
  }

  // member: obs_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "obs_width: ";
    rosidl_generator_traits::value_to_yaml(msg.obs_width, out);
    out << "\n";
  }

  // member: l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "l: ";
    rosidl_generator_traits::value_to_yaml(msg.l, out);
    out << "\n";
  }

  // member: s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s: ";
    rosidl_generator_traits::value_to_yaml(msg.s, out);
    out << "\n";
  }

  // member: s_2path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s_2path: ";
    rosidl_generator_traits::value_to_yaml(msg.s_2path, out);
    out << "\n";
  }

  // member: ds_dt_2path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ds_dt_2path: ";
    rosidl_generator_traits::value_to_yaml(msg.ds_dt_2path, out);
    out << "\n";
  }

  // member: t_in
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_in: ";
    rosidl_generator_traits::value_to_yaml(msg.t_in, out);
    out << "\n";
  }

  // member: t_out
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_out: ";
    rosidl_generator_traits::value_to_yaml(msg.t_out, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObsInfo & msg, bool use_flow_style = false)
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
  const base_msgs::msg::ObsInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::msg::ObsInfo & msg)
{
  return base_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::msg::ObsInfo>()
{
  return "base_msgs::msg::ObsInfo";
}

template<>
inline const char * name<base_msgs::msg::ObsInfo>()
{
  return "base_msgs/msg/ObsInfo";
}

template<>
struct has_fixed_size<base_msgs::msg::ObsInfo>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<base_msgs::msg::ObsInfo>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<base_msgs::msg::ObsInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__MSG__DETAIL__OBS_INFO__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:msg/LocalSpeedsPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__TRAITS_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/msg/detail/local_speeds_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace base_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LocalSpeedsPoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: t
  {
    out << "t: ";
    rosidl_generator_traits::value_to_yaml(msg.t, out);
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

  // member: dds_dt_2path
  {
    out << "dds_dt_2path: ";
    rosidl_generator_traits::value_to_yaml(msg.dds_dt_2path, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: acceleration
  {
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << ", ";
  }

  // member: dacceleration
  {
    out << "dacceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.dacceleration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LocalSpeedsPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: t
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t: ";
    rosidl_generator_traits::value_to_yaml(msg.t, out);
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

  // member: dds_dt_2path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dds_dt_2path: ";
    rosidl_generator_traits::value_to_yaml(msg.dds_dt_2path, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration, out);
    out << "\n";
  }

  // member: dacceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dacceleration: ";
    rosidl_generator_traits::value_to_yaml(msg.dacceleration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LocalSpeedsPoint & msg, bool use_flow_style = false)
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
  const base_msgs::msg::LocalSpeedsPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::msg::LocalSpeedsPoint & msg)
{
  return base_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::msg::LocalSpeedsPoint>()
{
  return "base_msgs::msg::LocalSpeedsPoint";
}

template<>
inline const char * name<base_msgs::msg::LocalSpeedsPoint>()
{
  return "base_msgs/msg/LocalSpeedsPoint";
}

template<>
struct has_fixed_size<base_msgs::msg::LocalSpeedsPoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<base_msgs::msg::LocalSpeedsPoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<base_msgs::msg::LocalSpeedsPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__TRAITS_HPP_

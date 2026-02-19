// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:msg/LocalPathPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__TRAITS_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/msg/detail/local_path_point__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace base_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LocalPathPoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: s
  {
    out << "s: ";
    rosidl_generator_traits::value_to_yaml(msg.s, out);
    out << ", ";
  }

  // member: l
  {
    out << "l: ";
    rosidl_generator_traits::value_to_yaml(msg.l, out);
    out << ", ";
  }

  // member: ds_dt
  {
    out << "ds_dt: ";
    rosidl_generator_traits::value_to_yaml(msg.ds_dt, out);
    out << ", ";
  }

  // member: dl_ds
  {
    out << "dl_ds: ";
    rosidl_generator_traits::value_to_yaml(msg.dl_ds, out);
    out << ", ";
  }

  // member: dl_dt
  {
    out << "dl_dt: ";
    rosidl_generator_traits::value_to_yaml(msg.dl_dt, out);
    out << ", ";
  }

  // member: dds_dt
  {
    out << "dds_dt: ";
    rosidl_generator_traits::value_to_yaml(msg.dds_dt, out);
    out << ", ";
  }

  // member: ddl_ds
  {
    out << "ddl_ds: ";
    rosidl_generator_traits::value_to_yaml(msg.ddl_ds, out);
    out << ", ";
  }

  // member: ddl_dt
  {
    out << "ddl_dt: ";
    rosidl_generator_traits::value_to_yaml(msg.ddl_dt, out);
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << ", ";
  }

  // member: kappa
  {
    out << "kappa: ";
    rosidl_generator_traits::value_to_yaml(msg.kappa, out);
    out << ", ";
  }

  // member: dkappa
  {
    out << "dkappa: ";
    rosidl_generator_traits::value_to_yaml(msg.dkappa, out);
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
  const LocalPathPoint & msg,
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

  // member: s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s: ";
    rosidl_generator_traits::value_to_yaml(msg.s, out);
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

  // member: ds_dt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ds_dt: ";
    rosidl_generator_traits::value_to_yaml(msg.ds_dt, out);
    out << "\n";
  }

  // member: dl_ds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dl_ds: ";
    rosidl_generator_traits::value_to_yaml(msg.dl_ds, out);
    out << "\n";
  }

  // member: dl_dt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dl_dt: ";
    rosidl_generator_traits::value_to_yaml(msg.dl_dt, out);
    out << "\n";
  }

  // member: dds_dt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dds_dt: ";
    rosidl_generator_traits::value_to_yaml(msg.dds_dt, out);
    out << "\n";
  }

  // member: ddl_ds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ddl_ds: ";
    rosidl_generator_traits::value_to_yaml(msg.ddl_ds, out);
    out << "\n";
  }

  // member: ddl_dt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ddl_dt: ";
    rosidl_generator_traits::value_to_yaml(msg.ddl_dt, out);
    out << "\n";
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }

  // member: kappa
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "kappa: ";
    rosidl_generator_traits::value_to_yaml(msg.kappa, out);
    out << "\n";
  }

  // member: dkappa
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dkappa: ";
    rosidl_generator_traits::value_to_yaml(msg.dkappa, out);
    out << "\n";
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

inline std::string to_yaml(const LocalPathPoint & msg, bool use_flow_style = false)
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
  const base_msgs::msg::LocalPathPoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::msg::LocalPathPoint & msg)
{
  return base_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::msg::LocalPathPoint>()
{
  return "base_msgs::msg::LocalPathPoint";
}

template<>
inline const char * name<base_msgs::msg::LocalPathPoint>()
{
  return "base_msgs/msg/LocalPathPoint";
}

template<>
struct has_fixed_size<base_msgs::msg::LocalPathPoint>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<base_msgs::msg::LocalPathPoint>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<base_msgs::msg::LocalPathPoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:srv/GlobalPathService.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__SRV__DETAIL__GLOBAL_PATH_SERVICE__TRAITS_HPP_
#define BASE_MSGS__SRV__DETAIL__GLOBAL_PATH_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/srv/detail/global_path_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pnc_map'
#include "base_msgs/msg/detail/pnc_map__traits.hpp"

namespace base_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GlobalPathService_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: global_planner_type
  {
    out << "global_planner_type: ";
    rosidl_generator_traits::value_to_yaml(msg.global_planner_type, out);
    out << ", ";
  }

  // member: pnc_map
  {
    out << "pnc_map: ";
    to_flow_style_yaml(msg.pnc_map, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GlobalPathService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: global_planner_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "global_planner_type: ";
    rosidl_generator_traits::value_to_yaml(msg.global_planner_type, out);
    out << "\n";
  }

  // member: pnc_map
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pnc_map:\n";
    to_block_style_yaml(msg.pnc_map, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GlobalPathService_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace base_msgs

namespace rosidl_generator_traits
{

[[deprecated("use base_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const base_msgs::srv::GlobalPathService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::srv::GlobalPathService_Request & msg)
{
  return base_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::srv::GlobalPathService_Request>()
{
  return "base_msgs::srv::GlobalPathService_Request";
}

template<>
inline const char * name<base_msgs::srv::GlobalPathService_Request>()
{
  return "base_msgs/srv/GlobalPathService_Request";
}

template<>
struct has_fixed_size<base_msgs::srv::GlobalPathService_Request>
  : std::integral_constant<bool, has_fixed_size<base_msgs::msg::PNCMap>::value> {};

template<>
struct has_bounded_size<base_msgs::srv::GlobalPathService_Request>
  : std::integral_constant<bool, has_bounded_size<base_msgs::msg::PNCMap>::value> {};

template<>
struct is_message<base_msgs::srv::GlobalPathService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'global_path'
#include "nav_msgs/msg/detail/path__traits.hpp"

namespace base_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GlobalPathService_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: global_path
  {
    out << "global_path: ";
    to_flow_style_yaml(msg.global_path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GlobalPathService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: global_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "global_path:\n";
    to_block_style_yaml(msg.global_path, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GlobalPathService_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace base_msgs

namespace rosidl_generator_traits
{

[[deprecated("use base_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const base_msgs::srv::GlobalPathService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::srv::GlobalPathService_Response & msg)
{
  return base_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::srv::GlobalPathService_Response>()
{
  return "base_msgs::srv::GlobalPathService_Response";
}

template<>
inline const char * name<base_msgs::srv::GlobalPathService_Response>()
{
  return "base_msgs/srv/GlobalPathService_Response";
}

template<>
struct has_fixed_size<base_msgs::srv::GlobalPathService_Response>
  : std::integral_constant<bool, has_fixed_size<nav_msgs::msg::Path>::value> {};

template<>
struct has_bounded_size<base_msgs::srv::GlobalPathService_Response>
  : std::integral_constant<bool, has_bounded_size<nav_msgs::msg::Path>::value> {};

template<>
struct is_message<base_msgs::srv::GlobalPathService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<base_msgs::srv::GlobalPathService>()
{
  return "base_msgs::srv::GlobalPathService";
}

template<>
inline const char * name<base_msgs::srv::GlobalPathService>()
{
  return "base_msgs/srv/GlobalPathService";
}

template<>
struct has_fixed_size<base_msgs::srv::GlobalPathService>
  : std::integral_constant<
    bool,
    has_fixed_size<base_msgs::srv::GlobalPathService_Request>::value &&
    has_fixed_size<base_msgs::srv::GlobalPathService_Response>::value
  >
{
};

template<>
struct has_bounded_size<base_msgs::srv::GlobalPathService>
  : std::integral_constant<
    bool,
    has_bounded_size<base_msgs::srv::GlobalPathService_Request>::value &&
    has_bounded_size<base_msgs::srv::GlobalPathService_Response>::value
  >
{
};

template<>
struct is_service<base_msgs::srv::GlobalPathService>
  : std::true_type
{
};

template<>
struct is_service_request<base_msgs::srv::GlobalPathService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<base_msgs::srv::GlobalPathService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__SRV__DETAIL__GLOBAL_PATH_SERVICE__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from base_msgs:srv/PNCMapService.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__TRAITS_HPP_
#define BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "base_msgs/srv/detail/pnc_map_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace base_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const PNCMapService_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: map_type
  {
    out << "map_type: ";
    rosidl_generator_traits::value_to_yaml(msg.map_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PNCMapService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: map_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_type: ";
    rosidl_generator_traits::value_to_yaml(msg.map_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PNCMapService_Request & msg, bool use_flow_style = false)
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
  const base_msgs::srv::PNCMapService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::srv::PNCMapService_Request & msg)
{
  return base_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::srv::PNCMapService_Request>()
{
  return "base_msgs::srv::PNCMapService_Request";
}

template<>
inline const char * name<base_msgs::srv::PNCMapService_Request>()
{
  return "base_msgs/srv/PNCMapService_Request";
}

template<>
struct has_fixed_size<base_msgs::srv::PNCMapService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<base_msgs::srv::PNCMapService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<base_msgs::srv::PNCMapService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'pnc_map'
#include "base_msgs/msg/detail/pnc_map__traits.hpp"

namespace base_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const PNCMapService_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: pnc_map
  {
    out << "pnc_map: ";
    to_flow_style_yaml(msg.pnc_map, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PNCMapService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pnc_map
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pnc_map:\n";
    to_block_style_yaml(msg.pnc_map, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PNCMapService_Response & msg, bool use_flow_style = false)
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
  const base_msgs::srv::PNCMapService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  base_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use base_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const base_msgs::srv::PNCMapService_Response & msg)
{
  return base_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<base_msgs::srv::PNCMapService_Response>()
{
  return "base_msgs::srv::PNCMapService_Response";
}

template<>
inline const char * name<base_msgs::srv::PNCMapService_Response>()
{
  return "base_msgs/srv/PNCMapService_Response";
}

template<>
struct has_fixed_size<base_msgs::srv::PNCMapService_Response>
  : std::integral_constant<bool, has_fixed_size<base_msgs::msg::PNCMap>::value> {};

template<>
struct has_bounded_size<base_msgs::srv::PNCMapService_Response>
  : std::integral_constant<bool, has_bounded_size<base_msgs::msg::PNCMap>::value> {};

template<>
struct is_message<base_msgs::srv::PNCMapService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<base_msgs::srv::PNCMapService>()
{
  return "base_msgs::srv::PNCMapService";
}

template<>
inline const char * name<base_msgs::srv::PNCMapService>()
{
  return "base_msgs/srv/PNCMapService";
}

template<>
struct has_fixed_size<base_msgs::srv::PNCMapService>
  : std::integral_constant<
    bool,
    has_fixed_size<base_msgs::srv::PNCMapService_Request>::value &&
    has_fixed_size<base_msgs::srv::PNCMapService_Response>::value
  >
{
};

template<>
struct has_bounded_size<base_msgs::srv::PNCMapService>
  : std::integral_constant<
    bool,
    has_bounded_size<base_msgs::srv::PNCMapService_Request>::value &&
    has_bounded_size<base_msgs::srv::PNCMapService_Response>::value
  >
{
};

template<>
struct is_service<base_msgs::srv::PNCMapService>
  : std::true_type
{
};

template<>
struct is_service_request<base_msgs::srv::PNCMapService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<base_msgs::srv::PNCMapService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__TRAITS_HPP_

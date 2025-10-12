// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/PNCMap.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__PNC_MAP__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__PNC_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/pnc_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_PNCMap_midline
{
public:
  explicit Init_PNCMap_midline(::base_msgs::msg::PNCMap & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::PNCMap midline(::base_msgs::msg::PNCMap::_midline_type arg)
  {
    msg_.midline = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::PNCMap msg_;
};

class Init_PNCMap_right_boundary
{
public:
  explicit Init_PNCMap_right_boundary(::base_msgs::msg::PNCMap & msg)
  : msg_(msg)
  {}
  Init_PNCMap_midline right_boundary(::base_msgs::msg::PNCMap::_right_boundary_type arg)
  {
    msg_.right_boundary = std::move(arg);
    return Init_PNCMap_midline(msg_);
  }

private:
  ::base_msgs::msg::PNCMap msg_;
};

class Init_PNCMap_left_boundary
{
public:
  explicit Init_PNCMap_left_boundary(::base_msgs::msg::PNCMap & msg)
  : msg_(msg)
  {}
  Init_PNCMap_right_boundary left_boundary(::base_msgs::msg::PNCMap::_left_boundary_type arg)
  {
    msg_.left_boundary = std::move(arg);
    return Init_PNCMap_right_boundary(msg_);
  }

private:
  ::base_msgs::msg::PNCMap msg_;
};

class Init_PNCMap_road_half_width
{
public:
  explicit Init_PNCMap_road_half_width(::base_msgs::msg::PNCMap & msg)
  : msg_(msg)
  {}
  Init_PNCMap_left_boundary road_half_width(::base_msgs::msg::PNCMap::_road_half_width_type arg)
  {
    msg_.road_half_width = std::move(arg);
    return Init_PNCMap_left_boundary(msg_);
  }

private:
  ::base_msgs::msg::PNCMap msg_;
};

class Init_PNCMap_road_length
{
public:
  explicit Init_PNCMap_road_length(::base_msgs::msg::PNCMap & msg)
  : msg_(msg)
  {}
  Init_PNCMap_road_half_width road_length(::base_msgs::msg::PNCMap::_road_length_type arg)
  {
    msg_.road_length = std::move(arg);
    return Init_PNCMap_road_half_width(msg_);
  }

private:
  ::base_msgs::msg::PNCMap msg_;
};

class Init_PNCMap_header
{
public:
  Init_PNCMap_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PNCMap_road_length header(::base_msgs::msg::PNCMap::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PNCMap_road_length(msg_);
  }

private:
  ::base_msgs::msg::PNCMap msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::PNCMap>()
{
  return base_msgs::msg::builder::Init_PNCMap_header();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__PNC_MAP__BUILDER_HPP_

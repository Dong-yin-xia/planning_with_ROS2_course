// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/LocalSpeedsPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/local_speeds_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_LocalSpeedsPoint_dacceleration
{
public:
  explicit Init_LocalSpeedsPoint_dacceleration(::base_msgs::msg::LocalSpeedsPoint & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::LocalSpeedsPoint dacceleration(::base_msgs::msg::LocalSpeedsPoint::_dacceleration_type arg)
  {
    msg_.dacceleration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::LocalSpeedsPoint msg_;
};

class Init_LocalSpeedsPoint_acceleration
{
public:
  explicit Init_LocalSpeedsPoint_acceleration(::base_msgs::msg::LocalSpeedsPoint & msg)
  : msg_(msg)
  {}
  Init_LocalSpeedsPoint_dacceleration acceleration(::base_msgs::msg::LocalSpeedsPoint::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_LocalSpeedsPoint_dacceleration(msg_);
  }

private:
  ::base_msgs::msg::LocalSpeedsPoint msg_;
};

class Init_LocalSpeedsPoint_speed
{
public:
  explicit Init_LocalSpeedsPoint_speed(::base_msgs::msg::LocalSpeedsPoint & msg)
  : msg_(msg)
  {}
  Init_LocalSpeedsPoint_acceleration speed(::base_msgs::msg::LocalSpeedsPoint::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_LocalSpeedsPoint_acceleration(msg_);
  }

private:
  ::base_msgs::msg::LocalSpeedsPoint msg_;
};

class Init_LocalSpeedsPoint_dds_dt_2path
{
public:
  explicit Init_LocalSpeedsPoint_dds_dt_2path(::base_msgs::msg::LocalSpeedsPoint & msg)
  : msg_(msg)
  {}
  Init_LocalSpeedsPoint_speed dds_dt_2path(::base_msgs::msg::LocalSpeedsPoint::_dds_dt_2path_type arg)
  {
    msg_.dds_dt_2path = std::move(arg);
    return Init_LocalSpeedsPoint_speed(msg_);
  }

private:
  ::base_msgs::msg::LocalSpeedsPoint msg_;
};

class Init_LocalSpeedsPoint_ds_dt_2path
{
public:
  explicit Init_LocalSpeedsPoint_ds_dt_2path(::base_msgs::msg::LocalSpeedsPoint & msg)
  : msg_(msg)
  {}
  Init_LocalSpeedsPoint_dds_dt_2path ds_dt_2path(::base_msgs::msg::LocalSpeedsPoint::_ds_dt_2path_type arg)
  {
    msg_.ds_dt_2path = std::move(arg);
    return Init_LocalSpeedsPoint_dds_dt_2path(msg_);
  }

private:
  ::base_msgs::msg::LocalSpeedsPoint msg_;
};

class Init_LocalSpeedsPoint_s_2path
{
public:
  explicit Init_LocalSpeedsPoint_s_2path(::base_msgs::msg::LocalSpeedsPoint & msg)
  : msg_(msg)
  {}
  Init_LocalSpeedsPoint_ds_dt_2path s_2path(::base_msgs::msg::LocalSpeedsPoint::_s_2path_type arg)
  {
    msg_.s_2path = std::move(arg);
    return Init_LocalSpeedsPoint_ds_dt_2path(msg_);
  }

private:
  ::base_msgs::msg::LocalSpeedsPoint msg_;
};

class Init_LocalSpeedsPoint_t
{
public:
  Init_LocalSpeedsPoint_t()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalSpeedsPoint_s_2path t(::base_msgs::msg::LocalSpeedsPoint::_t_type arg)
  {
    msg_.t = std::move(arg);
    return Init_LocalSpeedsPoint_s_2path(msg_);
  }

private:
  ::base_msgs::msg::LocalSpeedsPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::LocalSpeedsPoint>()
{
  return base_msgs::msg::builder::Init_LocalSpeedsPoint_t();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__BUILDER_HPP_

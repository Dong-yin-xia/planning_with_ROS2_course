// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/ObsInfo.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__OBS_INFO__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__OBS_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/obs_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_ObsInfo_t_out
{
public:
  explicit Init_ObsInfo_t_out(::base_msgs::msg::ObsInfo & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::ObsInfo t_out(::base_msgs::msg::ObsInfo::_t_out_type arg)
  {
    msg_.t_out = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::ObsInfo msg_;
};

class Init_ObsInfo_t_in
{
public:
  explicit Init_ObsInfo_t_in(::base_msgs::msg::ObsInfo & msg)
  : msg_(msg)
  {}
  Init_ObsInfo_t_out t_in(::base_msgs::msg::ObsInfo::_t_in_type arg)
  {
    msg_.t_in = std::move(arg);
    return Init_ObsInfo_t_out(msg_);
  }

private:
  ::base_msgs::msg::ObsInfo msg_;
};

class Init_ObsInfo_ds_dt_2path
{
public:
  explicit Init_ObsInfo_ds_dt_2path(::base_msgs::msg::ObsInfo & msg)
  : msg_(msg)
  {}
  Init_ObsInfo_t_in ds_dt_2path(::base_msgs::msg::ObsInfo::_ds_dt_2path_type arg)
  {
    msg_.ds_dt_2path = std::move(arg);
    return Init_ObsInfo_t_in(msg_);
  }

private:
  ::base_msgs::msg::ObsInfo msg_;
};

class Init_ObsInfo_s_2path
{
public:
  explicit Init_ObsInfo_s_2path(::base_msgs::msg::ObsInfo & msg)
  : msg_(msg)
  {}
  Init_ObsInfo_ds_dt_2path s_2path(::base_msgs::msg::ObsInfo::_s_2path_type arg)
  {
    msg_.s_2path = std::move(arg);
    return Init_ObsInfo_ds_dt_2path(msg_);
  }

private:
  ::base_msgs::msg::ObsInfo msg_;
};

class Init_ObsInfo_s
{
public:
  explicit Init_ObsInfo_s(::base_msgs::msg::ObsInfo & msg)
  : msg_(msg)
  {}
  Init_ObsInfo_s_2path s(::base_msgs::msg::ObsInfo::_s_type arg)
  {
    msg_.s = std::move(arg);
    return Init_ObsInfo_s_2path(msg_);
  }

private:
  ::base_msgs::msg::ObsInfo msg_;
};

class Init_ObsInfo_l
{
public:
  explicit Init_ObsInfo_l(::base_msgs::msg::ObsInfo & msg)
  : msg_(msg)
  {}
  Init_ObsInfo_s l(::base_msgs::msg::ObsInfo::_l_type arg)
  {
    msg_.l = std::move(arg);
    return Init_ObsInfo_s(msg_);
  }

private:
  ::base_msgs::msg::ObsInfo msg_;
};

class Init_ObsInfo_obs_width
{
public:
  explicit Init_ObsInfo_obs_width(::base_msgs::msg::ObsInfo & msg)
  : msg_(msg)
  {}
  Init_ObsInfo_l obs_width(::base_msgs::msg::ObsInfo::_obs_width_type arg)
  {
    msg_.obs_width = std::move(arg);
    return Init_ObsInfo_l(msg_);
  }

private:
  ::base_msgs::msg::ObsInfo msg_;
};

class Init_ObsInfo_obs_length
{
public:
  Init_ObsInfo_obs_length()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObsInfo_obs_width obs_length(::base_msgs::msg::ObsInfo::_obs_length_type arg)
  {
    msg_.obs_length = std::move(arg);
    return Init_ObsInfo_obs_width(msg_);
  }

private:
  ::base_msgs::msg::ObsInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::ObsInfo>()
{
  return base_msgs::msg::builder::Init_ObsInfo_obs_length();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__OBS_INFO__BUILDER_HPP_

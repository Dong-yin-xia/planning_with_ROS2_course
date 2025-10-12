// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/LocalPathPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/local_path_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_LocalPathPoint_rdkappa
{
public:
  explicit Init_LocalPathPoint_rdkappa(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::LocalPathPoint rdkappa(::base_msgs::msg::LocalPathPoint::_rdkappa_type arg)
  {
    msg_.rdkappa = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_rkappa
{
public:
  explicit Init_LocalPathPoint_rkappa(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_rdkappa rkappa(::base_msgs::msg::LocalPathPoint::_rkappa_type arg)
  {
    msg_.rkappa = std::move(arg);
    return Init_LocalPathPoint_rdkappa(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_rtheta
{
public:
  explicit Init_LocalPathPoint_rtheta(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_rkappa rtheta(::base_msgs::msg::LocalPathPoint::_rtheta_type arg)
  {
    msg_.rtheta = std::move(arg);
    return Init_LocalPathPoint_rkappa(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_rs
{
public:
  explicit Init_LocalPathPoint_rs(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_rtheta rs(::base_msgs::msg::LocalPathPoint::_rs_type arg)
  {
    msg_.rs = std::move(arg);
    return Init_LocalPathPoint_rtheta(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_dkappa
{
public:
  explicit Init_LocalPathPoint_dkappa(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_rs dkappa(::base_msgs::msg::LocalPathPoint::_dkappa_type arg)
  {
    msg_.dkappa = std::move(arg);
    return Init_LocalPathPoint_rs(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_kappa
{
public:
  explicit Init_LocalPathPoint_kappa(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_dkappa kappa(::base_msgs::msg::LocalPathPoint::_kappa_type arg)
  {
    msg_.kappa = std::move(arg);
    return Init_LocalPathPoint_dkappa(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_theta
{
public:
  explicit Init_LocalPathPoint_theta(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_kappa theta(::base_msgs::msg::LocalPathPoint::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_LocalPathPoint_kappa(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_ddl_dt
{
public:
  explicit Init_LocalPathPoint_ddl_dt(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_theta ddl_dt(::base_msgs::msg::LocalPathPoint::_ddl_dt_type arg)
  {
    msg_.ddl_dt = std::move(arg);
    return Init_LocalPathPoint_theta(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_ddl_ds
{
public:
  explicit Init_LocalPathPoint_ddl_ds(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_ddl_dt ddl_ds(::base_msgs::msg::LocalPathPoint::_ddl_ds_type arg)
  {
    msg_.ddl_ds = std::move(arg);
    return Init_LocalPathPoint_ddl_dt(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_dds_dt
{
public:
  explicit Init_LocalPathPoint_dds_dt(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_ddl_ds dds_dt(::base_msgs::msg::LocalPathPoint::_dds_dt_type arg)
  {
    msg_.dds_dt = std::move(arg);
    return Init_LocalPathPoint_ddl_ds(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_dl_dt
{
public:
  explicit Init_LocalPathPoint_dl_dt(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_dds_dt dl_dt(::base_msgs::msg::LocalPathPoint::_dl_dt_type arg)
  {
    msg_.dl_dt = std::move(arg);
    return Init_LocalPathPoint_dds_dt(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_dl_ds
{
public:
  explicit Init_LocalPathPoint_dl_ds(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_dl_dt dl_ds(::base_msgs::msg::LocalPathPoint::_dl_ds_type arg)
  {
    msg_.dl_ds = std::move(arg);
    return Init_LocalPathPoint_dl_dt(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_ds_dt
{
public:
  explicit Init_LocalPathPoint_ds_dt(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_dl_ds ds_dt(::base_msgs::msg::LocalPathPoint::_ds_dt_type arg)
  {
    msg_.ds_dt = std::move(arg);
    return Init_LocalPathPoint_dl_ds(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_l
{
public:
  explicit Init_LocalPathPoint_l(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_ds_dt l(::base_msgs::msg::LocalPathPoint::_l_type arg)
  {
    msg_.l = std::move(arg);
    return Init_LocalPathPoint_ds_dt(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_s
{
public:
  explicit Init_LocalPathPoint_s(::base_msgs::msg::LocalPathPoint & msg)
  : msg_(msg)
  {}
  Init_LocalPathPoint_l s(::base_msgs::msg::LocalPathPoint::_s_type arg)
  {
    msg_.s = std::move(arg);
    return Init_LocalPathPoint_l(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

class Init_LocalPathPoint_pose
{
public:
  Init_LocalPathPoint_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalPathPoint_s pose(::base_msgs::msg::LocalPathPoint::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_LocalPathPoint_s(msg_);
  }

private:
  ::base_msgs::msg::LocalPathPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::LocalPathPoint>()
{
  return base_msgs::msg::builder::Init_LocalPathPoint_pose();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__BUILDER_HPP_

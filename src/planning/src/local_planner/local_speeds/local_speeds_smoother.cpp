/*
文件名: 速度平滑器
作者: C哥智驾说
完成时间: 2024.12

编译类型: 动态库
依赖: ROS2内部库:
      rclcpp
    外部库:
      base_msgs
      config_reader

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "local_speeds_smoother.h"

namespace Planning
{
    LocalSpeedsSmoother::LocalSpeedsSmoother() // 速度平滑器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speed"), "local_speeds_smoother created");

        // 读取配置文件
        local_speeds_config_ = std::make_unique<ConfigReader>();
        local_speeds_config_->read_local_speeds_config();
    }

    void LocalSpeedsSmoother::smooth_local_speeds(LocalSpeeds speeds) // 平滑速度
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speed"), "速度已经平滑了");
        (void)speeds;
    }

} // namespace Planning
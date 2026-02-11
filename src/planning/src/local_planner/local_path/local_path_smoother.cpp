/*
文件名: 局部路径平滑器
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

#include "local_path_smoother.h"

namespace Planning
{
    LocalPathSmoother::LocalPathSmoother() // 局部路径平滑器 117
    {
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "局部路径平滑器已创建");

        // 读取配置文件
        local_path_config_ = std::make_unique<ConfigReader>();
        local_path_config_->read_local_path_config();
    }

    // 平滑局部路径
    void LocalPathSmoother::smooth_local_path(LocalPath &local_path)
    {
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "局部路径平滑完成");
        (void)local_path;
    }

} // namespace Planning
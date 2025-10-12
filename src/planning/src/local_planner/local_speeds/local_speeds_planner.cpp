/*
文件名: 速度规划器
作者: C哥智驾说
完成时间: 2024.12

编译类型: 动态库
依赖: ROS2内部库:
        rclcpp
      外部库:
        base_msgs
        config_reader
        mathlibs
        decision_center

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "local_speeds_planner.h"

namespace Planning
{
    LocalSpeedsPlanner::LocalSpeedsPlanner() // 速度规划器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speed"), "local_speeds_planner created");
    }

} // namespace Planning
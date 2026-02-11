/*
文件名: 轨迹合成器
作者: C哥智驾说
完成时间: 2024.12

编译类型: 动态库
依赖: ROS2内部库:
        rclcpp
    外部库:
        base_msgs

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "local_trajectory_combiner.h"

namespace Planning
{
    LocalTrajectoryCombiner::LocalTrajectoryCombiner() // 轨迹合成器
    {
        RCLCPP_INFO(rclcpp::get_logger("local_trajectory"), "本地轨迹合成器已创建");

        // 读取配置文件
        trajectory_config_ = std::make_unique<ConfigReader>();
        // trajectory_config_->read_local_trajectory_config();
    }

    LocalTrajectory LocalTrajectoryCombiner::combine_trajectory(const LocalPath &path, const LocalSpeeds &speeds)  // 合成轨迹
    {
        const int path_size = path.local_path.size();
        const int speeds_size = speeds.local_speeds.size();
        local_trajectory_.header = path.header;  // 设置消息头
        local_trajectory_.local_trajectory.clear();  // 清空轨迹

        if (path_size < 3 || speeds_size < 3)
        {
            RCLCPP_WARN(rclcpp::get_logger("trajectory"), "路径点数或速度点数为空");
            return local_trajectory_;
        }

        LocalTrajectoryPoint point_tmp;
        for (int i = 0; i < path_size; i++)
        {
            // 路径部分的赋值
            point_tmp.path_point = path.local_path[i];
            // 速度部分的赋值
            if (i < speeds_size) // 因为路径长度是随速度动态变化的，所以可以直接用速度规划的下标赋值
            {
                point_tmp.speed_point = speeds.local_speeds[i]; 
            }

            // 填充进轨迹
            local_trajectory_.local_trajectory.emplace_back(point_tmp);
        }

        RCLCPP_INFO(rclcpp::get_logger("trajectory"), "轨迹合成完成,轨迹点数: %ld", local_trajectory_.local_trajectory.size());
        return local_trajectory_;
    }

} // namespace Planning
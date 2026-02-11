/*
文件名: 普通全局路径规划器
作者: C哥智驾说
完成时间: 2024.12

编译类型: 动态库
依赖: ROS2内部库:
        rclcpp
        geometry_msgs
        nav_msgs
    外部库:
        base_msgs
        config_reader

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "global_planner_normal.h"
#include "global_planner_base.h"

namespace Planning
{
    GlobalPlannerNormal::GlobalPlannerNormal() // 普通全局路径规划器
    {
        RCLCPP_INFO(rclcpp::get_logger("global_path"), "全局路径_normal已创建");
    }

    Path GlobalPlannerNormal::search_global_path(const PNCMap &pnc_map) // 搜索全局路径
    {
        RCLCPP_INFO(rclcpp::get_logger("global_path"), "使用normal路径规划");

        global_path_.header.frame_id = pnc_map.header.frame_id;
        global_path_.header.stamp = rclcpp::Clock().now();
        global_path_.poses.clear();

        PoseStamped p_tmp;
        p_tmp.header = global_path_.header;
        p_tmp.pose.orientation.x = 0.0;
        p_tmp.pose.orientation.y = 0.0;
        p_tmp.pose.orientation.z = 0.0;
        p_tmp.pose.orientation.w = 1.0;

        const int midline_size = pnc_map.midline.points.size();
        for (int i = 0; i < midline_size; i++)
        {
            p_tmp.pose.position.x = (pnc_map.midline.points[i].x + pnc_map.right_boundary.points[i].x) / 2.0;
            p_tmp.pose.position.y = (pnc_map.midline.points[i].y + pnc_map.right_boundary.points[i].y) / 2.0;
            global_path_.poses.emplace_back(p_tmp);
        }

        RCLCPP_INFO(rclcpp::get_logger("global_path"), "全局路径已经创建，点的数量: %ld", global_path_.poses.size());
        return global_path_;
    }

} // namespace Planning
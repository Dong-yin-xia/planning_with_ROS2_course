/*
文件名: 全局路径服务器
作者: C哥智驾说
完成时间: 2024.12

编译类型: 节点（global_path_server_node）
依赖: ROS2内部库:
        rclcpp
        geometry_msgs
        nav_msgs
        visualization_msgs
    外部库:
        base_msgs
        config_reader
        global_planner

接收请求: base_msgs::msg::PNCMap pnc_map
响应: nav_msgs::msg::Path global_path
发布: nav_msgs::msg::Path global_path
    visualization_msgs::msg::Marker global_path_rviz

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "global_path_server.h"

namespace Planning
{
    GlobalPathServer::GlobalPathServer() : Node("global_path_server_node") // 全局路径服务器
    {
        RCLCPP_INFO(this->get_logger(), "global_path_server_node created");

        // 全局路径发布器，参数: 全局路径话题名，队列大小
        global_path_pnb_ = this->create_publisher<Path>("global_path", 10);

        // 全局路径marker发布器，参数: 全局路径话题名，队列大小
        global_path_rviz_pnb_ = this->create_publisher<Marker>("global_path_rviz", 10);

        // 全局路径服务器，参数: 全局路径服务话题名，回调函数
        global_path_service_ = this->create_service<GlobalPathService>(
            "global_path_service", 
            std::bind(&GlobalPathServer::response_global_path_callback, this, _1, _2)
        );
    }

    // 响应并发布全局路径回调函数
    void GlobalPathServer::response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request, 
                                                         const std::shared_ptr<GlobalPathService::Response> response)
    {
        // 接受请求，多态
        switch (request->global_planner_type)
        {
            case static_cast<int>(GlobalPlannerType::NORMAL): // 普通全局路径规划器
                global_planner_base_ = std::make_shared<GlobalPlannerNormal>();
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid global planner type!");
                return;
        }
        // 判断请求是否为空
        if (request->pnc_map.midline.points.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "pnc_map is empty, global path cannot be generated");
            return;
        }
        // 创建并响应全局路径
        const auto global_path = global_planner_base_->search_global_path(request->pnc_map);
        response->global_path = global_path;

        // 发布全局路径，Planning node使用
        // 因为只发布1次，并且path没有frame_locked，所以无法固定在rviz中
        global_path_pnb_->publish(global_path);
        RCLCPP_INFO(this->get_logger(), "global_path published");

        // 发布全局路径markerarray，rviz使用
        const auto global_path_rviz = path2marker(global_path);
        global_path_rviz_pnb_->publish(global_path_rviz);
        RCLCPP_INFO(this->get_logger(), "global_path for rviz published");
    }

    Marker GlobalPathServer::path2marker(const Path &path)
    {
        Marker path_rviz_;
        path_rviz_.header = path.header;
        path_rviz_.ns = "global_path";
        path_rviz_.id = 0;
        path_rviz_.action = Marker::ADD;
        path_rviz_.type = Marker::LINE_STRIP;  // 连续线条
        path_rviz_.scale.x = 0.05;  // 线条宽度
        path_rviz_.color.a = 1.0;  // 线条透明度
        path_rviz_.color.r = 0.8;  // 红色
        path_rviz_.color.g = 0.0;  // 绿色
        path_rviz_.color.b = 0.0;  // 蓝色
        path_rviz_.lifetime = rclcpp::Duration::max();  // 无限时间
        path_rviz_.frame_locked = true;  // 固定在rviz中

        Point p_tmp;
        for (const auto &pose : path.poses)
        {
            p_tmp.x = pose.pose.position.x;
            p_tmp.y = pose.pose.position.y;
            path_rviz_.points.emplace_back(p_tmp);
        }
        
        return path_rviz_;
    }

} // namespace Planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::GlobalPathServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
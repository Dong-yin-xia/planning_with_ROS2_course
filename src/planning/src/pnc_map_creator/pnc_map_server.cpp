/*
文件名: pnc地图服务器
作者: C哥智驾说
完成时间: 2024.12

编译类型: 节点（pnc_map_server_node）
依赖: ROS2内部库:
        rclcpp
    外部库:
        base_msgs
        config_reader
        pnc_map_creator

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "pnc_map_server.h"

namespace Planning
{
    PNCMapServer::PNCMapServer() : Node("pnc_map_server_node") // 全局路径服务器
    {
        RCLCPP_INFO(this->get_logger(), "pnc_map_server_node created");

        // 地图发布器，参数: 地图话题名，队列大小
        map_pnb_ = this->create_publisher<PNCMap>("pnc_map", 10);

        // 地图markerarray发布器，参数: 地图话题名，队列大小
        map_rviz_pnb_ = this->create_publisher<MarkerArray>("pnc_map_markerarray", 10);

        // 地图服务器，参数: 地图服务话题名，回调函数
        map_service_ = this->create_service<PNCMapService>(
            "pnc_map_service", 
            std::bind(&PNCMapServer::response_pnc_map_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

    }

    // 响应并发布地图回调函数
    void PNCMapServer::response_pnc_map_callback(const std::shared_ptr<PNCMapService::Request> request, 
                                                 const std::shared_ptr<PNCMapService::Response> response)
    {
        // 接受请求，多态
        switch (request->map_type)
        {
            case static_cast<int>(PNCMapType::STRAIGHT): // 直线路径
                map_creator_ = std::make_shared<PNCMapCreatorStraight>();
                break;
            case static_cast<int>(PNCMapType::STERN): // 转角路径
                map_creator_ = std::make_shared<PNCMapCreatorSTurn>();
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Invalid map type!");
                return;
        }
        // 创建并响应地图
        const auto pnc_map = map_creator_->create_pnc_map();
        response->pnc_map = pnc_map;

        // 发布地图，Planning node使用
        map_pnb_->publish(pnc_map);
        RCLCPP_INFO(this->get_logger(), "pnc_map published");

        // 发布地图markerarray，rviz使用
        const auto pnc_map_markerarray= map_creator_->pnc_map_markerarray();
        map_rviz_pnb_->publish(pnc_map_markerarray);
        RCLCPP_INFO(this->get_logger(),"pnc_map for rviz published");
    }

} // namespace Planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::PNCMapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
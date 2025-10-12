#ifndef GLOBAL_PATH_SERVER_H_
#define GLOBAL_PATH_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "global_planner_normal.h"
#include <geometry_msgs/msg/detail/point__struct.hpp>

namespace Planning
{
    using std::placeholders::_1; // 占位符1 
    using std::placeholders::_2; // 占位符2

    using base_msgs::srv::GlobalPathService;  // 全局路径服务
    using geometry_msgs::msg::Point;  // 点
    using visualization_msgs::msg::Marker;  // 标记

    class GlobalPathServer : public rclcpp::Node
    {
    public:
        GlobalPathServer();

    private:
        // 响应并发布全局路径回调函数
        void response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request, 
                                           const std::shared_ptr<GlobalPathService::Response> response);
        Marker path2marker(const Path &path);  // path转换为marker

    private:
        std::shared_ptr<GlobalPlannerBase> global_planner_base_; // 全局路径规划器，用智能指针管理
        rclcpp::Publisher<Path>::SharedPtr global_path_pnb_; // 全局路径发布器（规划模块使用）
        rclcpp::Publisher<Marker>::SharedPtr global_path_rviz_pnb_;// 全局路径marker发布器（rviz使用）
        rclcpp::Service<GlobalPathService>::SharedPtr global_path_service_; // 全局路径服务器
    };

} // namespace Planning
#endif // GLOBAL_PATH_SERVER_H_

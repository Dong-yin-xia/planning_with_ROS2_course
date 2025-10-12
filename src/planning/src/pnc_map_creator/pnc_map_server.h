#ifndef PNC_MAP_SERVER_H_
#define PNC_MAP_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
// #include "pnc_map_creator_base.h"
#include "pnc_map_creator_straight.h"
#include "pnc_map_creator_sturn.h"

namespace Planning
{
    using base_msgs::srv::PNCMapService;  // 引用服务

    class PNCMapServer : public rclcpp::Node
    {
    public:
        PNCMapServer();

    private:
        // 响应并发布地图回调函数
        void response_pnc_map_callback(const std::shared_ptr<PNCMapService::Request> request, 
                                       const std::shared_ptr<PNCMapService::Response> response);

    private:
        std::shared_ptr<PNCMapCreatorBase> map_creator_; // 地图创建器，用智能指针管理
        rclcpp::Publisher<PNCMap>::SharedPtr map_pnb_; // 地图发布器（规划模块使用）
        rclcpp::Publisher<MarkerArray>::SharedPtr map_rviz_pnb_;// 地图markerarray发布器（rviz使用）
        rclcpp::Service<PNCMapService>::SharedPtr map_service_; // 地图服务器
        
    };
} // namespace Planning
#endif // PNC_MAP_SERVER_H_

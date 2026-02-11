#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "base_msgs/srv/global_path_service.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "base_msgs/msg/obs_info.hpp"
#include "base_msgs/msg/plot_info.hpp"
#include "base_msgs/msg/referline.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"

#include "config_reader.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include "reference_line_creator.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"
#include "vehicle_info_base.h"

#include <base_msgs/msg/detail/obs_info__struct.hpp>
#include <base_msgs/msg/detail/plot_info__struct.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <vector>
#include <cmath>
#include <algorithm>

namespace Planning
{
    using namespace std::chrono_literals;
    using base_msgs::msg::LocalTrajectory;  // 轨迹
    using base_msgs::msg::ObsInfo;
    using base_msgs::msg::PlotInfo;
    using base_msgs::msg::PNCMap;  // pnc地图
    using base_msgs::srv::GlobalPathService;  // 全局路径服务
    using base_msgs::srv::PNCMapService;  // 地图服务
    using nav_msgs::msg::Path;  // 路径
    using geometry_msgs::msg::PoseStamped;  // 位姿
    using tf2_ros::StaticTransformBroadcaster;  // 静态变换广播器
    using tf2_ros::Buffer;  // 缓存
    using tf2_ros::TransformListener;  // 变换监听器


    class PlanningProcess : public rclcpp::Node // 规划总流程
    {
    public:
        PlanningProcess(); 
        bool process(); // 总流程
    
    private:
        bool planning_init(); // 规划初始化
        void vehicle_spawn(const std::shared_ptr<VehicleBase> &vehicle); // 车辆生成
        void get_location(const std::shared_ptr<VehicleBase> &vehicle); // 获取位置,监听定位点


        template<typename T>
        bool connect_server(const T &client); // 连接服务器
        bool map_request(); // 请求地图
        bool global_path_request(); // 请求全局路径
        void planning_callback(); // 总流程回调


    public:
        inline PNCMap pnc_map() const {return pnc_map_;}  // 获取pnc地图
        inline Path global_path() const {return global_path_;}  // 获取全局路径

    private:
        std::unique_ptr<ConfigReader> process_config_;          // 配置
        std::shared_ptr<VehicleBase> car_;                     // 主车
        std::vector<std::shared_ptr<VehicleBase>> obses_spawn_;   // 所有障碍物车辆，模拟感知信号
        std::vector<std::shared_ptr<VehicleBase>> obses_;   // 实际需要考虑的障碍物车辆
        double obs_dis_ = 0.0;                                  // 障碍物距离

        std::shared_ptr<StaticTransformBroadcaster> tf_broadcaster_;  // 静态变换广播器
        std::unique_ptr<Buffer> buffer_;  // 缓存对象
        std::shared_ptr<TransformListener> tf_listener_;  // 位置监听器


        PNCMap pnc_map_;                                        // pnc地图
        Path global_path_;                                      // 全局路径
        rclcpp::Client<PNCMapService>::SharedPtr map_client_; // 地图服务器(请求客户端)
        rclcpp::Client<GlobalPathService>::SharedPtr global_path_client_; // 全局路径服务器(请求客户端)

        std::shared_ptr<ReferencelineCreator> refer_line_creator_;  // 参考线创建器
        rclcpp::Publisher<Path>::SharedPtr refer_line_pnb_;  // 参考线发布器

        std::shared_ptr<DecisionCenter> decider_;  // 决策器

        std::shared_ptr<LocalPathPlanner> local_path_planner_;  // 局部路径规划器
        std::shared_ptr<LocalSpeedsPlanner> local_speeds_planner_;  // 速度规划器
        rclcpp::Publisher<Path>::SharedPtr local_path_pnb_;  // 局部路径发布器

        std::shared_ptr<LocalTrajectoryCombiner> local_trajectory_combiner_;  // 轨迹合成器
        rclcpp::Publisher<LocalTrajectory>::SharedPtr local_trajectory_pnb_;  // 轨迹发布器

        rclcpp::Publisher<PlotInfo>::SharedPtr plot_info_pub_;  // 绘图信息发布器
        
        rclcpp::TimerBase::SharedPtr timer_;  // 定时器事件

        // 参考线切换与拼接
        int reference_line_type_ = 0; // 0: normal, 1: stitch
        base_msgs::msg::Referline prev_refer_line_;

    };
} // namespace Planning
#endif // PLANNING_PROCESS_H_
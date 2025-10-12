#ifndef CAR_MOVE_CMD_H_
#define CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include <memory>
#include "config_reader.h"
#include "main_car_info.h"
#include "vehicle_info_base.h"

namespace Planning
{
    using namespace std::chrono_literals;
    using std::placeholders::_1;

    using base_msgs::msg::LocalTrajectory;
    using geometry_msgs::msg::TransformStamped;
    using tf2_ros::TransformBroadcaster;

    struct car_param
    {
        double pose_x_ = 0.0;  // x坐标
        double pose_y_ = 0.0;  // y坐标  
        double theta_ = 0.0;  // 航向角
        double speed_ = 0.0;  // 速度

    };
    
    class CarMoveCmd : public rclcpp::Node // 主车运动指令
    {
    public:
        CarMoveCmd();

    private:
        void car_broadcast_tf(const LocalTrajectory::SharedPtr trajectory);  // 广播主车坐标变换

    private:
        std::unique_ptr<ConfigReader> move_cmd_config_;  // 运动指令配置
        std::shared_ptr<TransformBroadcaster> broadcaster_;  // 变换广播器
        rclcpp::Subscription<LocalTrajectory>::SharedPtr local_trajectory_sub_;  // 局部轨迹订阅
        std::shared_ptr<VehicleBase> car_;  // 主车,仅用于初始化
        car_param car_param_;  // 主车参数


    };
} // namespace Planning
#endif // CAR_MOVE_CMD_H_

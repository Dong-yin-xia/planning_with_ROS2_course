/*
文件名: 主车运动指令
作者: C哥智驾说
完成时间: 2024.12

编译类型: 节点(move_cmd_node)
依赖: ROS2内部库:
      rclcpp
      tf2_ros
      geometry_msgs
    外部库:
      base_msgs
      config_reader
      vehicle_info

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "car_move_cmd.h"
#include "main_car_info.h"
#include <cmath>

namespace Planning
{
    CarMoveCmd::CarMoveCmd() : Node("car_move_cmd_node") // 主车运动指令
    {
        RCLCPP_INFO(this->get_logger(), "car_move_cmd_node已创建");

        // 读取配置文件
        move_cmd_config_ = std::make_unique<ConfigReader>();
        move_cmd_config_->read_move_cmd_config();

        // 初始化
        car_ = std::make_shared<MainCar>();
        car_param_.pose_x_ = car_->loc_point().pose.position.x;
        car_param_.pose_y_ = car_->loc_point().pose.position.y;
        car_param_.theta_ = car_->theta();
        car_param_.speed_ = car_->speed();

        // 变换广播器
        broadcaster_ = std::make_shared<TransformBroadcaster>(this);

        // 创建轨迹订阅器，广播运动指令
        local_trajectory_sub_ = this->create_subscription<LocalTrajectory>("planning/local_trajectory", 
                                            10, std::bind(&CarMoveCmd::car_broadcast_tf, this, _1));
    }

    void CarMoveCmd::car_broadcast_tf(const LocalTrajectory::SharedPtr trajectory)  // 广播主车坐标变换
    {
        // 接受轨迹
        const int trajectory_size = trajectory->local_trajectory.size();
        if (trajectory_size < 3)
        {
            RCLCPP_WARN(this->get_logger(), "轨迹点数小于3,无法广播主车坐标变换");
            return;
        }

        // 创建消息对象
        TransformStamped transform_data;
        transform_data.header.stamp = trajectory->header.stamp;
        transform_data.header.frame_id = move_cmd_config_->pnc_map().frame_;
        transform_data.child_frame_id = car_->child_frame();

        // 计算投影点，用匹配点近似代替
        double min_dis = std::numeric_limits<double>::max();
        int closest_index = -1;
        for (int i = 0; i < trajectory_size; i++)
        {
            double dis = std::hypot(trajectory->local_trajectory[i].path_point.pose.pose.position.x - car_param_.pose_x_, 
                                    trajectory->local_trajectory[i].path_point.pose.pose.position.y - car_param_.pose_y_);
            if (dis < min_dis)
            {
                min_dis = dis;
                closest_index = i;
            }
        }

        // 赋值

        // 速度更新
        const double speed_x = trajectory->local_trajectory[closest_index].speed_point.speed * std::cos(trajectory->local_trajectory[closest_index].path_point.theta);
        const double speed_y = trajectory->local_trajectory[closest_index].speed_point.speed * std::sin(trajectory->local_trajectory[closest_index].path_point.theta);

        // 位置更新
      #ifdef USE_ACTUAL_POS

        // 1. 直接用实际位置赋值，弯道上会有误差
        transform_data.transform.translation.x = car_param_.pose_x_;
        transform_data.transform.translation.y = car_param_.pose_y_;

      #else
        // 2. 用匹配点位置赋值，在加减速时会有误差
        transform_data.transform.translation.x = trajectory->local_trajectory[closest_index].path_point.pose.pose.position.x;
        transform_data.transform.translation.y = trajectory->local_trajectory[closest_index].path_point.pose.pose.position.y;

      #endif

        // 位置更新
        car_param_.pose_x_ += speed_x;
        car_param_.pose_y_ += speed_y;

        // 航向角更新
        transform_data.transform.rotation.x = trajectory->local_trajectory[closest_index].path_point.pose.pose.orientation.x;
        transform_data.transform.rotation.y = trajectory->local_trajectory[closest_index].path_point.pose.pose.orientation.y;
        transform_data.transform.rotation.z = trajectory->local_trajectory[closest_index].path_point.pose.pose.orientation.z;
        transform_data.transform.rotation.w = trajectory->local_trajectory[closest_index].path_point.pose.pose.orientation.w;

        // 广播消息
        RCLCPP_INFO(this->get_logger(), "广播主车坐标变换, pos:(%.2f, %.2f),speed:(%.2f, %.2f), local_trajectory_size: %ld",
                                      car_param_.pose_x_, car_param_.pose_y_, speed_x, speed_y, trajectory->local_trajectory.size());
        broadcaster_->sendTransform(transform_data);
    }

} // namespace Planning

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::CarMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
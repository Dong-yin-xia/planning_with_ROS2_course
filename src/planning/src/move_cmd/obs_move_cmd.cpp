/*
文件名: 障碍物运动指令
作者: C哥智驾说
完成时间: 2024.12

编译类型: 节点(move_cmd_node)
依赖: ROS2内部库:
      rclcpp
      tf2
      tf2_ros
      geometry_msgs
    外部库:
      config_reader
      vehical_info

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "obs_move_cmd.h"
#include "obs_car_info.h"
#include <memory>

namespace Planning
{
    ObsMoveCmd::ObsMoveCmd() : Node("obs_move_cmd_node") // 主车运动指令
    {
        RCLCPP_INFO(this->get_logger(), "obs_move_cmd_node已创建");

        // 读取配置文件
        obs_move_cmd_config_ = std::make_unique<ConfigReader>();
        obs_move_cmd_config_->read_move_cmd_config();

        // 初始化障碍物参数
        for (int i = 0; i < 3; i++)
        {
            ObsParam obs_param;
            obs_param.obs_ = std::make_shared<ObsCar>(i + 1);
            obs_param.obs_broadcaster_ = std::make_shared<TransformBroadcaster>(this);

            obs_param.pose_x_ = obs_param.obs_->loc_point().pose.position.x;
            obs_param.pose_y_ = obs_param.obs_->loc_point().pose.position.y;
            obs_param.theta_ = obs_param.obs_->theta();
            obs_param.speed_ = obs_param.obs_->speed();

            obses_param.emplace_back(obs_param);
        }

        // 让车辆运动
        timer_ = this->create_wall_timer(0.1s, std::bind(&ObsMoveCmd::obs_broadcast_tf, this));
    }

    void ObsMoveCmd::obs_broadcast_tf()  // 广播障碍物坐标变换
    {
        for (auto &obs_param : obses_param)
        {
            // 创建消息对象
            TransformStamped transform_data;
            transform_data.header.stamp = this->now();
            transform_data.header.frame_id = obs_move_cmd_config_->pnc_map().frame_;
            transform_data.child_frame_id = obs_param.obs_->child_frame();

            // 赋值
            transform_data.transform.translation.x = obs_param.pose_x_;
            transform_data.transform.translation.y = obs_param.pose_y_;
            const double speed_x = obs_param.speed_ * std::cos(obs_param.theta_);
            const double speed_y = obs_param.speed_ * std::sin(obs_param.theta_);

            // 位置更新
            obs_param.pose_x_ += speed_x;
            obs_param.pose_y_ += speed_y;
            
            tf2::Quaternion qtn;
            qtn.setRPY(0.0, 0.0, obs_param.theta_);
            transform_data.transform.rotation.x = qtn.getX();
            transform_data.transform.rotation.y = qtn.getY();
            transform_data.transform.rotation.z = qtn.getZ();
            transform_data.transform.rotation.w = qtn.getW();
            
            // 广播消息
            RCLCPP_INFO(this->get_logger(), "广播障碍物坐标变换, id: %d, pos:(%.2f, %.2f),speed:(%.2f, %.2f), theta: %.2f",
                                 obs_param.obs_->id(),obs_param.pose_x_, obs_param.pose_y_, speed_x, speed_y, obs_param.theta_);
            obs_param.obs_broadcaster_->sendTransform(transform_data);
        }
    }

} // namespace Planning

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::ObsMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
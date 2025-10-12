/*
文件名: 参考线创建器
作者: C哥智驾说
完成时间: 2024.12

编译类型: 动态库
依赖: ROS2内部库:
        rclcpp
        nav_msgs
        geometry_msgs
    外部库:
        base_msgs
        config_reader
        mathlibs

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "reference_line_creator.h"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>

namespace Planning
{
    ReferencelineCreator::ReferencelineCreator() // 创建参考线
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference_line_creator created");

        // 读取配置文件
        reference_line_config_ = std::make_unique<ConfigReader>();
        reference_line_config_->read_reference_line_config();

        // 创建平滑器
        reference_line_smoother_ = std::make_shared<ReferenceLineSmoother>();
    }

    // 生成参考线
    Referline ReferencelineCreator::create_reference_line(const Path &global_path,
                                                          const PoseStamped &target_point)
    {
        // 如果全局路径为空，返回空参考线
        if (global_path.poses.empty())
        {
            return refer_line_;
        }

        // 找到匹配点
        match_point_index_ = Curve::find_match_point(global_path, last_match_point_index_, target_point);
        last_match_point_index_ = match_point_index_;   // 更新上一次匹配点的下标
        if (match_point_index_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "寻找匹配点失败");
            return refer_line_;
        }

        // 计算最前点和最后点在全局路径下的下标
        const int global_path_size = global_path.poses.size();
        front_index_ = (global_path_size - 1 >= match_point_index_ + reference_line_config_->refer_line().front_size_)
                       ? (match_point_index_ + reference_line_config_->refer_line().front_size_)
                       : (global_path_size - 1);
        back_index_ = (0 <= match_point_index_ - reference_line_config_->refer_line().back_size_)
                        ? (match_point_index_ - reference_line_config_->refer_line().back_size_)
                        : 0;
        
        // 填充参考线
        refer_line_.header.frame_id = reference_line_config_->pnc_map().frame_;
        refer_line_.header.stamp = rclcpp::Clock().now();
        refer_line_.refer_line.clear();
        ReferlinePoint point_tmp;
        for (int i = back_index_; i <= front_index_; i++)
        {
            point_tmp.pose = global_path.poses[i];
            refer_line_.refer_line.emplace_back(point_tmp);
        }

        // 平滑整条参考线
        reference_line_smoother_->smooth_reference_line(refer_line_);


        // 计算投影点参数
        Curve::cal_projection_param(refer_line_);

        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "参考线创建完成，匹配点下标=%d,前方点数=%d,后方点数=%d,尺寸=%ld", 
                                            match_point_index_, front_index_, back_index_, refer_line_.refer_line.size());
        return refer_line_;

    }

    // 转换成rviz的Path格式
    Path ReferencelineCreator::referline_to_rviz()
    {
        refer_line_rviz_.header = refer_line_.header;
        refer_line_rviz_.poses.clear();

        PoseStamped point_tmp;
        for (const auto &point : refer_line_.refer_line)
        {
            point_tmp.header = refer_line_rviz_.header;
            point_tmp.pose = point.pose.pose;
            refer_line_rviz_.poses.emplace_back(point_tmp);
        }
        return refer_line_rviz_;
    }

}// namespace Planning
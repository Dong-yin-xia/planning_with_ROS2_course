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
#include "curve.h"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <base_msgs/msg/referline_point.hpp>

namespace Planning
{
    ReferencelineCreator::ReferencelineCreator() // 创建参考线
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "reference_line_creator已创建");

        // 读取配置文件
        reference_line_config_ = std::make_unique<ConfigReader>();
        reference_line_config_->read_reference_line_config();

        // 创建平滑器
        reference_line_smoother_ = std::make_shared<ReferenceLineSmoother>();
    }

    // 生成参考线
    base_msgs::msg::Referline ReferencelineCreator::create_reference_line(const nav_msgs::msg::Path &global_path,
                                                          const geometry_msgs::msg::PoseStamped &target_point)
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
        base_msgs::msg::ReferlinePoint point_tmp;
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
    nav_msgs::msg::Path ReferencelineCreator::referline_to_rviz()
    {
        refer_line_rviz_.header = refer_line_.header;
        refer_line_rviz_.poses.clear();

        geometry_msgs::msg::PoseStamped point_tmp;
        for (const auto &point : refer_line_.refer_line)
        {
            point_tmp.header = refer_line_rviz_.header;
            point_tmp.pose = point.pose.pose;
            refer_line_rviz_.poses.emplace_back(point_tmp);
        }
        return refer_line_rviz_;
    }

    // 生成参考线（重载，基于参考线拼接）
    base_msgs::msg::Referline ReferencelineCreator::create_reference_line(const base_msgs::msg::Referline &prev_refer_line,
                                                          const nav_msgs::msg::Path &global_path,
                                                          const geometry_msgs::msg::PoseStamped &target_point)
    {
        // 如果全局路径为空，返回空参考线
        if (global_path.poses.empty())
        {
            return refer_line_;
        }

        // 先在全局路径上找到匹配点
        match_point_index_ = Curve::find_match_point(global_path, last_match_point_index_, target_point);
        last_match_point_index_ = match_point_index_;
        if (match_point_index_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "寻找匹配点失败");
            return refer_line_;
        }

        // 根据配置切换：0=普通模式，1=拼接模式
        const int reference_line_type = reference_line_config_->refer_line().type_;
        if (reference_line_type == 0 || prev_refer_line.refer_line.empty())
        {
            return create_reference_line(global_path, target_point);
        }

        // 在上一帧参考线上找到匹配点
        const int prev_match_index = Curve::find_match_point(prev_refer_line, target_point);
        if (prev_match_index < 0)
        {
            // 匹配失败则退化为普通方式
            return create_reference_line(global_path, target_point);
        }

        // 计算拼接区间（修正后的拼接逻辑）：
        // 1) 复用上一帧：从上一帧“尾部”（prev_match_index向后取80个点作为起点）到“上一帧最后3个点之前”全部直接使用；
        // 2) 新增全局段：当前帧匹配点前方200个点，但跳过前3个点，以保证与上一帧末尾平滑衔接；
        const int global_path_size = global_path.poses.size();
        const int kPrevTailPoints = reference_line_config_->refer_line().back_size_;
        const int kForwardNewPoints = reference_line_config_->refer_line().front_size_;
        const int kSkipOverlapPoints = 3;

        front_index_ = (global_path_size - 1 >= match_point_index_ + kForwardNewPoints)
                       ? (match_point_index_ + kForwardNewPoints)
                       : (global_path_size - 1);

        const int prev_size = static_cast<int>(prev_refer_line.refer_line.size());
        const int prev_tail_start = (0 <= prev_match_index - kPrevTailPoints)
                                      ? (prev_match_index - kPrevTailPoints)
                                      : 0;
        back_index_ = prev_tail_start;

        // 上一帧可复用的末端（去掉最后3个点）
        int prev_keep_end = (prev_size > kSkipOverlapPoints) ? (prev_size - kSkipOverlapPoints - 1) : (prev_size - 1);
        if (prev_keep_end < prev_tail_start)
        {
            prev_keep_end = prev_tail_start - 1; // 无可复用段
        }

        // 填充参考线头
        refer_line_.header.frame_id = reference_line_config_->pnc_map().frame_;
        refer_line_.header.stamp = rclcpp::Clock().now();
        refer_line_.refer_line.clear();

        // 1) 先加入上一帧复用段：从尾部到上一帧最后3个点之前
        for (int i = prev_tail_start; i <= prev_keep_end; ++i)
        {
            refer_line_.refer_line.emplace_back(prev_refer_line.refer_line[i]);
        }

        // 2) 再加入当前帧全局路径新增段：
        //    新段应当紧接在已复用的上一帧末尾之后，避免重复与顺序错乱。
        //    已复用的“向前段”长度（相对上一帧匹配点）为：max(0, prev_keep_end - prev_match_index + 1)
        int reused_forward_len = prev_keep_end >= prev_match_index ? (prev_keep_end - prev_match_index + 1) : 0;
        int start_idx_on_global = match_point_index_ + kSkipOverlapPoints + reused_forward_len;
        if (start_idx_on_global < match_point_index_ + kSkipOverlapPoints)
        {
            start_idx_on_global = match_point_index_ + kSkipOverlapPoints;
        }
        if (start_idx_on_global > front_index_)
        {
            start_idx_on_global = front_index_;
        }

        base_msgs::msg::ReferlinePoint point_tmp;
        for (int i = start_idx_on_global; i <= front_index_; ++i)
        {
            point_tmp.pose = global_path.poses[i];
            refer_line_.refer_line.emplace_back(point_tmp);
        }

        // 平滑整条参考线
        reference_line_smoother_->smooth_reference_line(refer_line_);

        // 计算投影点参数
        Curve::cal_projection_param(refer_line_);

        RCLCPP_INFO(rclcpp::get_logger("reference_line"),
                    "拼接参考线创建完成（修正逻辑），prev_keep=[%d,%d], global_added=[%d,%d], 尺寸=%ld",
                    prev_tail_start, prev_keep_end, start_idx_on_global, front_index_,
                    refer_line_.refer_line.size());

        return refer_line_;
    }

}// namespace Planning
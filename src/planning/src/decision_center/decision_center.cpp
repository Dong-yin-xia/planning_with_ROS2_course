/*
文件名: 决策中心
作者: C哥智驾说
完成时间: 2024.12

编译类型: 动态库
依赖: ROS2内部库:
        rclcpp
    外部库:
        config_reader
        vehicle_info

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "decision_center.h"
#include <algorithm>

namespace Planning
{
    DecisionCenter::DecisionCenter()
    {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "决策中心已创建");

        // 读取配置文件
        decision_config_ = std::make_unique<ConfigReader>();
        decision_config_->read_decision_config();
    }

    // 路径决策
    void DecisionCenter::make_path_decision(const std::shared_ptr<VehicleBase> &car, 
                                            const std::vector<std::shared_ptr<VehicleBase>> &obses) 
    {
        if (obses.empty()) // 没有障碍物
        {
            return;
        }
        
        // 初始化
        sl_points_.clear();
        const double left_bound_l = decision_config_->pnc_map().road_half_width_ * 1.5;// 道路左边界
        const double right_bound_l = -decision_config_->pnc_map().road_half_width_ / 2.0;// 道路右边界
        const double dis_time = static_cast<double>(decision_config_->local_path().path_size_ -50);// 开始考虑障碍物的范围
        const double least_length = std::max(car->ds_dt() * dis_time, 30.0); // 最小变道距离
        const double referline_end_length = decision_config_->refer_line().front_size_ *
                                            decision_config_->pnc_map().segment_len_;  // 参考线前段的长度的最大值
        SLPoint p;

        // 针对每个障碍物计算变道点位
        for (const auto &obs : obses) 
        {
            const double obs_dis_s = obs->s() - car->s();  // 与障碍物的距离
            if (obs_dis_s > referline_end_length || obs_dis_s < -least_length) // 如果障碍物在参考线末端的前面，即使接近地图终点，参考线变短，也要考虑最长距离，防止撞上
            {
                continue;
            }

            if (obs->l() > right_bound_l && obs->l() < left_bound_l &&  // 如果障碍物在车道横向中间
                fabs(obs->dl_dt()) < min_speed && obs->ds_dt() < car->ds_dt() / 2.0)  // 侧向速度为0，纵向速度小于主车的一半
                {
                    p.s_ = obs->s() + obs->ds_dt() * obs_dis_s / (car->ds_dt() - obs->ds_dt()); // 虚拟障碍物速度
                    const double obs_left_bound_l = obs->l() + obs->width() / 2.0; // 障碍物左边界
                    const double obs_right_bound_l = obs->l() - obs->width() / 2.0; // 障碍物右边界
                    const double left_width = left_bound_l - obs_left_bound_l; // 左边界宽度
                    const double right_width = obs_right_bound_l - right_bound_l; // 右边界宽度

                    if (left_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0)// 如果左边宽度可以通过
                    {
                        p.l_ = (left_bound_l + obs_left_bound_l) / 2.0;
                        p.type_ = static_cast<int>(SLPointType::LEFT_PASS);
                        sl_points_.emplace_back(p);
                    }
                    else 
                    {
                        if (right_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0)  // 如果右边宽度可以通过
                        {
                            p.l_ = (right_bound_l + obs_right_bound_l) / 2.0;
                            p.type_ = static_cast<int>(SLPointType::RIGHT_PASS);
                            sl_points_.emplace_back(p);
                        }
                        else // 两边宽度都不能通过
                        {
                            p.l_ = 0.0;
                            p.s_ = obs->s() - decision_config_->decision().safe_dis_s_;
                            p.type_ = static_cast<int>(SLPointType::STOP);
                            sl_points_.emplace_back(p);
                            RCLCPP_INFO(rclcpp::get_logger("decision_center"), "两边都不能通过，需要停车,p:(s=%.2f,l=%.2f)", p.s_, p.l_);
                            break; // 更前方的障碍物不用考虑
                        
                        }
                    
                    }
                    
                    
                }
        }

        if (sl_points_.empty()) // 没有变道点位
        {
            return;
        }

        // 头尾的处理
        SLPoint p_start; // 整个过程的起点
        p_start.s_ = sl_points_[0].s_ - least_length;
        p_start.l_ = 0.0;
        p_start.type_ = static_cast<int>(SLPointType::START);
        sl_points_.emplace(sl_points_.begin(), p_start); // 头插

        if (sl_points_.back().type_ != static_cast<int>(SLPointType::STOP)) // 如果最后一个点位不是停止
        {
            SLPoint p_end; // 整个过程的终点
            p_end.s_ = sl_points_.back().s_ + least_length;
            p_end.l_ = 0.0;
            p_end.type_ = static_cast<int>(SLPointType::END);
            sl_points_.emplace_back(p_end); // 尾插
        }
        
    }

}// namespace Planning
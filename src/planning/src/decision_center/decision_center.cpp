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
#include <cmath>
#include <rclcpp/logger.hpp>

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
    // 传入参数：表示主车的当前信息。std::shared_ptr 允许多个对象共享同一资源，const & 确保了不会修改主车对象本身，只是读取其状态
    // 表示检测到的所有障碍物的列表
    void DecisionCenter::make_path_decision(const std::shared_ptr<VehicleBase> &car, 
                                            const std::vector<std::shared_ptr<VehicleBase>> &obses) 
    {
        if (obses.empty()) // 没有障碍物
        {
            return;
        }
        
        // 初始化
        // 清空 DecisionCenter 内部存储的 sl_points_ 列表。这是为了确保每次调用make_path_decision都能从一个干净的状态开始，只包含当前帧的决策点。
        sl_points_.clear();
        const double left_bound_l = decision_config_->pnc_map().road_half_width_ * 1.5;// 道路左边界
        const double right_bound_l = -decision_config_->pnc_map().road_half_width_ / 2.0;// 道路右边界
        const double dis_time = static_cast<double>(decision_config_->local_path().path_size_ -50);// 开始考虑障碍物的范围
        // 取两者中的较大值。这个逻辑确保了即使车速很低，也至少有30米的距离来执行变道或停车决策，避免了过近的危险决策。
        const double least_length = std::max(car->ds_dt() * dis_time, 30.0); // 最小变道距离
        const double referline_end_length = decision_config_->refer_line().front_size_ *
                                            decision_config_->pnc_map().segment_len_;  // 参考线前段的长度的最大值
        SLPoint p;

        // 针对每个障碍物计算变道点位
        for (const auto &obs : obses) 
        {
            const double obs_dis_s = obs->s() - car->s();  // 与障碍物的纵向距离
            if (obs_dis_s > referline_end_length || obs_dis_s < -least_length) // 如果障碍物在参考线末端的前面，即使接近地图终点，参考线变短，也要考虑最长距离，防止撞上
            {
                continue;
            }

            if (obs->l() > right_bound_l && obs->l() < left_bound_l &&  // 如果障碍物在车道横向中间（大于右边界，小于左边界）
                fabs(obs->dl_dt()) < min_speed && obs->ds_dt() < car->ds_dt() / 2.0)  // 侧向速度为0，纵向速度小于主车的一半
                {
                    p.s_ = obs->s() + obs->ds_dt() * obs_dis_s / (car->ds_dt() - obs->ds_dt()); // 预测交汇点的s坐标
                    const double obs_left_bound_l = obs->l() + obs->width() / 2.0; // 障碍物左边界
                    const double obs_right_bound_l = obs->l() - obs->width() / 2.0; // 障碍物右边界
                    const double left_width = left_bound_l - obs_left_bound_l; // 左边界宽度
                    const double right_width = obs_right_bound_l - right_bound_l; // 右边界宽度

                    if (left_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0)// 如果左边宽度可以通过
                    {
                        // 简单地取道路左边界和障碍物左边界的中点，作为变道的横向目标点
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
        // sl_points_[0].s_: 这个值代表了当前决策中心计算出的第一个需要采取行动的SL点的纵向S坐标
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

    void DecisionCenter::make_speed_decision(const std::shared_ptr<VehicleBase> &car, 
                                            const std::vector<std::shared_ptr<VehicleBase>> &obses) 
    {
        if (obses.empty()) // 没有障碍物
        {
            return;
        }

        st_points_.clear();
        const double ori_dis_time = static_cast<double>(decision_config_->local_speeds().speed_size_ -50);// 开始考虑障碍物的时间，50帧
        const double ori_dis = ori_dis_time * decision_config_->main_car().speed_ori_;// 开始考虑障碍物的距离，50米
        const double real_brake_time = (ori_dis_time + static_cast<double>(decision_config_->local_speeds().speed_size_)) / 2.0; // 50 +100 /2 = 75帧
        
        STPoint p;

        // 针对每个障碍物计算变速点位
        for (const auto &obs : obses) 
        {
            const double obs_dis_s = obs->s_2path() + car->speed(); // 主车与障碍物的距离（沿路径）
            const double follow_safe_dis = obs->ds_dt_2path() * 50.0; // 跟车安全距离
            if (obs_dis_s > ori_dis || // 如果车辆还没有进入考虑范围
                obs_dis_s < -decision_config_->decision().safe_dis_s_) // 或者车辆在障碍物前方
            {
                continue;
            }

            double t_in; // 切入时间
            double t_out; // 切出时间

            if (fabs(obs->l2path()) < obs->width() / 2.0)// 如果障碍物已经占据了路径
            {
                if (fabs(obs->dl_ds_2path()) < min_speed)  // 并且侧向速度很低
                {
                    if (obs->ds_dt_2path() > car->speed() + 0.5) // 如果障碍物纵向车速大于主车
                    {
                        continue;
                    }
                    // 计算初始s t，切入时间，切出时间
                    obs->update_t0(); // 计时
                    p.t0_ = obs->t0();
                    p.s0_ = obs_dis_s + obs->ds_dt_2path() * p.t0_ - ori_dis;
                    t_in = 0.0; // 切入时间定为0帧
                    t_out = decision_config_->local_speeds().speed_size_; // 切出时间定为计时最后，100帧
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "-----obs_dis_s = %.2f, p.t0_ = %.2f, p.s0_ = %.2f, t_in = %.2f, t_out = %.2f", 
                                                                        obs_dis_s, p.t0_, p.s0_, t_in, t_out);
                    // 计算st点
                    p.t_ = p.t0_ + real_brake_time;
                    p.s_2path_ = obs_dis_s - follow_safe_dis + obs->ds_dt_2path() * p.t_;
                    p.ds_dt_2path_ = obs->ds_dt_2path();
                    p.type_ = static_cast<int>(STPointType::STOP);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "---stop or follow obs, p:(s=%.2f, t=%.2f, ds_dt=%.2f)", 
                                                                            p.s_2path_, p.t_, p.ds_dt_2path_);
                    obs->update_t_in_out(p.t_, t_in, t_out); // 更新障碍物的时间
                    break; // 只处理一个障碍物
                }
            }
        

            else // 障碍物没有占据路径
            {
                 if (fabs(obs->dl_ds_2path()) < min_speed) // 如果侧向没有速度
                 {
                    continue;
                 }   
                 
                 if (decision_config_->main_car().speed_ori_ < min_speed) // 如果主车初速度很低，则无需减速或强行
                 {
                    continue;
                 }

                 const double car_dis_time = obs_dis_s/decision_config_->main_car().speed_ori_; // 主车中心到达障碍物S位置的时间
                 const double obs_dis_time = (0.0 - obs->l2path()) / obs->dl_dt_2path(); // 障碍物中心到达路径的时间
                 if (obs_dis_time < 0.0)
                 {
                    continue;
                 }

                 // 确定初始的s t
                 obs->update_t0(); // 计时
                 p.t0_ = obs->t0();
                 p.s0_ = obs_dis_s - ori_dis;

                 // 计算切入 切出时间
                 const double delta_t = decision_config_->decision().safe_dis_s_ / decision_config_->main_car().speed_ori_; // 安全时间阈值
                 const double half_through_time = fabs(obs->length() / 2.0 / obs->dl_dt_2path()); // 半个车身穿过路径的时间
                 t_in = obs_dis_time - half_through_time;
                 t_out = obs_dis_time + half_through_time;
                 RCLCPP_INFO(rclcpp::get_logger("decision_center"), "----obs_dis_s = %.2f, p.t0 = %.2f, p.s0 = %.2f, car_dis_time = %.2f, obs_dis_time = %.2f, t_in = %.2f, t_out = %.2f, delta_t = %.2f",
                                                obs_dis_s, p.t0_, p.s0_, car_dis_time, obs_dis_time, t_in, t_out, delta_t);

                // 计算S T 点
                if (car_dis_time > obs_dis_time && car_dis_time < t_out + delta_t) // 让行
                {
                    p.t_ = t_out;
                    p.s_2path_ = obs_dis_s - decision_config_->decision().safe_dis_s_;
                    p.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
                    p.type_ = static_cast<int>(STPointType::GIVE_WAY);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "---give way obs, p:(s=%.2f, t=%.2f, ds_dt=%.2f)", 
                                                                          p.s_2path_, p.t_, p.ds_dt_2path_);
                    // obs->update_t_in_out(p.t_, t_in, t_out); // 更新障碍物的时间
                    // break; // 只处理一个障碍物
                }
                else if (car_dis_time < obs_dis_time && car_dis_time < t_out + delta_t) // 抢行
                {
                    p.t_ = t_out;
                    p.s_2path_ = obs_dis_s + decision_config_->decision().safe_dis_s_;
                    p.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
                    p.type_ = static_cast<int>(STPointType::RUSH_OUT);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "---rush out obs, p:(s=%.2f, t=%.2f, ds_dt=%.2f)", 
                                                                          p.s_2path_, p.t_, p.ds_dt_2path_);
                    // obs->update_t_in_out(p.t_, t_in, t_out); // 更新障碍物的时间
                    // break; // 只处理一个障碍物
                }
                obs->update_t_in_out(p.t_, t_in, t_out);

            }
        }
        if (st_points_.empty()) // 没有变速点位
        {
            return;
        }

        STPoint p_start; // 整个过程的起点
        p_start.t_ = st_points_[0].t0_;
        p_start.s_2path_ = st_points_[0].s0_;
        p_start.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
        p_start.type_ = static_cast<int>(STPointType::START);
        st_points_.emplace(st_points_.begin(), p_start); // 头插

        STPoint p_end; // 整个过程的终点
        p_end.t_ = decision_config_->local_speeds().speed_size_;
        p_end.s_2path_ = st_points_.back().s_2path_ + st_points_.back().ds_dt_2path_ * (p_end.t_ - st_points_.back().t_);
        p_end.ds_dt_2path_ = st_points_.back().ds_dt_2path_;
        p_end.type_ = static_cast<int>(STPointType::END);
        st_points_.emplace_back(p_end); // 尾插

        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "---p_start:(t=%.2f, s=%.2f, ds_dt=%.2f), p_end:(t=%.2f, s=%.2f, ds_dt=%.2f)", 
                                                                          p_start.t_, p_start.s_2path_, p_start.ds_dt_2path_, 
                                                                          p_end.t_, p_end.s_2path_, p_end.ds_dt_2path_);
        
    }

}// namespace Planning
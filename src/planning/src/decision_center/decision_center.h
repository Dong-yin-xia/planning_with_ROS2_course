#ifndef DECISION_CENTER_H_
#define DECISION_CENTER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "main_car_info.h"
#include "obs_car_info.h"
#include "vehicle_info_base.h"
#include <memory>
#include <vector>

namespace Planning
{
    const double min_speed = 0.03; // 最小速度
    enum class SLPointType // 变道点位类型
    {
        LEFT_PASS,  // 从左绕行
        RIGHT_PASS, // 从右绕行
        STOP,       // 停止
        START,      // 整个过程起点
        END         // 整个过程终点
    };

    enum class STPointType // 速度点位类型
    {
        GIVE_WAY, // 让行
        RUSH_OUT, // 抢行
        STOP,     // 停止
        START,    // 整个过程起点
        END       // 整个过程终点
    };

    struct SLPoint // 变道点位
    {
        double s_ = 0.0; // 纵向距离
        double l_ = 0.0; // 横向距离
        int type_ = 0; // 类型
    };

    struct STPoint // 变速点位
    {
        double t_ = 0.0; // 时间
        double s_2path_ = 0.0; // 路径投影的s
        double ds_dt_2path_ = 0.0; // 路径投影的速度
        double t0_ = 0.0; // 初始时间
        double s0_ = 0.0; // 初始位置
        int type_ = 0; // 类型
    };

    class DecisionCenter // 决策中心
    {
    public:
        DecisionCenter();
        void make_path_decision(const std::shared_ptr<VehicleBase> &car, 
                                const std::vector<std::shared_ptr<VehicleBase>> &obses);  // 路径决策
        void make_speed_decision(const std::shared_ptr<VehicleBase> &car, 
                                const std::vector<std::shared_ptr<VehicleBase>> &obses);  // 速度决策
        inline std::vector<SLPoint> sl_points() const { return sl_points_; }  // 获取变道点位
        inline std::vector<STPoint> st_points() const { return st_points_; }  // 获取变速点位
    private:
        std::unique_ptr<ConfigReader> decision_config_; // 决策配置读取器
        std::vector<SLPoint> sl_points_; // 变道点位
        std::vector<STPoint> st_points_; // 变速点位
    };
} // namespace Planning
#endif // DECISION_CENTER_H_

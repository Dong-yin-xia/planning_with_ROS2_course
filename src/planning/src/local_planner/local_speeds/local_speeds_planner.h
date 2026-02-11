#ifndef LOCAL_SPEED_PLANNER_H_
#define LOCAL_SPEED_PLANNER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "base_msgs/msg/local_speeds_point.hpp"


#include "config_reader.h"
#include "polynomial_curve.h"
#include "decision_center.h"
#include "local_speeds_smoother.h"
#include <base_msgs/msg/detail/local_speeds__struct.hpp>
#include <memory>

namespace Planning
{
    using base_msgs::msg::LocalSpeeds;
    using base_msgs::msg::LocalSpeedsPoint;

    class LocalSpeedsPlanner // 速度规划器
    {
    public:
        LocalSpeedsPlanner();
        LocalSpeeds cal_speed(const std::shared_ptr<DecisionCenter> &decision); // 计算速度

    private:
        void init_local_speeds(); // 初始化速度

    public:
        inline LocalSpeeds local_speeds() const {return local_speeds_;} // 获取速度
        
    private:
        std::unique_ptr<ConfigReader> local_speeds_config_; // 速度配置读取器
        LocalSpeeds local_speeds_; // 速度
        std::shared_ptr<LocalSpeedsSmoother> local_speeds_smoother_; // 速度平滑器


    };
} // namespace Planning
#endif // LOCAL_SPEED_PLANNER_H_
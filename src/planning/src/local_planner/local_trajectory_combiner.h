#ifndef LOCAL_TRAJECTORY_COMBINER_H_
#define LOCAL_TRAJECTORY_COMBINER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "base_msgs/msg/local_trajectory_point.hpp"

#include "config_reader.h"
#include <base_msgs/msg/detail/local_trajectory__struct.hpp>
#include <memory>

namespace Planning
{
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::LocalSpeeds;
    using base_msgs::msg::LocalTrajectory;
    using base_msgs::msg::LocalTrajectoryPoint;

    class LocalTrajectoryCombiner // 轨迹合成器
    {
    public:
        LocalTrajectoryCombiner();
        LocalTrajectory combine_trajectory(const LocalPath &path, const LocalSpeeds &speeds);
        inline LocalTrajectory local_trajectory() const { return local_trajectory_; }  // 获取局部轨迹

    private:
        std::unique_ptr<ConfigReader> trajectory_config_; // 轨迹配置
        LocalTrajectory local_trajectory_; // 局部轨迹
    };
} // namespace Planning
#endif // LOCAL_TRAJECTORY_COMBINER_H_
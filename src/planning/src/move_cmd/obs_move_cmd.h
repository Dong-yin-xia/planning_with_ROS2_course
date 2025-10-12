#ifndef OBS_MOVE_CMD_H_
#define OBS_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <memory>
#include <vector>
#include "config_reader.h"
#include "obs_car_info.h"
#include "vehicle_info_base.h"

namespace Planning
{
    using namespace std::chrono_literals;
    using geometry_msgs::msg::TransformStamped;
    using tf2_ros::TransformBroadcaster;

    struct ObsParam  // 障碍物参数
    {
        std::shared_ptr<VehicleBase> obs_;  // 障碍物
        std::shared_ptr<TransformBroadcaster> obs_broadcaster_;  // 坐标广播器
        double pose_x_ = 0.0;  // x坐标
        double pose_y_ = 0.0;  // y坐标  
        double theta_ = 0.0;  // 航向角
        double speed_ = 0.0;  // 速度
    };

    

    class ObsMoveCmd : public rclcpp::Node // 障碍物运动指令
    {
    public:
        ObsMoveCmd();

    private:
        void obs_broadcast_tf();  // 广播障碍物坐标变换

    private:
        std::unique_ptr<ConfigReader> obs_move_cmd_config_;  // 运动指令配置
        std::vector<ObsParam> obses_param;  // 所有障碍物参数
        rclcpp::TimerBase::SharedPtr timer_;  // 障碍物定时器
    };
} // namespace Planning
#endif // OBS_MOVE_CMD_H_
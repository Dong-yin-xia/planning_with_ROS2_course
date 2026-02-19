#ifndef VEHICLE_BASE_H_
#define VEHICLE_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/local_trajectory_point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "config_reader.h"
#include "curve.h"
#include <base_msgs/msg/detail/local_trajectory_point__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <string>

namespace Planning
{
    using base_msgs::msg::LocalTrajectoryPoint;
    using base_msgs::msg::Referline;
    using geometry_msgs::msg::PoseStamped;
    using geometry_msgs::msg::TransformStamped;
    using nav_msgs::msg::Path;

    class VehicleBase // 车辆基类
    {
    public:
        // 更新参数
        inline void update_location(const PoseStamped &loc) { loc_point_ = loc; }
        inline void update_cartesian_info(const LocalTrajectoryPoint  &point)
        {
            theta_ = point.path_point.theta;
            kappa_ = point.path_point.kappa;
            dkappa_ = point.path_point.dkappa;
            speed_ = point.speed_point.speed;
            acceleration_ = point.speed_point.acceleration;
            dacceleration_ = point.speed_point.dacceleration;

        }

        // 每调用一次就向前推进一帧时间（正向累加）
        inline void update_t0() { t0_ -= 1.0; }
        inline void update_t_in_out(const double &t, const double &t_in, const double &t_out)
        {
            t_ = t;
            t_in_ = t_in;
            t_out_ = t_out;
        }
        
        // 定位点转frenet
        virtual void vehicle_cartesian_to_frenet(const Referline &refer_line) = 0; // 定位点在参考线上的投影点参数
        virtual void vehicle_cartesian_to_frenet_2path(const LocalPath &local_path,
                                                       const Referline &refer_line,
                                                       const std::shared_ptr<VehicleBase> &car) = 0; // 定位点在局部路径上的投影点参数

        // 基本属性
        inline std::string child_frame() const { return child_frame_; }
        inline double length() const { return length_; }
        inline double width() const { return width_; }
        inline int id() const { return id_; }

        // 笛卡尔参数
        inline PoseStamped loc_point() const { return loc_point_; }
        inline double theta() const { return theta_; }
        inline double kappa() const { return kappa_; }
        inline double dkappa() const { return dkappa_; }
        inline double speed() const { return speed_; }
        inline double acceleration() const { return acceleration_; }
        inline double dacceleration() const { return dacceleration_; }

        // 向参考线投影的frenet参数
        inline double s() const { return s_; }
        inline double l() const { return l_; }
        inline double ds_dt() const { return ds_dt_; }
        inline double dl_ds() const { return dl_ds_; }
        inline double dl_dt() const { return dl_dt_; }
        inline double dds_dt() const { return dds_dt_; }
        inline double ddl_ds() const { return ddl_ds_; }
        inline double ddl_dt() const { return ddl_dt_; }

        // 向路径投影的frenet参数
        inline double s_2path() const { return s_2path_; }
        inline double l2path() const { return l_2path_; }
        inline double ds_dt_2path() const { return ds_dt_2path_; }
        inline double dl_ds_2path() const { return dl_ds_2path_; }
        inline double dl_dt_2path() const { return dl_dt_2path_; }
        inline double dds_dt_2path() const { return dds_dt_2path_; }
        inline double ddl_ds_2path() const { return ddl_ds_2path_; }
        inline double ddl_dt_2path() const { return ddl_dt_2path_; }

        // 时间参数
        inline double t0() const { return t0_; }
        inline double t() const { return t_; }
        inline double t_in() const { return t_in_; }
        inline double t_out() const { return t_out_; }
    
        // 虚析构
        virtual ~VehicleBase(){}

    protected:
        // 基本属性
        std::unique_ptr<ConfigReader> vehicle_config_;  // 车辆配置
        std::string child_frame_; //坐标名
        double length_ = 0.0; // 长度
        double width_ = 0.0; // 宽度
        int id_ = 0; // 序号

        // 笛卡尔参数
        PoseStamped loc_point_;  // 定位点
        double theta_ = 0.0; // 航向角
        double kappa_ = 0.0; // 曲率
        double dkappa_ = 0.0;  // 曲率变化率
        double speed_ = 0.0; // 速度
        double acceleration_ = 0.0; // 加速度
        double dacceleration_ = 0.0; // 加速度变化率

        // 向参考线投影的frenet参数
        double s_ = 0.0;
        double l_ = 0.0;
        double ds_dt_ = 0.0;
        double dl_ds_ = 0.0;
        double dl_dt_ = 0.0;
        double dds_dt_ = 0.0;
        double ddl_ds_ = 0.0;
        double ddl_dt_ = 0.0; 

        // 向路径投影的frenet参数
        double s_2path_ = 0.0;
        double l_2path_ = 0.0;
        double ds_dt_2path_ = 0.0;
        double dl_ds_2path_ = 0.0;
        double dl_dt_2path_ = 0.0;
        double dds_dt_2path_ = 0.0;
        double ddl_ds_2path_ = 0.0;
        double ddl_dt_2path_ = 0.0;

        // 时间参数
        double t0_ = 0.0; // 起始时间
        double t_ = 0.0; // 当前时间
        double t_in_ = 0.0; // 障碍物进入路径时间
        double t_out_ = 0.0; // 障碍物退出路径时间

    };
} // namespace Planning
#endif // VEHICLE_BASE_H_
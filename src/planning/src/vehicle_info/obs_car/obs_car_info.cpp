/*
文件名: 障碍物信息
作者: C哥智驾说
完成时间: 2024.12

编译类型: 动态库
依赖: ROS2内部库:
        rclcpp
        nav_msgs
        geometry_msgs
        tf2
    外部库:
        base_msgs
        config_reader
        mathlibs

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "obs_car_info.h"

namespace Planning
{
    ObsCar::ObsCar(const int &id) // 障碍物信息
    {
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "障碍物车辆已创建");

        // 读取配置文件
        vehicle_config_ = std::make_unique<ConfigReader>();
        vehicle_config_->read_vehicles_config();

        // 更新基本属性
        child_frame_ = vehicle_config_->obs_pair()[id].frame_;
        length_ = vehicle_config_->obs_pair()[id].length_;
        width_ = vehicle_config_->obs_pair()[id].width_;
        theta_ = vehicle_config_->obs_pair()[id].pose_theta_;
        speed_ = vehicle_config_->obs_pair()[id].speed_ori_;
        id_ = id;

        // 初始化定位点
        tf2::Quaternion qtn;
        qtn.setRPY(0.0, 0.0, theta_);
        loc_point_.header.frame_id = vehicle_config_->pnc_map().frame_;
        loc_point_.header.stamp = rclcpp::Clock().now();
        loc_point_.pose.position.x = vehicle_config_->obs_pair()[id].pose_x_;
        loc_point_.pose.position.y = vehicle_config_->obs_pair()[id].pose_y_;
        loc_point_.pose.position.z = 0.0;
        loc_point_.pose.orientation.x = qtn.getX();
        loc_point_.pose.orientation.y = qtn.getY();
        loc_point_.pose.orientation.z = qtn.getZ();
        loc_point_.pose.orientation.w = qtn.getW();
    }

        // 定位点转frenet
    void ObsCar::vehicle_cartesian_to_frenet(const Referline &refer_line)  // 定位点在参考线上的投影点参数
    {
        double rs, rx, ry, rtheta, rkappa, rdkappa;

        // 计算定位点在参考线上的投影点
        Curve::find_projection_point(refer_line, loc_point_, rs, rx, ry, rtheta, rkappa, rdkappa);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "障碍物的投影点: rs = %.2f, rx = %.2f, ry = %.2f, rtheta = %.2f, rkappa = %.2f, rdkappa = %.2f", rs, rx, ry, rtheta, rkappa, rdkappa);

        // 计算定位点在frence坐标下的参数
        Curve::cartesian_to_frenet(loc_point_.pose.position.x, loc_point_.pose.position.y,
                                   theta_, speed_, acceleration_, kappa_, 
                                   rs, rx, ry, rtheta, rkappa, rdkappa, 
                                   s_, ds_dt_, dds_dt_, l_, dl_ds_, dl_dt_, ddl_ds_, ddl_dt_);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "障碍物的frence坐标下的参数: s = %.2f, ds_dt = %.2f, dds_dt = %.2f, l = %.2f, dl_ds = %.2f, dl_dt = %.2f, ddl_ds = %.2f, ddl_dt = %.2f", 
                                                       s_, ds_dt_, dds_dt_, l_, dl_ds_, dl_dt_, ddl_ds_, ddl_dt_);
    
    }

    // 向路径投影
    void ObsCar::vehicle_cartesian_to_frenet_2path(const LocalPath &local_path,
                                                    const Referline &refer_line,
                                                    const std::shared_ptr<VehicleBase> &car)
    {
        // 计算路径起点终点在参考线上的下标
        const double path0_index = Curve::find_match_point(refer_line, local_path.local_path[0].pose);
        const double path_end_index = Curve::find_match_point(refer_line, local_path.local_path.back().pose);

        // 当障碍物在参考线上的S值超出路径首尾范围时
        if (s_ > refer_line.refer_line[path_end_index].rs || s_ < refer_line.refer_line[path0_index].rs) // 超出路径前段或后段
        {
            // 直接用参考线投影的值近似替代路径投影
            s_2path_ = s_ - refer_line.refer_line[path0_index].rs;
            ds_dt_2path_ = ds_dt_;
            l_2path_ = l_ - car->l();
            dl_ds_2path_ = dl_ds_;
            dl_dt_2path_ = dl_dt_;
            dds_dt_2path_ = dds_dt_;
            ddl_ds_2path_ = ddl_ds_;
            ddl_dt_2path_ = ddl_dt_;
            
            RCLCPP_INFO(rclcpp::get_logger("vehicle"), "障碍物汽车笛卡尔转frenet到路径投影：path0_rs = %.2f, path_end_rs = %.2f, s2path = %.2f, ds_dt_2path = %.2f, l2path = %.2f, dl_ds_2path = %.2f, dl_dt_2path = %.2f", 
                                                  refer_line.refer_line[path0_index].rs, refer_line.refer_line[path_end_index].rs, s_2path_, ds_dt_2path_, l_2path_, dl_ds_2path_, dl_dt_2path_);
            return;
        }

        double rs, rx, ry, rtheta, rkappa, rdkappa;

        // 计算定位点在路径上的投影点参数
        Curve::find_projection_point(local_path, loc_point_, rs, rx, ry, rtheta, rkappa, rdkappa);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "障碍物汽车笛卡尔转frenet到路径投影：rs = %.2f, rx = %.2f, ry = %.2f, rtheta = %.2f, rkappa = %.2f, rdkappa = %.2f", rs, rx, ry, rtheta, rkappa, rdkappa);

        // 计算定位点在frenet坐标下的参数
        Curve::cartesian_to_frenet(loc_point_.pose.position.x, loc_point_.pose.position.y,
                                   theta_, speed_, acceleration_, kappa_, 
                                   rs, rx, ry, rtheta, rkappa, rdkappa, 
                                   s_2path_, ds_dt_2path_, dds_dt_2path_, l_2path_, dl_ds_2path_, dl_dt_2path_, ddl_ds_2path_, ddl_dt_2path_);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "障碍物汽车笛卡尔转frenet到路径投影：s2path = %.2f, ds_dt_2path = %.2f,l2path = %.2f, dl_ds_2path = %.2f, dl_dt_2path = %.2f", 
                                                  s_2path_, ds_dt_2path_, l_2path_, dl_ds_2path_, dl_dt_2path_);
        return;
    }

} // namespace Planning
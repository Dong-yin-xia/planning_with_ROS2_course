/*
文件名: 曲线计算
作者: C哥智驾说
完成时间: 2024.12

编译类型: 动态库
依赖: ROS2内部库:
        rclcpp
        base_msgs
        nav_msgs
        geometry_msgs

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "curve.h"
#include <cmath>

namespace Planning
{
        
        // 归一化角度,约束到[-pi, pi]
        double Curve::NormalizeAngle(const double &angle)
        {
            double a = std::fmod(angle + M_PI, 2.0 * M_PI);
            if (a < 0.0) {
                a += (2.0 * M_PI);
            }
            return a - M_PI;
        }

        // 笛卡尔转自然坐标系
        void Curve::cartesian_to_frenet(const double &x, const double &y, const double &theta,
                                        const double &speed, const double &a, const double &kappa, // 输入1：目标点在笛卡尔下的参数 x y theta speed a
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2：目标点在参考线的投影点在笛卡尔下的参数 rs rx ry rtheta rkappa rdkappa
                                        double &s, double &ds_dt, double &dds_dt,
                                        double &l, double &dl_ds, double &dl_dt, double &ddl_ds, double &ddl_dt) // 输出：目标点在自然坐标系下的参数 s ds_dt dds_dt l dl_ds dl_dt ddl_ds ddl_dt
        {
                // 计算s
                s = rs;

                // 计算l
                const double dx = x - rx;
                const double dy = y - ry;
                
                const double cos_theta_r = std::cos(rtheta);
                const double sin_theta_r = std::sin(rtheta);
                
                const double cross_r_x = cos_theta_r * dy - sin_theta_r * dx;

                l = std::copysign(std::hypot(dx, dy), cross_r_x);

                // 计算l' = dl/ds
                const double delta_theta = theta - rtheta;
                const double tan_delta_theta = std::tan(delta_theta);
                const double cos_delta_theta = std::cos(delta_theta);
                const double sin_delta_theta = std::sin(delta_theta);
                const double one_minus_kappa_l = 1.0 - rkappa * l;
                dl_ds = one_minus_kappa_l * tan_delta_theta;

                // 计算l'' = d(dl)/ds
                const double kappa_l_prime = rdkappa * l + rkappa * dl_ds;
                const double delta_theta_prime = one_minus_kappa_l / cos_delta_theta * kappa - rkappa;
                ddl_ds = -kappa_l_prime * tan_delta_theta + 
                         one_minus_kappa_l / (cos_delta_theta * cos_delta_theta) * delta_theta_prime;

                // 计算ds/dt
                ds_dt = speed * cos_delta_theta / one_minus_kappa_l;

                // 计算d(ds)/dt
                dds_dt = (a * cos_delta_theta - (ds_dt * ds_dt) * (dl_ds * delta_theta_prime - kappa_l_prime)) / one_minus_kappa_l;

                // 计算dl_dt
                dl_dt = speed * sin_delta_theta;

                // 计算ddl_dt
                ddl_dt = a * sin_delta_theta;

        }

        // 自然坐标系转笛卡尔
        void Curve::frenet_to_cartesian(const double &s, const double &ds_dt, const double &dds_dt,
                                        const double &l, const double &dl_ds, const double &ddl_ds, // 输入1：目标点在自然坐标系下的参数 s ds_dt dds_dt l dl_ds dl_dt ddl_ds ddl_dt                                       
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2：目标点在参考线的投影点在笛卡尔下的参数 rs rx ry rtheta rkappa rdkappa
                                        double &x, double &y, double &theta,
                                        double &speed, double &a, double &kappa) // 输出：目标点在笛卡尔下的参数 x y theta speed a
        {
                // 判断 s 和 rs 是否足够近
                if (std::fabs(rs - s) > delta_s_min)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("math"), "s 和 rs 距离太远,无法转换! rs = %.2f, s = %.2f", rs, s);
                    return;
                }

                // 计算X Y
                const double cos_theta_r = std::cos(rtheta);
                const double sin_theta_r = std::sin(rtheta);

                x = rx - sin_theta_r * l;
                y = ry + cos_theta_r * l;

                // 计算theta
                const double one_minus_kappa_l = 1.0 - rkappa * l;
                const double tan_delta_theta = dl_ds / one_minus_kappa_l;
                const double delta_theta = std::atan2(dl_ds, one_minus_kappa_l);
                const double cos_delta_theta = std::cos(delta_theta);
                theta = NormalizeAngle(delta_theta + rtheta);

                // 计算kappa
                const double kappa_l_prime = rdkappa * l + rkappa * dl_ds;
                kappa = ((ddl_ds + kappa_l_prime * tan_delta_theta) * (cos_delta_theta * cos_delta_theta) / one_minus_kappa_l + rkappa) *
                        cos_delta_theta / one_minus_kappa_l;

                // 计算speed
                speed = std::hypot(ds_dt * one_minus_kappa_l, ds_dt * dl_ds);

                // 计算a 
                const double delta_theta_prime = one_minus_kappa_l / cos_delta_theta * kappa - rkappa;
                a = dds_dt * one_minus_kappa_l / cos_delta_theta + 
                   (ds_dt * ds_dt) / cos_delta_theta * 
                   (dl_ds * delta_theta_prime - kappa_l_prime);

                

        }

        // 找匹配点的下标（利用上一帧)
        int Curve::find_match_point(const Path &path, const int &last_match_point_index,const PoseStamped &target_point)
        {
            const int path_size = path.poses.size();
            if (path_size <= 1) {
                return path_size - 1;
            }

            double min_dis = std::numeric_limits<double>::max();
            int closest_index = -1;
            for (int i = 0; i < path_size; i++) //计算车辆定位点到路径点的距离，找出最近的投影点
            {
                double dis = std::hypot(path.poses[i].pose.position.x - target_point.pose.position.x, 
                                        path.poses[i].pose.position.y - target_point.pose.position.y);
                if (dis < min_dis) 
                {
                    if (abs(last_match_point_index - i) > 100) // 如果距离大于100，则跳过,这是特殊情况，如路径交叉
                    {
                        continue;
                    }
                    min_dis = dis;
                    closest_index = i;
                }
            }
            return closest_index;
        }

        // 找匹配点的下标(在参考线上查找匹配点)
        int Curve::find_match_point(const Referline &refer_line, const PoseStamped &target_point)
        {
            const int path_size = refer_line.refer_line.size();
            if (path_size <= 1) {
                return path_size - 1;
            }

            double min_dis = std::numeric_limits<double>::max();
            int closest_index = -1;
            for (int i = 0; i < path_size; i++) //计算车辆定位点到路径点的距离，找出最近的投影点
            {
                double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.x - target_point.pose.position.x, 
                                        refer_line.refer_line[i].pose.pose.position.y - target_point.pose.position.y);
                if (dis < min_dis) 
                {
                    min_dis = dis;
                    closest_index = i;
                }
            }
            return closest_index;
        }

        // 在局部路径上查找匹配点下标 116
        int Curve::find_match_point(const LocalPath &path, const PoseStamped &target_point)
        {
                const int path_size = path.local_path.size();
                if (path_size <= 1) {
                    return path_size - 1;
                }
    
                double min_dis = std::numeric_limits<double>::max();
                int closest_index = -1;
                for (int i = 0; i < path_size; i++) //计算车辆定位点到路径点的距离，找出最近的投影点
                {
                    double dis = std::hypot(path.local_path[i].pose.pose.position.x - target_point.pose.position.x, 
                                            path.local_path[i].pose.pose.position.y - target_point.pose.position.y);
                    if (dis < min_dis) 
                    {
                        min_dis = dis;
                        closest_index = i;
                    }
                }
                return closest_index;
        }

        // 通过rs查找匹配点下标
        int Curve::find_match_point(const Referline &path, const double &rs)
        {
            const int path_size = path.refer_line.size();
            if (path_size <= 1) {
                return path_size - 1;
            }

            double min_delta_s = std::numeric_limits<double>::max();
                int closest_index = -1;
                for (int i = 0; i < path_size; i++) //计算车辆定位点到路径点的距离，找出最近的投影点
                {
                    double delta_s = std::fabs(rs - path.refer_line[i].rs);
                    if (delta_s < min_delta_s) 
                    {
                        min_delta_s = delta_s;
                        closest_index = i;
                    }
                }
                return closest_index;        
        }

        //找到投影点（参考线）
        void Curve::find_projection_point(const Referline &referline, const PoseStamped &target_point, // 输入：参考线和目标点
                                          double &rs, double &rx, double &ry,
                                          double &rtheta, double &rkappa, double &rdkappa) // 输出：目标点在参考线的投影点在笛卡尔下的参数 rs rx ry rtheta rkappa rdkappa
        {
                // 简化：用匹配点近似代替  前提：参考线足够密且足够平滑
                const int match_index = find_match_point(referline, target_point);
                if (match_index < 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("curve"), "找到匹配点失败");
                    return;
                }
                rs = referline.refer_line[match_index].rs;
                rx = referline.refer_line[match_index].pose.pose.position.x;
                ry = referline.refer_line[match_index].pose.pose.position.y;
                rtheta = referline.refer_line[match_index].rtheta;
                rkappa = referline.refer_line[match_index].rkappa;
                rdkappa = referline.refer_line[match_index].rdkappa;
        }

        // 找到投影点（局部路径）
        void Curve::find_projection_point(const LocalPath &local_path, const PoseStamped &target_point, // 输入：局部路径和目标点
                                          double &rs, double &rx, double &ry,
                                          double &rtheta, double &rkappa, double &rdkappa) // 输出：目标点在局部路径的投影点在笛卡尔下的参数 rs rx ry rtheta rkappa rdkappa
        {
                // 简化：用匹配点近似代替  前提：参考线足够密且足够平滑
                const int match_index = find_match_point(local_path, target_point);
                if (match_index < 0)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("curve"), "找到匹配点失败");
                    return;
                }
                rs = local_path.local_path[match_index].rs;
                rx = local_path.local_path[match_index].pose.pose.position.x;
                ry = local_path.local_path[match_index].pose.pose.position.y;
                rtheta = local_path.local_path[match_index].rtheta;
                rkappa = local_path.local_path[match_index].rkappa;
                rdkappa = local_path.local_path[match_index].rdkappa;
        }

        // 计算投影点参数（参考线）
        void Curve::cal_projection_param(Referline &refer_line)
        {
                const int path_size = refer_line.refer_line.size();
                if (path_size < 3) 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("curve"), "参考线点数小于3,太短了,无法计算投影点参数");
                    return;
                }

                // 计算rs
                double rs = 0.0;  // 声明并初始化rs变量
                for (int i = 0; i < path_size; i++) 
                {
                        if (i == 0) 
                        {
                                rs = 0.0;
                        }else 
                        {
                                rs += std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y, 
                                                 refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                        }
                        refer_line.refer_line[i].rs = rs;
                }

                // 计算航向角
                for (int i = 0; i < path_size; i++) 
                {
                        if (i < path_size - 1)
                        {
                                refer_line.refer_line[i].rtheta = 
                                        std::atan2(refer_line.refer_line[i+1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y, 
                                                   refer_line.refer_line[i+1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
                        }else 
                        {
                                refer_line.refer_line[i].rtheta = 
                                        std::atan2(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i-1].pose.pose.position.y, 
                                                   refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i-1].pose.pose.position.x);
                        }
                }


                // 计算曲率
                for (int i = 0; i < path_size; i++) 
                {
                        if (i < path_size - 1)
                        {
                                const double dis = std::hypot(refer_line.refer_line[i+1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y, 
                                                              refer_line.refer_line[i+1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
                                if(dis <= kMathEpsilon)  
                                {
                                        refer_line.refer_line[i].rkappa = 0.0;
                                }else
                                {
                                        refer_line.refer_line[i].rkappa = (refer_line.refer_line[i+1].rtheta - refer_line.refer_line[i].rtheta) / dis;
                                }                            
                        }
                        else 
                        {
                                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y, 
                                                              refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                                if(dis <= kMathEpsilon)  
                                {
                                        refer_line.refer_line[i].rkappa = 0.0;
                                }else
                                {
                                        refer_line.refer_line[i].rkappa = (refer_line.refer_line[i].rtheta - refer_line.refer_line[i-1].rtheta) / dis;
                                }                               
                        }
                }

                // 计算曲率变化率
                for (int i = 0; i < path_size; i++) 
                {
                        if (i < path_size - 1)
                        {
                                const double dis = std::hypot(refer_line.refer_line[i+1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y, 
                                                              refer_line.refer_line[i+1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
                                if(dis <= kMathEpsilon)  
                                {
                                        refer_line.refer_line[i].rkappa = 0.0;
                                }else
                                {
                                        refer_line.refer_line[i].rkappa = (refer_line.refer_line[i+1].rkappa - refer_line.refer_line[i].rkappa) / dis;
                                }                            
                        }
                        else 
                        {
                                const double dis = std::hypot(refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y, 
                                                              refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
                                if(dis <= kMathEpsilon)  
                                {
                                        refer_line.refer_line[i].rkappa = 0.0;
                                }else
                                {
                                        refer_line.refer_line[i].rkappa = (refer_line.refer_line[i].rkappa - refer_line.refer_line[i-1].rkappa) / dis;
                                }                               
                        }
                }

        }

        // 计算投影点参数（局部路径）
        void Curve::cal_projection_param(LocalPath &local_path)
        {
                const int path_size = local_path.local_path.size();
                if (path_size < 3) 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("curve"), "局部路径点数小于3,太短了,无法计算投影点参数");
                    return;
                }

                // 计算rs
                double rs = 0.0;  // 声明并初始化rs变量
                for (int i = 0; i < path_size; i++) 
                {
                        if (i == 0) 
                        {
                                rs = 0.0;
                        }else 
                        {
                                rs += std::hypot(local_path.local_path[i].pose.pose.position.y - local_path.local_path[i - 1].pose.pose.position.y, 
                                                 local_path.local_path[i].pose.pose.position.x - local_path.local_path[i - 1].pose.pose.position.x);
                        }
                        local_path.local_path[i].rs = rs;
                }

                // 计算航向角和曲率，因为之前frenet转笛卡尔已经计算过，直接赋值就行
                for (int i = 0; i < path_size; i++) 
                {
                        local_path.local_path[i].rtheta = local_path.local_path[i].theta;
                        local_path.local_path[i].rkappa = local_path.local_path[i].kappa;
                }

                // 计算曲率变化率
                for (int i = 0; i < path_size; i++) 
                {
                        if (i < path_size - 1)
                        {
                                const double dis = std::hypot(local_path.local_path[i+1].pose.pose.position.y - local_path.local_path[i].pose.pose.position.y, 
                                                              local_path.local_path[i+1].pose.pose.position.x - local_path.local_path[i].pose.pose.position.x);
                                if(dis <= kMathEpsilon)  
                                {
                                        local_path.local_path[i].rkappa = 0.0;
                                }else
                                {
                                        local_path.local_path[i].rkappa = (local_path.local_path[i+1].rkappa - local_path.local_path[i].rkappa) / dis;
                                }                            
                        }
                        else 
                        {
                                const double dis = std::hypot(local_path.local_path[i].pose.pose.position.y - local_path.local_path[i - 1].pose.pose.position.y, 
                                                              local_path.local_path[i].pose.pose.position.x - local_path.local_path[i - 1].pose.pose.position.x);
                                if(dis <= kMathEpsilon)  
                                {
                                        local_path.local_path[i].rkappa = 0.0;
                                }else
                                {
                                        local_path.local_path[i].rkappa = (local_path.local_path[i].rkappa - local_path.local_path[i-1].rkappa) / dis;
                                }                               
                        }
                }
        }
} // namespace Planning
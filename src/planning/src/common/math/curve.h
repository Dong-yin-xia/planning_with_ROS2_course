#ifndef CURVE_H_
#define CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include <base_msgs/msg/detail/local_path__struct.hpp>
#include <base_msgs/msg/detail/referline__struct.hpp>
#include <cmath>
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"



namespace Planning
{
    using base_msgs::msg::LocalPath;
    using base_msgs::msg::Referline;
    using geometry_msgs::msg::PoseStamped;
    using nav_msgs::msg::Path;

    constexpr double delta_s_min = 1.0;
    constexpr double kMathEpsilon = 1.0e-6;

    class Curve // 曲线
    {
    public:
        Curve() = default;

        static double NormalizeAngle(const double &angle);   // 归一化角度,约束到[-pi, pi]

        // 笛卡尔转自然坐标系
        static void cartesian_to_frenet(const double &x, const double &y, const double &theta,
                                        const double &speed, const double &a, const double &kappa, // 输入1：目标点在笛卡尔下的参数 x y theta speed a
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2：目标点在参考线的投影点在笛卡尔下的参数 rs rx ry rtheta rkappa rdkappa
                                        double &s, double &ds_dt, double &dds_dt,
                                        double &l, double &dl_ds, double &dl_dt, double &ddl_ds, double &ddl_dt); // 输出：目标点在自然坐标系下的参数 s ds_dt dds_dt l dl_ds dl_dt ddl_ds ddl_dt

        // 自然坐标系转笛卡尔
        static void frenet_to_cartesian(const double &s, const double &ds_dt, const double &dds_dt,
                                        const double &l, const double &dl_ds, const double &ddl_ds, // 输入1：目标点在自然坐标系下的参数 s ds_dt dds_dt l dl_ds dl_dt ddl_ds ddl_dt                                       
                                        const double &rs, const double &rx, const double &ry,
                                        const double &rtheta, const double &rkappa, const double &rdkappa, // 输入2：目标点在参考线的投影点在笛卡尔下的参数 rs rx ry rtheta rkappa rdkappa
                                        double &x, double &y, double &theta,
                                        double &speed, double &a, double &kappa); // 输出：目标点在笛卡尔下的参数 x y theta speed a

        // 找匹配点的下标
        static int find_match_point(const Path &path, const int &last_match_point_index,const PoseStamped &target_point); // 利用上一帧
        static int find_match_point(const Referline &path, const PoseStamped &target_point); // 在参考线上查找匹配点下标
        static int find_match_point(const LocalPath &path, const PoseStamped &target_point); // 在局部路径上查找匹配点下标
        static int find_match_point(const Referline &path, const double &rs); // 通过rs查找匹配点下标


        // 找到投影点
        static void find_projection_point(const Referline &referline, const PoseStamped &target_point, // 输入：参考线和目标点
                                          double &rs, double &rx, double &ry,
                                          double &rtheta, double &rkappa, double &rdkappa); // 输出：目标点在参考线的投影点在笛卡尔下的参数 rs rx ry rtheta rkappa rdkappa

        static void find_projection_point(const LocalPath &local_path, const PoseStamped &target_point, // 输入：局部路径和目标点
                                          double &rs, double &rx, double &ry,
                                          double &rtheta, double &rkappa, double &rdkappa); // 输出：目标点在局部路径的投影点在笛卡尔下的参数 rs rx ry rtheta rkappa rdkappa

        // 计算投影点参数
        static void cal_projection_param(Referline &refer_line); // 参考线
        static void cal_projection_param(LocalPath &local_path); // 局部路径
        

    };
} // namespace Planning
#endif // CURVE_H_

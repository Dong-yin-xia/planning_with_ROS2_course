"""
文件名: 数据绘制
作者: C哥智驾说
完成时间: 2024.12

编译类型: 节点
依赖: ROS2内部库:
        rclpy
    外部库:
        base_msgs
        numpy
        matplotlib

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
"""

import rclpy
from rclpy.node import Node
from base_msgs.msg import PlotInfo
import numpy as np
import matplotlib.pyplot as plt


class PlotData(Node):
    def __init__(self):
        super().__init__("data_plot_node")
        self.get_logger().info("data_plot_node created")

        # 订阅绘图信息
        self.subscription = self.create_subscription(
            PlotInfo,
            "planning/plot_info",
            self.do_plot,
            10,
        )

    # 绘图
    def do_plot(self, plot_info):
        plt.clf() # 清空绘图区（当前 figure）

        if not plot_info.trajectory_info.local_trajectory:  # 确保数组不为空
            self.get_logger().warn("收到的轨迹为空")
            return

    # 从已有的数组创建数组
        s = np.asarray(
            [
                point.path_point.s 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        l = np.asarray(
            [
                point.path_point.l 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        dl_ds = np.asarray(
            [
                point.path_point.dl_ds 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        theta = np.asarray(
            [
                point.path_point.theta 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        kappa = np.asarray(
            [
                point.path_point.kappa 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        # 下面这些量属于速度点，应从 speed_point 读取
        t = np.asarray(
            [
                point.speed_point.t 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        s_2path = np.asarray(
            [
                point.speed_point.s_2path 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        speed = np.asarray(
            [
                point.speed_point.speed 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        acceleration = np.asarray(
            [
                point.speed_point.acceleration 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        # 创建子图
        fig1 = plt.subplot(4,1,1) # 4行1列的子图，第1个
        fig2 = plt.subplot(4,1,2) # 4行1列的子图，第2个
        fig3 = plt.subplot(4,1,3) # 4行1列的子图，第3个
        fig4 = plt.subplot(4,1,4) # 4行1列的子图，第4个

        # 选择fig1
        plt.sca(fig1)
        plt.plot(s, l, color = "green", label = "l", linestyle = "solid") # 绘制l关于s的图

        # 绘制障碍物
        for obs in plot_info.obs_info:
            # 计算障碍物上下左右边界
            s_left = obs.s - obs.obs_length / 2.0
            s_right = obs.s + obs.obs_length / 2.0
            l_bottom = obs.s - obs.obs_width / 2.0
            l_up = obs.s + obs.obs_width / 2.0

            # 绘制障碍物多边形
            p_sl = plt.Polygon(
                xy = [
                    [s_left, l_bottom],
                    [s_right, l_bottom],
                    [s_right, l_up],
                    [s_left, l_up],
                ],
                color = "blue",
                alpha = 0.8,
            )
            fig1.add_patch(p_sl) # 添加这个多边形到绘图区

        plt.title("ls info") # 图的标题
        plt.xlabel("s") # 横坐标
        plt.legend() # 图例

        # 选择fig2
        plt.sca(fig2)
        plt.plot(s, dl_ds, color = "red", label = "dl_ds", linestyle = "solid")     # 斜率关于s
        plt.plot(s, theta, color = "orange", label = "theta", linestyle = "solid")  # 航向角关于s
        plt.plot(s, kappa, color = "cyan", label = "kappa", linestyle = "solid")    # 曲率关于s
        plt.title("ls params") # 图的标题
        plt.xlabel("s") # 横坐标
        plt.legend() # 图例

        # 选择fig3
        plt.sca(fig3)
        plt.plot(t, s_2path, color = "green", label = "s_2path", linestyle = "solid")     # s_2path关于t

        # 绘制障碍物
        for obs in plot_info.obs_info:
            # 计算障碍物4个点坐标
            delta_s = obs.ds_dt_2path * (obs.t_out - obs.t_in)
            s_left_bottom = obs.s_2path - obs.obs_length / 2.0
            s_left_up = obs.s_2path + obs.obs_length / 2.0
            s_right_bottom = s_left_bottom + delta_s
            s_right_up = s_left_up + delta_s

            # 绘制障碍物多边形
            p_st = plt.Polygon(
                xy = [
                    [obs.t_in, s_left_bottom],
                    [obs.t_out, s_right_bottom],
                    [obs.t_out, s_right_up],
                    [obs.t_in, s_left_up],
                ],
                color = "blue",
                alpha = 0.8,
            )
            fig3.add_patch(p_st) # 添加这个多边形到绘图区

        plt.title("st info") # 图的标题
        plt.xlabel("t") # 横坐标
        plt.legend() # 图例

        # 选择fig4
        plt.sca(fig4)
        plt.plot(t, speed, color = "red", label = "speed", linestyle = "solid")     # 速度关于t
        plt.plot(t, acceleration, color = "cyan", label = "acceleration", linestyle = "solid")  # 加速度关于t
        plt.title("st params") # 图的标题
        plt.xlabel("t") # 横坐标
        plt.legend() # 图例


        plt.pause(0.05) # 暂停一小段时间，用于显示图像，不会阻塞


def main(args=None):
    rclpy.init(args=args)
    plot_node = PlotData()
    # plt.show(block=False)  # 显式显示窗口（非阻塞）

    try:
        rclpy.spin(plot_node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        rclpy.shutdown()  # 防止按ctrl+c时无法关闭


if __name__ == "__main__":
    main()

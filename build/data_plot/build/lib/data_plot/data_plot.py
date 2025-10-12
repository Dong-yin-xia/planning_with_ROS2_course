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
import numpy as np
import matplotlib.pyplot as plt


class PlotData(Node):
    def __init__(self):
        super().__init__("data_plot_node")
        self.get_logger().info("data_plot_node created")


def main(args=None):
    rclpy.init(args=args)
    plot_node = PlotData()

    try:
        rclpy.spin(plot_node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        rclpy.shutdown()  # 防止按ctrl+c时无法关闭


if __name__ == "__main__":
    main()

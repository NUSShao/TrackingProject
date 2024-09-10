#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from math import cos, sin
import matplotlib.pyplot as plt
import numpy as np

class LaserScanMapper(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        # 创建订阅器
        self.subscriber = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.lidar_callback,
            10
        )
        # 初始化图像显示窗口
        self.grid_size = 100
        self.resolution = 0.05  # 固定分辨率，每个栅格代表 5cm
        self.fig, self.ax = plt.subplots()
        self.grid_map = np.zeros((self.grid_size, self.grid_size))

        # 设置绘图参数，使用手动设置的vmin和vmax
        self.im = self.ax.imshow(self.grid_map, cmap='hot', origin='lower', vmin=0, vmax=1)
        plt.ion()  # 开启交互模式
        plt.show()

    def lidar_callback(self, scan_msg):
        # 获取Laserscan中有效信息
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        angle_increment = scan_msg.angle_increment
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max
        ranges = scan_msg.ranges

        # 初始化栅格地图
        self.grid_map = np.zeros((self.grid_size, self.grid_size))

        # 原点坐标（通常是小车中心）
        origin_x = self.grid_size // 2
        origin_y = self.grid_size // 2

        # 遍历ranges数组，获取有效范围内的障碍点
        for i, distance in enumerate(ranges):
            if distance > range_max or distance < range_min:
                continue

            # 计算角度
            angle = angle_min + i * angle_increment

            # 计算坐标
            x = distance * cos(angle) 
            y = distance * sin(angle)
            grid_x = int(x / self.resolution) + origin_x
            grid_y = -int(y / self.resolution) + origin_y # 左右镜像回来

            # 确保不超出地图范围
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                self.grid_map[grid_x][grid_y] = 1  # 表示有障碍物

        # 更新图像
        self.im.set_data(self.grid_map)
        plt.draw()
        plt.pause(0.01)  # 刷新图像


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

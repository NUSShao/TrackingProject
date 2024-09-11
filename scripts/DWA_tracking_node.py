#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import math
import cv2


# DWA参数设置
class DWAParams:
    def __init__(self):
        self.max_speed = 12.01  # 最大线速度
        self.max_yaw_rate = 1.2  # 最大角速度
        self.speed_resolution = 3.0  # 速度分辨率
        self.yaw_rate_resolution = 0.2  # 角速度分辨率
        self.dt = 0.5  # 时间间隔
        self.predict_time = 4.0  # 轨迹推演的时间
        self.safe_radius = 9.0  # 小车半径，防止碰撞
        self.angle_gain = 3.0  # 目标导向权重
        self.obstacle_gain = 4.0  # 障碍物避让权重
        self.speed_gain = 0.0  # 速度权重
        self.distance_gain = 0.5 # 目标距离权重

class DWAController(Node):
    def __init__(self):
        super().__init__('dwa_controller')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/robot/robot_diff_controller/cmd_vel_unstamped', 10)
        self.grid_subscriber = self.create_subscription(Image, '/robot/grid_map', self.grid_callback, 10)
        self.target_subscriber = self.create_subscription(Vector3, '/robot/relative_position', self.target_callback, 10)
        self.bridge = CvBridge()
        self.obstacle_map = None
        self.target_distance = None
        self.target_angle = None
        self.current_speed = 0.0
        self.current_yaw_rate = 0.0 # 需要更新
        self.get_logger().info('DWA tracking node started!')

    def grid_callback(self, msg):
        # 将障碍物的mono8编码的Image信息转换为栅格地图
        self.obstacle_map = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.obstacle_map = cv2.resize(self.obstacle_map, (100, 100))  # 确保尺寸为100x100

    def target_callback(self, msg):
        # 获取目标相对距离和方位角
        self.target_distance = msg.x  # 第0个位置是相对距离
        self.target_angle = -( msg.y / 180 ) * math.pi  # 第1个位置是相对角度，转换为弧度制
        print(self.target_angle)

def generate_trajectories(current_speed, current_yaw_rate, params):
    # 在速度和角速度的限制下，生成多个可能的轨迹，并保存对应的速度和角速度
    trajectories = []
    for v in np.arange(0, params.max_speed, params.speed_resolution):  # 从0到最大线速度
        for w in np.arange(-params.max_yaw_rate, params.max_yaw_rate, params.yaw_rate_resolution):  # 角速度从负最大值到正最大值
            trajectory = predict_trajectory(v, w, current_speed, current_yaw_rate, params)
            trajectories.append((trajectory, v, w))  # 保存轨迹及其对应的速度和角速度
    return trajectories


def predict_trajectory(v, w, current_speed, current_yaw_rate, params):
    # 轨迹的起点设为障碍物图的中心，障碍图的中央代表小车在(0,0)位置
    x, y, theta = 50, 50, 0  # 轨迹起点为(0,0)，即地图中央
    trajectory = [(x, y)]
    # 迭代次数等于预测时长除以步长
    for _ in range(int(params.predict_time / params.dt)):
        x += v * math.cos(theta) * params.dt
        y += v * math.sin(theta) * params.dt
        theta += w * params.dt
        trajectory.append((x, y))
    return trajectory

def angle_cost(trajectory, w, target_distance, target_angle, params):
    # 计算轨迹与目标之间的夹角
    target_x = 50 + (target_distance / 0.05) * math.cos(target_angle)
    target_y = 50 + (target_distance / 0.05) * math.sin(target_angle)
    final_x, final_y = trajectory[-1]
    final_y = 100 - final_y
    accumulate_offset = w * params.predict_time
    final_offset = math.atan(( target_y - final_y ) / (target_x - final_x))
    angle_offset = accumulate_offset - final_offset
    return abs(angle_offset)

def obstacle_cost(trajectory, obstacle_map, params):
    # 计算轨迹中最接近障碍物的点
    min_distance = 99999.0
    for point in trajectory:
        distance = get_min_distance_to_obstacle(point, obstacle_map)
        if distance < min_distance:
            min_distance = distance + 0.000001
    # 带有缓冲区的反比惩罚
    if min_distance >= params.safe_radius:
        return 0.000001
    else:
        # return math.sqrt(params.safe_radius / min_distance)
        return params.safe_radius / min_distance
    
def speed_cost(target_distance, v):
    return v
    
def distance_cost(trajectory, target_distance, target_angle):
    target_x = 50 + (target_distance / 0.05) * math.cos(target_angle)
    target_y = 50 + (target_distance / 0.05) * math.sin(target_angle)
    final_x, final_y = trajectory[-1]
    return math.sqrt((target_x - final_x)**2 + (target_y - final_y)**2)


def get_min_distance_to_obstacle(point, obstacle_map):
    # 将轨迹点映射到障碍物栅格地图，1个栅格等于0.05米
    x, y = point
    y_mirrored = 100 - y
    grid_x = int(x)
    grid_y = int(y_mirrored)

    if not (0 <= grid_x < obstacle_map.shape[1] and 0 <= grid_y < obstacle_map.shape[0]):
        return 99999.0  # 超出地图范围

    # 辐射搜索最近的障碍物
    max_radius = 10
    for radius in range(1, max_radius + 1):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                new_x = grid_x + dx
                new_y = grid_y + dy
                if 0 <= new_x < obstacle_map.shape[1] and 0 <= new_y < obstacle_map.shape[0]:
                    if obstacle_map[new_y, new_x] == 1:  # 找到障碍物
                        distance = math.sqrt((dx) ** 2 + (dy) ** 2)  # 计算实际距离，1个格子=0.05米
                        return distance

    return 99999.0  # 如果没有找到障碍物，返回无限大

# 评估每个轨迹
def evaluate_trajectory(trajectory, v, w, params, target_distance, target_angle, obstacle_map):
    angle_score = -angle_cost(trajectory, w, target_distance, target_angle, params)
    obstacle_score = -obstacle_cost(trajectory, obstacle_map, params)
    speed_score = speed_cost(v, target_distance)
    distance_score = -distance_cost(trajectory, target_distance, target_angle)
    return params.angle_gain * angle_score + params.obstacle_gain * obstacle_score + params.speed_gain * speed_score + params.distance_gain * distance_score

def visualize_paths(map_image, best_trajectory, all_trajectories):
    map_with_paths = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

    # # 绘制所有轨迹（白色线）
    # for trajectory, _, _ in all_trajectories:
    #     for i in range(len(trajectory) - 1):
    #         point1 = trajectory[i]
    #         point2 = trajectory[i + 1]
    #         # 解包point1和point2，确保它们是(x, y)坐标
    #         x1, y1 = point1[:2]
    #         x2, y2 = point2[:2]
    #         if (0 <= int(x1) < map_image.shape[1] and 0 <= int(y1) < map_image.shape[0] and
    #             0 <= int(x2) < map_image.shape[1] and 0 <= int(y2) < map_image.shape[0]):
    #             cv2.line(map_with_paths, (int(x1), int(y1)), (int(x2), int(y2)), (255, 255, 255), 1)

    # 绘制最优轨迹（绿色线）
    best_trajectory_points, _, _ = best_trajectory
    for i in range(len(best_trajectory_points) - 1):
        point1 = best_trajectory_points[i]
        point2 = best_trajectory_points[i + 1]
        # 解包point1和point2，确保它们是(x, y)坐标
        x1, y1 = point1[:2]
        x2, y2 = point2[:2]
        if (0 <= int(x1) < map_image.shape[1] and 0 <= int(y1) < map_image.shape[0] and
            0 <= int(x2) < map_image.shape[1] and 0 <= int(y2) < map_image.shape[0]):
            cv2.line(map_with_paths, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1)

    # 绘制障碍物（红色）
    map_with_paths[map_image == 1] = [255, 0, 0]

    # 清除之前的图像，重新绘制
    plt.clf()
    plt.imshow(map_with_paths)
    plt.title("DWA Path Visualization")

    # 使用 plt.pause() 来刷新图像而不阻塞
    plt.pause(0.001)


def dwa_control():
    rclpy.init()
    node = DWAController()
    params = DWAParams()

    plt.ion()  # 启用交互模式，允许图像窗口自动更新

    while rclpy.ok():
        rclpy.spin_once(node)

        if node.obstacle_map is not None and node.target_distance is not None:
            trajectories = generate_trajectories(node.current_speed, node.current_yaw_rate, params)
            best_trajectory = max(trajectories, key=lambda traj_info: evaluate_trajectory(traj_info[0], traj_info[1], traj_info[2], params, node.target_distance, node.target_angle, node.obstacle_map))
            # 可视化雷达图以及路径
            # visualize_paths(node.obstacle_map, best_trajectory, trajectories)
            # 发布最优轨迹对应的速度
            best_v, best_w = best_trajectory[1], best_trajectory[2]
            twist = Twist()

            twist.angular.z = best_w
            
            if node.target_distance <= 1:
                twist.linear.x = 0.0
            elif node.target_distance >= 3:
                twist.linear.x = best_v * 0.05
                # print(f"speed:{twist.linear.x}")
            else:
                twist.linear.x = best_v * 0.05 * ( 1 - math.pow( math.e, -2.5 * (node.target_distance-1) )) 
                # print(f"speed:{twist.linear.x}")
            # print(f"angle speed:{twist.angular.z}")
            node.cmd_vel_publisher.publish(twist)
            node.current_yaw_rate = -best_w
            node.current_speed = best_v

    rclpy.shutdown()

if __name__ == '__main__':
    dwa_control()


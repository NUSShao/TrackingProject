import numpy as np
import math
import cv2
import matplotlib.pyplot as plt

# DWA参数设置
class DWAParams:
    def __init__(self):
        self.max_speed = 2.0  # 最大线速度
        self.max_yaw_rate = 1.0  # 最大角速度
        self.speed_resolution = 0.5  # 速度分辨率
        self.yaw_rate_resolution = 0.05  # 角速度分辨率
        self.dt = 1.0  # 时间间隔
        self.predict_time = 10.0  # 轨迹推演的时间
        self.safe_radius = 3.0  # 小车半径，防止碰撞
        self.goal_gain = 1.0  # 目标导向权重
        self.obstacle_gain = 1.0  # 障碍物避让权重
        self.speed_gain = 1.0  # 速度权重

# 生成多个可能轨迹
def generate_trajectories(params):
    trajectories = []
    for v in np.arange(0, params.max_speed, params.speed_resolution):  # 速度范围
        for w in np.arange(-params.max_yaw_rate, params.max_yaw_rate, params.yaw_rate_resolution):  # 角速度范围
            trajectory = predict_trajectory(v, w, params)
            trajectories.append((trajectory, v, w))
    return trajectories

# 预测轨迹
def predict_trajectory(v, w, params):
    x, y, theta = 50, 50, 0  # 初始位置在地图中心
    trajectory = [(x, y)]
    for _ in range(int(params.predict_time / params.dt)):
        x += v * math.cos(theta) * params.dt
        y += v * math.sin(theta) * params.dt
        theta += w * params.dt
        trajectory.append((x, y))
    return trajectory

# 计算与目标角度的偏差
def angle_cost(w, target_angle, params):
    initial_offset = target_angle
    accumulate_offset = w * params.predict_time
    angle_offset = initial_offset + accumulate_offset
    return abs(angle_offset)

# 计算轨迹中最接近障碍物的距离
def obstacle_cost(trajectory, obstacle_map, params):
    min_distance = float('inf')
    for point in trajectory:
        distance = get_min_distance_to_obstacle(point, obstacle_map)
        if distance < min_distance:
            min_distance = distance
    if min_distance >= params.safe_radius:
        return 0
    else:
        return math.sqrt(params.safe_radius / min_distance)

# 计算点到障碍物的最小距离
def get_min_distance_to_obstacle(point, obstacle_map):
    x, y = point
    grid_x = int(x)
    grid_y = int(y)

    if not (0 <= grid_x < obstacle_map.shape[1] and 0 <= grid_y < obstacle_map.shape[0]):
        return np.inf

    max_radius = 15
    for radius in range(1, max_radius + 1):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                new_x = grid_x + dx
                new_y = grid_y + dy
                if 0 <= new_x < obstacle_map.shape[1] and 0 <= new_y < obstacle_map.shape[0]:
                    if obstacle_map[new_y, new_x] == 1:
                        return math.sqrt((dx) ** 2 + (dy) ** 2)

    return np.inf

# 评估每个轨迹
def evaluate_trajectory(trajectory, v, w, params, target_angle, obstacle_map):
    angle_score = -angle_cost(w, target_angle, params)
    obstacle_score = -obstacle_cost(trajectory, obstacle_map, params)
    speed_score = v
    return params.goal_gain * angle_score + params.obstacle_gain * obstacle_score + params.speed_gain * speed_score

# 加载地图并可视化轨迹
def visualize_paths(map_image, best_trajectory, all_trajectories):
    map_with_paths = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)

    # 绘制所有轨迹（白色线）
    for trajectory, _, _ in all_trajectories:
        for i in range(len(trajectory) - 1):
            point1 = trajectory[i]
            point2 = trajectory[i + 1]
            if (0 <= int(point1[0]) < map_image.shape[1] and 0 <= int(point1[1]) < map_image.shape[0] and
                0 <= int(point2[0]) < map_image.shape[1] and 0 <= int(point2[1]) < map_image.shape[0]):
                cv2.line(map_with_paths, (int(point1[0]), int(point1[1])), 
                         (int(point2[0]), int(point2[1])), (255, 255, 255), 1)

    # 绘制最优轨迹（绿色线）
    for i in range(len(best_trajectory) - 1):
        point1 = best_trajectory[i]
        point2 = best_trajectory[i + 1]
        if (0 <= int(point1[0]) < map_image.shape[1] and 0 <= int(point1[1]) < map_image.shape[0] and
            0 <= int(point2[0]) < map_image.shape[1] and 0 <= int(point2[1]) < map_image.shape[0]):
            cv2.line(map_with_paths, (int(point1[0]), int(point1[1])), 
                     (int(point2[0]), int(point2[1])), (0, 255, 0), 1)

    # 绘制障碍物（红色）
    map_with_paths[map_image == 1] = [255, 0, 0]

    # 显示结果
    plt.imshow(map_with_paths)
    plt.title("DWA Path Visualization")
    plt.show()

# 主函数
def main():
    # 加载地图 (100x100 PNG 图片，0表示自由区域，1表示障碍物)
    obstacle_map = cv2.imread('/home/nusshao/dev_ws/map_001.png', cv2.IMREAD_GRAYSCALE)
    # _, obstacle_map = cv2.threshold(obstacle_map, 128, 1, cv2.THRESH_BINARY)

    # 定义DWA参数
    params = DWAParams()
    target_angle = 0  # 目标角度，例如朝向正前方

    # 生成所有轨迹
    trajectories = generate_trajectories(params)

    # 评估每条轨迹并选择最优轨迹
    best_trajectory = None
    best_score = float('-inf')
    for trajectory, v, w in trajectories:
        score = evaluate_trajectory(trajectory, v, w, params, target_angle, obstacle_map)
        if score > best_score:
            best_score = score
            best_trajectory = trajectory

    # 可视化所有轨迹和最优轨迹
    visualize_paths(obstacle_map, best_trajectory, trajectories)

if __name__ == '__main__':
    main()

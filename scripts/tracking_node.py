#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class TrackingNode(Node):
    def __init__(self):
        super().__init__("tracking_node")
    
        # 创建cvbridge实例
        self.bridge = CvBridge()

        # 订阅相机的话题
        self.subscription = self.create_subscription(
            Image,
            '/target/camera/image_raw',
            self.listener_callback,
            10
            )
        
        self.subscriptions # 防止被垃圾回收

        # 发布控制命令到cmd_vel话题
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/robot/robot_diff_controller/cmd_vel_unstamped', 10
        )

        # 发布相对位置到 /robot/relative_position
        self.relative_position_publisher = self.create_publisher(
            Vector3, '/robot/relative_position', 10
        )

        # 加载YOLOV8模型
        self.model = YOLO('/home/nusshao/dev_ws/src/tracking_project/yolo_training/best.pt') # 改称自己的模型参数文件路径

        # 相机参数
        self.horizontal_fov = 60.0  # 水平视野（度）
        self.image_width = 640
        self.image_height = 480

        self.get_logger().info('Tracking Node has been started')

    def listener_callback(self, msg):
        # ROS图像转OpenCV图像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting ROS2 Image msg to cv2 image: {e}")
            return

        # 使用YOLO模型进行检测，并获取检测框的信息
        results = self.model(cv_image)
        if len(results[0].boxes) > 0:
            # 获取第一个检测到的目标
            box = results[0].boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            target_center_x = (x1 + x2) / 2
            target_center_y = (y1 + y2) / 2

            # 计算目标在图像中的水平偏移量
            offset_x = target_center_x - (self.image_width / 2)
            offset_x_angle = (offset_x / self.image_width) * self.horizontal_fov

            # 估算目标距离
            target_width_in_image = x2 - x1
            distance_to_target = self.estimate_distance(target_width_in_image)

            # 发布控制命令
            self.track_target(offset_x_angle, distance_to_target)

            # 可视化检测结果
            annotated_frame = results[0].plot()
            annotated_frame = self.display_offset_on_image(annotated_frame, offset_x_angle, distance_to_target)
            cv2.imshow("YOLO Detection", annotated_frame)
            cv2.waitKey(1)

            # 发布相对位置到 /robot/relative_position
            relative_position_msg = Vector3()
            relative_position_msg.x = distance_to_target
            relative_position_msg.y = offset_x_angle
            relative_position_msg.z = 0.0  # Z轴不用
            self.relative_position_publisher.publish(relative_position_msg)

        else:
            self.get_logger().info("No target detected. Stopping robot.")
            self.stop_robot()
            cv2.imshow("YOLO Detection", cv_image)
            cv2.waitKey(1)
            # 发布相对位置到 /robot/relative_position
            relative_position_msg = Vector3()
            relative_position_msg.x = 99999 # 检测不到，将距离和角度偏差都设为99999
            relative_position_msg.y = 99999
            relative_position_msg.z = 0.0  # Z轴不用
            self.relative_position_publisher.publish(relative_position_msg)


    def estimate_distance(self, target_width_in_image):
        # 估算目标距离的函数（根据目标的实际大小和图像中的大小进行估算）
        # 这里使用简单的比例方法来估计，实际应用中可以用更加复杂的距离估计方法
        known_object_width = 0.44  # 假设目标的实际宽度为0.5米
        focal_length = 528.43  # 焦距（像素为单位）f= W / 2 * tan(fov / 2)
        distance_to_target = (known_object_width * focal_length) / target_width_in_image
        return distance_to_target
    
    def track_target(self, offset_x_angle, distance_to_target):
        # 创建Twist消息
        twist_msg = Twist()

        # 设置线速度
        if distance_to_target > 1.5:
            twist_msg.linear.x = 0.45
        elif distance_to_target < 1.0:
            twist_msg.linear.x = -0.2
        else:
            twist_msg.linear.x = 0.0

        # 设置角速度
        if abs(offset_x_angle) > 5:
            twist_msg.angular.z = -0.05 * offset_x_angle
        else:
            twist_msg.angular.z = -0.01 * offset_x_angle

        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        # 没有目标时，停止移动
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
    
    def display_offset_on_image(self, image, offset_x_angle, distance_to_target):
        """在图像上显示角度偏移和距离信息"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = f"Angle Offset: {offset_x_angle:.2f} deg | Distance: {distance_to_target:.2f} m"
        
        # 在图像上绘制文本
        cv2.putText(image, text, (10, 30), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        return image

def main(args=None):
    rclpy.init(args=args)
    tracking_node = TrackingNode()

    try:
        rclpy.spin(tracking_node)
    except KeyboardInterrupt:
        pass
    finally:
        tracking_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
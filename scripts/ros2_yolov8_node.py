#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class Yolov8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')

        # 创建一个cvbridge实例，用来将Image msg转换为CV2格式
        self.bridge = CvBridge()

        # 订阅camera话题
        self.subscription = self.create_subscription(
            Image,
            '/robot/camera/image_raw',
            self.listener_callback,
            10  # 订阅者信息队列深度
        )

        self.subscriptions # 防止被垃圾回收

        # 加载YoloV8模型
        self.model = YOLO('/home/nusshao/dev_ws/src/tracking_project/yolo_training/best.pt') # 改称自己的模型参数文件路径
        self.get_logger().info('YOLOv8 Node has been started')

    def listener_callback(self, msg):
        # Image 转 CV2
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting ROS Image message to OpenCV: {e}")
            return
        
        # 调用 YOLOV8
        results = self.model(cv_image)

        # 绘制检测框
        annotated_frame = results[0].plot()

        # 结果可视化窗口
        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.waitKey(1) # 实时更新窗口


def main(args=None):
    rclpy.init(args=args)
    yolov8_node = Yolov8Node()

    try:
        rclpy.spin(yolov8_node)
    except KeyboardInterrupt:
        pass
    finally:
        yolov8_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class GridMapSubscriber(Node):
    def __init__(self):
        super().__init__('grid_map_subscriber')
        self.subscriber = self.create_subscription(
            Image, 
            "/robot/grid_map", 
            self.save_map_callback, 
            10
        )
        self.bridge = CvBridge()  # CvBridge用于ROS和OpenCV格式的转换

    def save_map_callback(self, msg):
        # 将ROS的Image消息转换为OpenCV格式
        obstacle_map = self.bridge.imgmsg_to_cv2(msg, "mono8")

        # 保存为图像文件
        filename = 'saved_grid_map.png'
        cv2.imwrite(filename, obstacle_map)
        self.get_logger().info(f'Grid map saved as {filename}!')

def main(args=None):
    rclpy.init(args=args)
    node = GridMapSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import os
import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, jsonify
from flask_cors import CORS  # 引入 CORS

# 创建 Flask 应用
app = Flask(__name__)
CORS(app)  # 启用 CORS

# 图像保存路径
SAVE_PATH = '/home/nusshao/Desktop/raw_data'  # 这里替换为你希望保存图片的路径

# ~/dev_ws/src/tracking_project
# python3 webserver.py

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/robot/camera/image_raw',  # 订阅的图像话题
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.current_image = None

    def listener_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def get_next_image_filename(self):
        # 确保保存路径存在
        if not os.path.exists(SAVE_PATH):
            os.makedirs(SAVE_PATH)
        
        # 获取文件夹中的所有文件
        files = os.listdir(SAVE_PATH)

        # 使用正则表达式匹配文件名中的数字部分，假设格式为XXX.png（如001.png, 002.png）
        number_pattern = re.compile(r'(\d{3})\.png')

        max_number = 0
        for file in files:
            match = number_pattern.match(file)
            if match:
                number = int(match.group(1))  # 获取文件名中的数字部分
                if number > max_number:
                    max_number = number

        # 下一个文件名编号 +1，并格式化为三位数字
        next_number = max_number + 1
        next_filename = f"{next_number:03}.png"  # 将数字格式化为三位，如001, 002

        return os.path.join(SAVE_PATH, next_filename)

    def save_image(self):
        if self.current_image is not None:
            # 获取下一个文件名
            img_filepath = self.get_next_image_filename()

            # 保存图像
            cv2.imwrite(img_filepath, self.current_image)
            self.get_logger().info(f"Image saved as {img_filepath}")
            return img_filepath
        else:
            return None

# 全局变量，持有 ImageSaver 实例
image_saver_node = None

# 创建 Flask 路由，处理保存图像的请求
@app.route('/save_image', methods=['GET'])
def save_image():
    global image_saver_node
    if image_saver_node:
        # 调用 ImageSaver 类的 save_image 方法
        filepath = image_saver_node.save_image()
        if filepath:
            return jsonify({"message": f"Image saved at {filepath}"}), 200
        else:
            return jsonify({"message": "No image available yet."}), 404
    else:
        return jsonify({"message": "Image saver node not initialized."}), 500

# 启动 ROS2 节点
def main(args=None):
    global image_saver_node
    # 初始化 ROS2
    rclpy.init(args=args)
    # 创建 ImageSaver 节点实例
    image_saver_node = ImageSaver()
    
    # 启动 ROS2 节点（在新的线程中运行）
    rclpy.spin(image_saver_node)

if __name__ == '__main__':
    from threading import Thread
    # 启动 ROS2 节点的线程
    ros_thread = Thread(target=main)
    ros_thread.start()

    # 启动 Flask Web 服务器，监听 5000 端口
    app.run(host='0.0.0.0', port=5000)

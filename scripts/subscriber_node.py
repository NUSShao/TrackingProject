#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber') #Calling the Node class init func to specify the name of the topic
        self.subscriber = self.create_subscription(
            String,
            'py_pub_topic',
            self.listener_callback,
            10
        )
        self.subscriptions

    def listener_callback(self, msg):
        self.get_logger().info('I heard "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
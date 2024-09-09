#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'py_pub_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS2!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Q1、__init__(self)是起到什么作用？为什么要把self参数传进去？
# A：__init__ 是 Python 中类的构造函数。当你创建一个类的实例时，__init__ 方法会被自动调用，用于初始化该实例的属性。
# self 参数是每个类方法的第一个参数，它指向类的实例本身。通过 self，你可以访问和修改这个实例的属性和方法。
# self 允许你在类内部访问实例变量。没有 self，类中的方法和属性无法与具体的实例关联。
# 
# Q2、super().__init__('simple_publisher')起到什么作用？参数是什么意思？
# A： super() 调用父类的构造函数。在 rclpy 中，Node 类是一个父类，当你创建自己的节点时，你需要初始化这个父类，以便使用其功能。
# 'simple_publisher' 是节点的名称，每个 ROS 2 节点都必须有一个唯一的名字。
# 
# Q3、self.publisher_ = self.create_publisher(String, 'py_pub_topic', 10)当中，
# QoS设置为10有什么用？如果我设置为别的值呢？
# A：QoS（Quality of Service，服务质量）控制了消息传递的可靠性和性能。这里的 10 代表了消息队列的深度，
# 也就是最多可以存储 10 条未处理的消息。
# 如果发布者的消息比订阅者处理的速度快，超过队列的深度后，新的消息会丢弃。
# 如果你设置其他值，例如更大的值，会允许更多的未处理消息排队；
# 而设置更小的值，会限制消息数量，可能导致消息更容易丢失，尤其在系统负载高的时候。
# 
# Q4、main函数为什么要传一个args=None的参数？他对于rclpy的init有什么用？
# A：args=None 是为了允许传入命令行参数。rclpy.init(args=args) 会使用命令行参数来初始化 ROS 2 的系统。
# 如果你不需要传递命令行参数，你可以省略这个参数，但保留它可以让代码更通用。
# 如果你的程序需要从命令行传递参数，args 会捕获这些参数并传递给 rclpy。 
#  
# Q5、spin是rclpy的什么？它起到什么作用？
# A：rclpy.spin(node) 是一个阻塞函数，它会让程序持续运行，等待和处理来自 ROS 2 网络的消息或事件。
# 通过 spin，你的节点才能保持活动并持续监听订阅的话题或其他 ROS 事件。
# 如果不调用 spin，节点可能会立即退出。 
# 
# Q6、except KeyboardInterrupt有什么用？如果我不设置这一行会怎样？
# A：KeyboardInterrupt 捕获的是用户在终端使用 Ctrl+C 中断程序时的异常。如果不捕获这个异常，
# 按下 Ctrl+C 时程序会直接退出，可能不会执行后续的资源清理工作（例如关闭节点和释放资源）。
# 
# Q7、finally语法是什么用处？
# A：finally 语法用于确保不管有没有抛出异常，都会执行指定的代码块。
# 在这个例子中，finally 确保节点会在程序结束时被销毁，释放资源。
# 
# Q8、if __name__ == '__main__':起到什么作用？
# A：这行代码用于确保脚本作为主程序运行时才会执行特定代码块。
# 如果这个脚本被另一个模块导入，这部分代码将不会执行。这样可以避免模块导入时意外运行主程序逻辑。
#
# Q9、self.get_logger()是干什么的？
# self.get_logger() 是 ROS 2 中节点用于记录日志信息的方法。
# 当你调用 self.get_logger() 时，它会返回一个 Logger 对象，
# 该对象能够以不同的日志级别打印消息到控制台或日志文件中。
# Debug: logger.debug()，调试信息，通常是最详细的日志信息。
# Info: logger.info()，用于打印普通的信息消息，常用于程序的正常运行情况。
# Warn: logger.warn()，用于打印警告消息，提醒可能会有问题但程序还能继续运行。
# Error: logger.error()，用于打印错误信息，通常意味着程序遇到了某种错误但还能继续运行。
# Fatal: logger.fatal()，用于打印严重错误，通常意味着程序无法继续运行。
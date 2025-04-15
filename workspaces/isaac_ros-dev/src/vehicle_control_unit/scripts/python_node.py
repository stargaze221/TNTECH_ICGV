#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PythonNode(Node):
    def __init__(self):
        super().__init__('python_node')
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from Python! 123'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    python_node = PythonNode()
    rclpy.spin(python_node)
    python_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

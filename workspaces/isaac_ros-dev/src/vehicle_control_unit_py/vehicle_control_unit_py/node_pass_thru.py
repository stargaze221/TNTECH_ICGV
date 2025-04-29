#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class PWMPassthrough(Node):
    def __init__(self):
        super().__init__('pwm_passthrough')

        # Subscriber to incoming PWM sensor values
        self.subscriber = self.create_subscription(
            Int32MultiArray,
            'pwm_sensor',
            self.sensor_callback,
            10)

        # Publisher to PWM command
        self.publisher = self.create_publisher(
            Int32MultiArray,
            'pwm_command',
            10)

        self.get_logger().info('PWM passthrough node started')

    def sensor_callback(self, msg):
        # Directly forward sensor data as command
        self.publisher.publish(msg)
        # self.get_logger().info(f'Forwarded PWM: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = PWMPassthrough()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

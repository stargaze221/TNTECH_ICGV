#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')

        # Parameters
        self.wheelbase = 0.3  # meters

        # Publishers
        self.throttle_pub = self.create_publisher(Float32, '/throttle_cmd', 10)
        self.steering_pub = self.create_publisher(Float32, '/steering_cmd', 10)

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info('CmdVel to Ackermann converter started.')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x  # Forward velocity (m/s)
        omega = msg.angular.z  # Yaw rate (rad/s)

        throttle_cmd = Float32()
        steering_cmd = Float32()

        throttle_cmd.data = v

        # Avoid division by zero when v is very small
        if abs(v) > 1e-4:
            steering_cmd.data = math.atan(self.wheelbase * omega / v)
        else:
            steering_cmd.data = 0.0

        self.throttle_pub.publish(throttle_cmd)
        self.steering_pub.publish(steering_cmd)

        self.get_logger().info(f'Throttle: {v:.2f} m/s | Steering Angle: {steering_cmd.data:.2f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

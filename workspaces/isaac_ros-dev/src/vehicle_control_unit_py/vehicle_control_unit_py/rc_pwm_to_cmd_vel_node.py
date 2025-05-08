#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class RCPWMToCmdVel(Node):
    def __init__(self):
        super().__init__('rc_pwm_to_cmd_vel_node')

        # Parameters
        self.declare_parameter('deadband_pwm', 30)
        self.declare_parameter('center_pwm', 1500)
        self.declare_parameter('max_throttle_pwm', 1900)
        self.declare_parameter('min_throttle_pwm', 1100)
        self.declare_parameter('max_steering_pwm', 1900)
        self.declare_parameter('min_steering_pwm', 1100)
        self.declare_parameter('max_speed', 0.7)  # m/s
        self.declare_parameter('max_steering_angle', 0.5)  # rad

        # Get parameters
        self.deadband_pwm = self.get_parameter('deadband_pwm').get_parameter_value().integer_value
        self.center_pwm = self.get_parameter('center_pwm').get_parameter_value().integer_value
        self.max_throttle_pwm = self.get_parameter('max_throttle_pwm').get_parameter_value().integer_value
        self.min_throttle_pwm = self.get_parameter('min_throttle_pwm').get_parameter_value().integer_value
        self.max_steering_pwm = self.get_parameter('max_steering_pwm').get_parameter_value().integer_value
        self.min_steering_pwm = self.get_parameter('min_steering_pwm').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value

        # Publisher and Subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rc_pwm_sub = self.create_subscription(Int32MultiArray, '/rc_pwm', self.rc_pwm_callback, 10)

        self.get_logger().info("rc_pwm_to_cmd_vel node started and listening to /rc_pwm")

    def rc_pwm_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn("Invalid /rc_pwm message: expected 2 elements [steering, throttle]")
            return

        steering_pwm = msg.data[0]
        throttle_pwm = msg.data[1]
        self.publish_cmd_vel(throttle_pwm, steering_pwm)

    def publish_cmd_vel(self, throttle_pwm, steering_pwm):
        cmd_vel_msg = Twist()

        # --- Throttle
        throttle_error = throttle_pwm - self.center_pwm
        if abs(throttle_error) <= self.deadband_pwm:
            throttle_error = 0
        else:
            if throttle_error > 0:
                throttle_error = (throttle_pwm - self.center_pwm) / (self.max_throttle_pwm - self.center_pwm)
            else:
                throttle_error = (throttle_pwm - self.center_pwm) / (self.center_pwm - self.min_throttle_pwm)
        cmd_vel_msg.linear.x = throttle_error * self.max_speed

        # --- Steering
        steering_error = steering_pwm - self.center_pwm
        if abs(steering_error) <= self.deadband_pwm:
            steering_error = 0
        else:
            if steering_error > 0:
                steering_error = (steering_pwm - self.center_pwm) / (self.max_steering_pwm - self.center_pwm)
            else:
                steering_error = (steering_pwm - self.center_pwm) / (self.center_pwm - self.min_steering_pwm)
        cmd_vel_msg.angular.z = steering_error * self.max_steering_angle

        self.cmd_vel_pub.publish(cmd_vel_msg)

        # self.get_logger().info(
        #     f"RC PWM → Throttle: {throttle_pwm}, Steering: {steering_pwm} "
        #     f"=> CmdVel: linear.x={cmd_vel_msg.linear.x:.2f}, angular.z={cmd_vel_msg.angular.z:.2f}"
        # )

def main(args=None):
    rclpy.init(args=args)
    node = RCPWMToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
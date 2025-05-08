#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import serial
import math

class CmdVelToPWMSerialNode(Node):
    def __init__(self):
        super().__init__('cmdvel_to_pwm_serial_node')

        # --- Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('mode', 'ackermann')
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_angular', 0.5)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('center_pwm', 1500)
        self.declare_parameter('max_pwm', 1700)
        self.declare_parameter('min_pwm', 1300)

        # --- Get Parameters ---
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.center_pwm = self.get_parameter('center_pwm').get_parameter_value().integer_value
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().integer_value
        self.min_pwm = self.get_parameter('min_pwm').get_parameter_value().integer_value

        # --- Serial Setup ---
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            exit(1)

        # --- Subscribers & Publishers ---
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.rc_pub = self.create_publisher(Int32MultiArray, '/rc_pwm', 10)

        # --- Timer for polling serial input ---
        self.timer = self.create_timer(0.05, self.sensor_timer_callback)  # 20 Hz

        self.get_logger().info(f'Running in {self.mode.upper()} mode.')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        if self.mode == 'diffdrive':
            throttle_pwm = self.compute_throttle_pwm(v)
            steering_pwm = self.compute_steering_pwm_diffdrive(omega)
        elif self.mode == 'ackermann':
            throttle_pwm = self.compute_throttle_pwm(v)
            steering_pwm = self.compute_steering_pwm_ackermann(v, omega)
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}")
            return

        command = f"{steering_pwm},{throttle_pwm}\n"
        self.ser.write(command.encode('utf-8'))

    def compute_throttle_pwm(self, v):
        v = max(min(v, self.max_speed), -self.max_speed)
        if v >= 0:
            pwm = int(self.center_pwm + (v / self.max_speed) * (self.max_pwm - self.center_pwm))
        else:
            pwm = int(self.center_pwm + (v / self.max_speed) * (self.center_pwm - self.min_pwm))
        return pwm

    def compute_steering_pwm_diffdrive(self, omega):
        omega = max(min(omega, self.max_angular), -self.max_angular)
        if omega >= 0:
            pwm = int(self.center_pwm + (omega / self.max_angular) * (self.max_pwm - self.center_pwm))
        else:
            pwm = int(self.center_pwm + (omega / self.max_angular) * (self.center_pwm - self.min_pwm))
        return pwm

    def compute_steering_pwm_ackermann(self, v, omega):
        if abs(v) < 1e-4:
            steering_angle = 0.0
        else:
            steering_angle = math.atan(self.wheelbase * omega / v)
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)
        if steering_angle >= 0:
            pwm = int(self.center_pwm + (steering_angle / self.max_steering_angle) * (self.max_pwm - self.center_pwm))
        else:
            pwm = int(self.center_pwm + (steering_angle / self.max_steering_angle) * (self.center_pwm - self.min_pwm))
        return pwm

    def sensor_timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith("SENSOR:"):
                parts = line.replace("SENSOR:", "").split(",")
                if len(parts) == 2:
                    pwm1 = int(parts[0])
                    pwm2 = int(parts[1])
                    msg = Int32MultiArray()
                    msg.data = [pwm1, pwm2]
                    self.rc_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Serial read failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPWMSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
import serial
import math
import time

class CmdVelToPWMSerialNode(Node):
    def __init__(self):
        super().__init__('cmdvel_to_pwm_serial_node')

        # --- Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('mode', 'ackermann')  # We are not using the kinematics.
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_angular', 0.5)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('center_pwm', 1500)
        self.declare_parameter('max_pwm', 1700)
        self.declare_parameter('min_pwm', 1300)

        # PID gains (tune these)
        self.Kp_v = 50.0 #500.0
        self.Ki_v = 0.0 #50.0
        self.Kd_v = 0.0 #100.0

        self.Kp_w = 100.0 #300.0
        self.Ki_w = 0.0 #30.0
        self.Kd_w = 0.0 #60.0

        self.deadzone_threshold = 0.05

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
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.rc_pub = self.create_publisher(Int32MultiArray, '/rc_pwm', 10)

        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.current_vx = 0.0
        self.current_wz = 0.0

        # PID state
        self.prev_error_vx = 0.0
        self.integral_vx = 0.0
        self.prev_error_wz = 0.0
        self.integral_wz = 0.0

        self.last_time = self.get_clock().now()

        # --- Timer for polling serial input ---
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(f'Running in {self.mode.upper()} mode.')

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone_threshold:
            return 0.0
        return value
    
    def cmd_vel_callback(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

    def odom_callback(self, msg):
        self.current_vx = msg.twist.twist.linear.x
        self.current_wz = msg.twist.twist.angular.z

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt == 0:
            return
        
        error_vx = self.cmd_vx - self.current_vx
        error_wz = self.cmd_wz - self.current_wz

        self.integral_vx += error_vx * dt
        derivative_vx = (error_vx - self.prev_error_vx) / dt

        self.integral_wz += error_wz * dt
        derivative_wz = (error_wz - self.prev_error_wz) / dt

        control_throttle = (
            self.Kp_v * error_vx +
            self.Ki_v * self.integral_vx +
            self.Kd_v * derivative_vx
        )
        control_steering = (
            self.Kp_w * error_wz +
            self.Ki_w * self.integral_wz +
            self.Kd_w * derivative_wz
        )

        self.prev_error_vx = error_vx
        self.prev_error_wz = error_wz

        # Kickstart logic: apply boost if robot is commanded to move but not moving
        if abs(self.current_vx) < 0.1 and abs(self.cmd_vx) > 0.1:
            control_throttle += 100  # adjust this boost value based on your motor

    



        control_throttle = self.apply_deadzone(control_throttle)
        control_steering = self.apply_deadzone(control_steering)

        throttle_pwm = int(self.center_pwm + max(min(control_throttle, self.max_pwm - self.center_pwm), self.min_pwm - self.center_pwm))
        steering_pwm = int(self.center_pwm + max(min(control_steering, self.max_pwm - self.center_pwm), self.min_pwm - self.center_pwm))

        self.get_logger().info(
            f"PWM command:\n"
            f"  Throttle PWM: {throttle_pwm}\n"
            f"  Steering PWM: {steering_pwm}\n"
            f"CmdVel:\n"
            f"  linear.x = {self.cmd_vx:.2f}, angular.z = {self.cmd_wz:.2f}\n"
            f"Current:\n"
            f"  linear.x = {self.current_vx:.2f}, angular.z = {self.current_wz:.2f}\n"
            f"Error:\n"
            f"  linear.x = {self.cmd_vx - self.current_vx:.2f}, angular.z = {self.cmd_wz - self.current_wz:.2f}"
            f"PID State:\n"
            f"  self.integral_vx = {self.integral_vx:.2f}"
            f"Contoller:\n"
            f"  control_throttle = {control_throttle:.2f}"
        )

        command = f"{steering_pwm},{throttle_pwm}\n"
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Failed to write to serial: {e}")

        #Read RC and publish
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
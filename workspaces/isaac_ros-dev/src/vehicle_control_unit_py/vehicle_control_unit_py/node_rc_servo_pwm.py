#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class DualPWMInterface(Node):
    def __init__(self):
        super().__init__('dual_pwm_interface')

        # Open serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

        # ROS publishers and subscribers
        self.publisher_ = self.create_publisher(Int32MultiArray, 'pwm_sensor', 10)

        # Subscribe to the PWM command topic to send PWM values to Arduino
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'pwm_command',
            self.command_callback,
            10
        )

        # Set up a timer to periodically check for sensor data from Arduino
        self.timer = self.create_timer(0.05, self.read_from_arduino)

    def command_callback(self, msg):
        """Callback to handle PWM command from ROS to Arduino"""
        if len(msg.data) >= 2:
            pwm1 = msg.data[0]  # First PWM value (1-2000 µs)
            pwm2 = msg.data[1]  # Second PWM value (1-2000 µs)

            # Send PWM values to Arduino
            command_str = f"{pwm1},{pwm2}\n"
            # print(command_str)
            self.ser.write(command_str.encode('utf-8'))

    def read_from_arduino(self):
        """Read and parse sensor data from Arduino"""
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith("SENSOR:"):
                try:
                    # Parse the PWM values received from Arduino
                    parts = line.replace("SENSOR:", "").split(",")
                    pwm1 = int(parts[0])
                    pwm2 = int(parts[1])
                    # print(parts)

                    # Create a message to publish the sensor values
                    msg = Int32MultiArray()
                    msg.data = [pwm1, pwm2]
                    self.publisher_.publish(msg)

                except ValueError:
                    self.get_logger().warn(f"Failed to parse: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = DualPWMInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
import rosgraph_msgs.msg
import threading
import time
import sys
from datetime import datetime, timedelta

class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')
        self.topic_data = {}
        self.lock = threading.Lock()
        self.timer = self.create_timer(5.0, self.print_topic_status)
        self.create_timer(2.0, self.check_new_topics)

    def check_new_topics(self):
        all_topics = self.get_topic_names_and_types()
        with self.lock:
            for topic, types in all_topics:
                if topic not in self.topic_data:
                    msg_type = types[0]
                    msg_class = get_message(msg_type)
                    sub = self.create_subscription(
                        msg_class, topic,
                        lambda msg, t=topic: self.record_callback(t),
                        qos_profile_sensor_data
                    )
                    self.topic_data[topic] = {'last_msg_time': None, 'count': 0, 'type': msg_type, 'start': time.time()}

    def record_callback(self, topic_name):
        now = time.time()
        with self.lock:
            topic_info = self.topic_data[topic_name]
            topic_info['last_msg_time'] = now
            topic_info['count'] += 1

    def print_topic_status(self):
        with self.lock:
            print(f"\n{'Topic':<40} {'Type':<35} {'Hz':<6} {'Active':<6}")
            print("-" * 90)
            now = time.time()
            for topic, info in sorted(self.topic_data.items()):
                dt = now - info['start']
                hz = round(info['count'] / dt, 1) if dt > 0 else 0.0
                active = '✅' if info['last_msg_time'] and now - info['last_msg_time'] < 5 else '❌'
                print(f"{topic:<40} {info['type']:<35} {hz:<6} {active:<6}")

def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

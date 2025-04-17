from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vehicle_control_unit_py',
            executable='pwm_passthrough',
            name='my_node',
            output='screen'
        ),
        Node(
            package='vehicle_control_unit_py',
            executable='dual_pwm_interface',
            name='my_other_node',
            output='screen'
        )
    ])
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        # Push a namespace for better organization (optional)
        PushRosNamespace('vehicle_control_unit'),

        # Launch C++ node
        Node(
            package='vehicle_control_unit',
            executable='cpp_node',
            name='cpp_node',
            output='screen',
            parameters=[]  # Add parameters here if needed
        ),

        # Launch Python node
        Node(
            package='vehicle_control_unit',
            executable='python_node.py',  # Executable name, assuming it's in the scripts folder
            name='python_node',
            output='screen',
            parameters=[]  # Add parameters here if needed
        ),
    ])

from setuptools import setup
from glob import glob
import os

package_name = 'vehicle_control_unit_py'

launch_files = [
    (os.path.join('share', package_name, os.path.dirname(f)), [f])
    for f in glob(os.path.join('launch', '**', '*.py'), recursive=True)
]

config_files = [
    (os.path.join('share', package_name, os.path.dirname(f)), [f])
    for f in glob(os.path.join('config', '**', '*.yaml'), recursive=True)
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + launch_files + config_files,
    install_requires=['setuptools'],
    zip_safe=False,  # <--- change this
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Python-based ROS 2 node example with launch file',
    license='MIT',
    tests_require=['pytest'],
    
    entry_points={
    'console_scripts': [
        'odom_relay_node = vehicle_control_unit_py.odom_relay_node:main',
        'rc_pwm_to_cmd_vel_node = vehicle_control_unit_py.rc_pwm_to_cmd_vel_node:main',
        'cmdvel_to_pwm_serial_node = vehicle_control_unit_py.cmdvel_to_pwm_serial_node:main',
        'cmdvel_to_pwm_serial_pid_ctrl_node = vehicle_control_unit_py.cmdvel_to_pwm_serial_pid_ctrl_node:main',
        ],
    },

)

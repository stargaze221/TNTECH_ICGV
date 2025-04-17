from setuptools import setup

package_name = 'vehicle_control_unit_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/my_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Python-based ROS 2 node example with launch file',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_node = vehicle_control_unit_py.python_node:main',
            'pwm_passthrough = vehicle_control_unit_py.node_pass_thru:main',
            'dual_pwm_interface = vehicle_control_unit_py.node_rc_servo_pwm:main',
        ],
    },
)
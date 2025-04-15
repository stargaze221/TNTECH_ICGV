from setuptools import setup

package_name = 'vehicle_control_unit'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_both_nodes.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='My ROS2 mixed package with Python nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_node = vehicle_control_unit.python_node:main',
            'pwm_passthrough = vehicle_control_unit.node_pass_thru:main',
            'dual_pwm_interface = vehicle_control_unit.node_rc_servo_pwm:main',
        ],
    },
)

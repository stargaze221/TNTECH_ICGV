from setuptools import find_packages, setup

package_name = 'vehicle_control_unit_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoonlab07nano',
    maintainer_email='yoonlab07nano@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'python_node = vehicle_control_unit_py.python_node:main',
            'pwm_passthrough = vehicle_control_unit_py.node_pass_thru:main',
            'dual_pwm_interface = vehicle_control_unit_py.node_rc_servo_pwm:main',
        ],
    },
)

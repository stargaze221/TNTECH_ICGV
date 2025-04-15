cmake_minimum_required(VERSION 3.8)
project(vehicle_control_unit)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)

# C++ node (cpp_node.cpp)
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)
install(TARGETS cpp_node DESTINATION lib/${PROJECT_NAME})

# Install Python package using setup.py
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.5)
project(vehicle_control_unit)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

# Add C++ executable
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp std_msgs)

# Install C++ executable
install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python node (without include directories, it's not needed for Python)
install(PROGRAMS
  scripts/python_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

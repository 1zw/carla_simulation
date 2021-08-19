cmake_minimum_required(VERSION 3.5)
project(carla_simulation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Install Python Modules
ament_python_install_package(${PROJECT_NAME})

# Install Python Executables
install(PROGRAMS
  scripts/lane_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Custom Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneGeometry.msg"
  "msg/LaneParameters.msg"
 )

# Install Launch Directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Config Directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

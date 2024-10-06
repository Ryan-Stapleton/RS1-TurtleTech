cmake_minimum_required(VERSION 3.5)
project(sprint_02)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create the executable
add_executable(${PROJECT_NAME} src/sprint_02_node.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

# Link ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


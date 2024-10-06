cmake_minimum_required(VERSION 3.5)
project(sprint_03)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create the executable
add_executable(map_matching_node src/map_overlay.cpp)
add_executable(object_detection_node src/obj_detect.cpp)

# Link libraries
target_link_libraries(map_matching_node
  ${OpenCV_LIBRARIES}
)

# Link libraries
target_link_libraries(object_detection_node
  ${OpenCV_LIBRARIES}
)

# Link ROS2 dependencies
ament_target_dependencies(map_matching_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  cv_bridge
)

# Link ROS2 dependencies
ament_target_dependencies(object_detection_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  cv_bridge
)


# Install the executable
install(TARGETS map_matching_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the executable
install(TARGETS object_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
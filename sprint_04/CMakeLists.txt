cmake_minimum_required(VERSION 3.8)
project(sprint_04)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)  # Add this if you plan to use OpenCV functions
find_package(nav_msgs REQUIRED)  # Add this line for odometry messages
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add the executable for the node
add_executable(rgbDetect src/rgbDetect.cpp)

# Link the required libraries
ament_target_dependencies(rgbDetect
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV  # Add this if OpenCV functions are used
  nav_msgs  # Link the nav_msgs dependency
  tf2
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS
  rgbDetect
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(x4_service_scripts)

# # Enable C++17
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ardupilot_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
# Include directories
include_directories(include)

add_library(copter_waypoint_node SHARED src/copter_waypoint_follower.cpp)
ament_target_dependencies(copter_waypoint_node rclcpp ardupilot_msgs geometry_msgs std_msgs geographic_msgs)

add_executable(copter_waypoint src/copter_waypoint_main.cpp)
target_link_libraries(copter_waypoint copter_waypoint_node)
ament_target_dependencies(copter_waypoint rclcpp ardupilot_msgs geometry_msgs std_msgs)

install( TARGETS copter_waypoint_node LIBRARY DESTINATION lib)
install( TARGETS copter_waypoint DESTINATION lib/${PROJECT_NAME})

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

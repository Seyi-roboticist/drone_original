cmake_minimum_required(VERSION 3.8)
project(x4_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(v4l2_camera REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(include)

add_executable(aruco_detector_node src/aruco_detector.cpp)
ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
  vision_msgs
  geometry_msgs
)

target_link_libraries(aruco_detector_node ${OpenCV_LIBS})

install(TARGETS aruco_detector_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz include DESTINATION share/${PROJECT_NAME})

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

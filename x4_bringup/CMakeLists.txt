cmake_minimum_required(VERSION 3.8)
project(x4_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(x4_description REQUIRED)
find_package(x4_gazebo REQUIRED)
find_package(x4_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(mavros REQUIRED)

install( DIRECTORY launch DESTINATION share/${PROJECT_NAME} )
install( DIRECTORY config DESTINATION share/${PROJECT_NAME} )

ament_package()

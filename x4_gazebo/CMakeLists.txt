cmake_minimum_required(VERSION 3.8)
project(x4_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-cmake2 REQUIRED)

find_package(ardupilot_msgs REQUIRED)
find_package(ardupilot_sitl REQUIRED)
find_package(micro_ros_agent REQUIRED)

ign_find_package( ignition-plugin1 REQUIRED COMPONENTS register )
ign_find_package( ignition-gazebo6 REQUIRED )

install(DIRECTORY 
  world 
  config 
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(tello_teleop)

add_compile_options(-g)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(PROGRAMS 
  scripts/tello_joy_teleop
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
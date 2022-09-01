cmake_minimum_required(VERSION 3.8)
project(tello_bringup)

add_compile_options(-g)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tello_msgs REQUIRED)
find_package(image_transport REQUIRED)

install(PROGRAMS 
  scripts/base_topics_handler
  scripts/image_saver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

set(VIDEO_INTERFACE_DEPS
  cv_bridge
  geometry_msgs
  OpenCV
  rclcpp
  sensor_msgs
  std_msgs
  tello_msgs
  image_transport
)

add_executable(video_interface src/video_interface.cpp)
ament_target_dependencies(video_interface ${VIDEO_INTERFACE_DEPS})

install(TARGETS 
  video_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
cmake_minimum_required(VERSION 3.8)
project(gesture_steering)

add_compile_options(-g)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

#include_directories(
#  include
#)

install(PROGRAMS 
  scripts/gesture_steering
  scripts/hand_tracking.py
  scripts/math_gesture_interpreter.py
  scripts/data_generator.py
  scripts/hand_mask_recorder
  scripts/movement_steering
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
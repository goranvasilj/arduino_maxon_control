cmake_minimum_required(VERSION 2.8.3)
project(motor_control_arduino)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  sensor_msgs
  std_msgs
  message_generation
)


catkin_package(
  CATKIN_DEPENDS
    serial
    sensor_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_control_arduino src/motor_control_arduino.cpp)



target_link_libraries(motor_control_arduino
  ${catkin_LIBRARIES}
)


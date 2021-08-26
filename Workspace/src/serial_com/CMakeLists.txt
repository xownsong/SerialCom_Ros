cmake_minimum_required(VERSION 2.8.3)
project(serial_com)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_node_decision src/serial_node_decision.cpp)

target_link_libraries(serial_node_decision
  ${catkin_LIBRARIES}
)




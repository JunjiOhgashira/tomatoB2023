cmake_minimum_required(VERSION 3.0.2)
project(tomato_dynamixel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_sdk
  command_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamixel_controller src/dynamixel_controller.cpp)
target_link_libraries(dynamixel_controller ${catkin_LIBRARIES})

add_executable(arm_ik_node src/arm_ik_node.cpp)
target_link_libraries(arm_ik_node ${catkin_LIBRARIES})

add_executable(elevator_commander src/elevator_commander.cpp)
target_link_libraries(elevator_commander ${catkin_LIBRARIES})
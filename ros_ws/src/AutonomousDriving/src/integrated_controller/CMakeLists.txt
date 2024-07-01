cmake_minimum_required(VERSION 3.0.2)
project(integrated_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  mav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs mav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(integrated_controller_node src/integrated_controller.cpp)
target_link_libraries(integrated_controller_node
  ${catkin_LIBRARIES}
)

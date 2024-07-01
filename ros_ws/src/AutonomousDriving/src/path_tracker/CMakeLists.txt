cmake_minimum_required(VERSION 3.0.2)
project(path_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_tracker_node src/path_tracker_node.cpp)

target_link_libraries(path_tracker_node
  ${catkin_LIBRARIES}
)

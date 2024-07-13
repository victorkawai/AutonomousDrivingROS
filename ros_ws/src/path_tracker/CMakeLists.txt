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
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/path_tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

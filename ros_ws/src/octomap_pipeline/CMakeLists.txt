cmake_minimum_required(VERSION 2.8.3)
project(octomap_pipeline)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  depth_image_proc
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros depth_image_proc
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/fix_depth_frame.py
  src/traffic_light_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

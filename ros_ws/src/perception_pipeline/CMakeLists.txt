cmake_minimum_required(VERSION 2.8.3)
project(perception_pipeline)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  depth_image_proc
  message_generation
)

add_message_files(
  FILES
  RedLightDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_ros depth_image_proc message_runtime
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

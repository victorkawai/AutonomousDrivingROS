cmake_minimum_required(VERSION 3.0.2)
project(semaphore_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(semaphore_detection_node src/semaphore_detection.cpp)
target_link_libraries(semaphore_detection_node
  ${catkin_LIBRARIES}
)

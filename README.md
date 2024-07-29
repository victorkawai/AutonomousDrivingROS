# AutonomousDrivingROS
Introduction to ROS  - TUM

Ubuntu 20.04 used with ros-noetic-desktop-full.

Navigate to ros_ws/ and execute ```catkin build``` then ```source devel/setup.bash```.

Launch with ```roslaunch simulation simulation.launch```!

NOTE: Due to the large file size, the unity simulation files are not inclued in the repository and you will need to copy them to devel/lib/simulation/ manually.

# Dependencies
(Assumes you have the ros-noetic-desktop-full installation)
* ros-noetic-octomap-server
* (ros-noetic-octomap-rviz-plugins), only needed for visualization, but recommended
* ros-noetic-navigation
* ros-noetic-move-base
* sudo apt-get install ros-noetic-catkin python3-catkin-tools



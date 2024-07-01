#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

nav_msgs::Path path;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  path.header = msg->header;  // Use the same header as the pose message
  path.poses.push_back(*msg); // Add the received pose to the path
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_tracker");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub = nh.subscribe("/unity_ros/OurCar/Sensors/IMU/pose", 10, poseCallback);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("car_path", 10, true);

  path.header.frame_id = "body";  // Set the frame to "body" as per your message

  ros::Rate rate(10.0);
  while (ros::ok()) {
    path_pub.publish(path);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

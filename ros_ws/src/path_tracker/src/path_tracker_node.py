#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathCreator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('path_tracker_node', anonymous=True)

        # Subscriber to the pose topic
        self.pose_sub = rospy.Subscriber('/unity_ros/OurCar/Sensors/IMU/pose', PoseStamped, self.pose_callback)

        # Publisher for the path topic
        self.path_pub = rospy.Publisher('/car_path', Path, queue_size=10)

        # Initialize the Path message
        self.path = Path()
        self.path.header.frame_id = "world"  # Use the appropriate frame ID

    def pose_callback(self, pose_msg):
        # Append the new pose to the path
        self.path.poses.append(pose_msg)

        # Update the header timestamp
        self.path.header.stamp = rospy.Time.now()

        # Publish the path
        self.path_pub.publish(self.path)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        path_creator = PathCreator()
        path_creator.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
from time import sleep

class CheckpointHandler:
    def __init__(self):
        self.checkpoints = [
            Point(-8.0, -62.0, 1.0),
            Point(-24.0, -62.0, 1.0),
            Point(-40.0, -58.0, 1.0),
            Point(-50.0, -48.0, 1.0)
        ]

        # These are the real checkpoints.
        # self.checkpoints = [
        #     Point(-12.0, -62.0, 1.0),
        #     Point(-53.0, 112.0, 1.0),
        #     Point(-12.0, 123.0, 1.0),
        #     Point(-2.0, 218.0, 1.0),
        #     Point(-53.0, 53.0, 1.0),
        #     Point(-15.0, 45.0, 1.0),
        #     Point(-1.0, -52.0, 1.0)
        # ]


        self.current_checkpoint_index = 0
        self.threshold_radius = 5
        self.costmap_received = False # To make sure the first checkpoint is published after the costmaps exist.

        rospy.init_node('checkpoint_handler', anonymous=True)

        self.pose_subscriber = rospy.Subscriber('/pose_est', PoseStamped, self.pose_callback)
        self.costmap_subscriber = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
        self.goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    
    def costmap_callback(self, costmap_msg):
        if not self.costmap_received:
            self.costmap_received = True
            sleep(4)
            rospy.loginfo("Costmap received. Publishing the first checkpoint.")
            self.publish_goal(self.checkpoints[self.current_checkpoint_index])

    def pose_callback(self, pose_msg):

        if not self.costmap_received:
            rospy.loginfo("Waiting for costmap...")
            sleep(2)
            return
        
        if self.current_checkpoint_index >= len(self.checkpoints):
            # rospy.loginfo("All checkpoints reached!")
            return
        
        current_position = pose_msg.pose.position
        checkpoint_position = self.checkpoints[self.current_checkpoint_index]
        
        distance = self.calculate_distance(current_position, checkpoint_position)
        # rospy.loginfo(f"Distance to checkpoint: {distance}")

        if distance < self.threshold_radius:
            self.current_checkpoint_index += 1
            self.publish_goal(self.checkpoints[self.current_checkpoint_index])

    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)

    def publish_goal(self, checkpoint):
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.header.frame_id = "world"
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position = checkpoint
        goal.goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Publishing goal to move_base: {checkpoint}")
        self.goal_publisher.publish(goal)

if __name__ == '__main__':
    try:
        navigator = CheckpointHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

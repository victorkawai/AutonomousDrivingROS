#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class PathPlanningNode:
    def __init__(self):
        rospy.init_node('path_planning_node')

        # Action client to send goals to the move_base server
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # Subscriber to the target checkpoint topic
        # NOTE: Can publish manually: rostopic pub /checkpoint geometry_msgs/PoseStamped "{header: {seq: 42, stamp: {secs: 42, nsecs: 42}, frame_id: 'world'}, pose: {position: {x: 0.0, y: -60.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
        self.checkpoint_sub = rospy.Subscriber('/checkpoint', PoseStamped, self.checkpoint_callback)

    def checkpoint_callback(self, msg):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "world"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = msg.pose

        rospy.loginfo("Sending goal: ({}, {}, {})".format(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        self.move_base_client.send_goal(goal, done_cb=self.done_callback, active_cb=self.active_callback, feedback_cb=self.feedback_callback)

    def done_callback(self, status, result):
        rospy.loginfo("Goal reached!")

    def active_callback(self):
        rospy.loginfo("Goal is active.")

    def feedback_callback(self, feedback):
        rospy.loginfo("Current position: ({}, {})".format(feedback.base_position.pose.position.x, feedback.base_position.pose.position.y))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PathPlanningNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

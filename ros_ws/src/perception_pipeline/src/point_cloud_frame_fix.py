#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

def callback(data):
    data.header.frame_id = "map"
    pub.publish(data)

rospy.init_node('fix_point_cloud_frame_id', anonymous=True)
pub = rospy.Publisher('/depth_registered/points_corrected', PointCloud2, queue_size=10)
rospy.Subscriber('/depth_registered/points', PointCloud2, callback)
rospy.spin()

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

def callback(data):
    data.header.frame_id = "OurCar/Sensors/RGBCameraLeft"
    pub.publish(data)

rospy.init_node('fix_depth_frame_id', anonymous=True)
pub = rospy.Publisher('/unity_ros/OurCar/Sensors/DepthCamera/image_raw_fixed', Image, queue_size=10)
rospy.Subscriber('/unity_ros/OurCar/Sensors/DepthCamera/image_raw', Image, callback)
rospy.spin()

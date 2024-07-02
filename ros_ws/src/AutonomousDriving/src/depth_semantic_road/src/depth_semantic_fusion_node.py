#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from sensor_msgs import point_cloud2
import cv_bridge

class DepthSemanticRoadNode:
    def __init__(self):
        rospy.init_node('depth_semantic_road_node')
        
        self.bridge = cv_bridge.CvBridge()
        self.depth_sub = rospy.Subscriber('/unity_ros/OurCar/Sensors/DepthCamera/image_raw', Image, self.depth_callback)
        self.semantic_sub = rospy.Subscriber('/unity_ros/OurCar/Sensors/SemanticCamera/image_raw', Image, self.semantic_callback)
        self.camera_info_sub = rospy.Subscriber('unity_ros/OurCar/Sensors/DepthCamera/camera_info', CameraInfo, self.camera_info_callback)
        self.point_cloud_pub = rospy.Publisher('/road_point_cloud', PointCloud2, queue_size=10) 
        
        self.depth_image = None
        self.semantic_image = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

    def semantic_callback(self, msg):
        try:
            self.semantic_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def create_point_cloud(self):
        if self.depth_image is None or self.semantic_image is None:
            return
        
        points = []
        for v in range(self.depth_image.shape[0]):
            for u in range(self.depth_image.shape[1]):
                if self.semantic_image[v, u] == 127:  # Grey color representing the road
                    z = self.depth_image[v, u] / 1000.0  # Convert from mm to meters
                    if z > 0:  # Avoid points with zero depth
                        x = (u - self.depth_image.shape[1] / 2) * z / self.fx
                        y = (v - self.depth_image.shape[0] / 2) * z / self.fy
                        points.append([x, z, y])

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'true_body'
        point_cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.point_cloud_pub.publish(point_cloud_msg)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.create_point_cloud()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = DepthSemanticRoadNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

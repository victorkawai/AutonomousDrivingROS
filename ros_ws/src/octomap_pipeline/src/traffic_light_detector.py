#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

class RedLightDetectionNode:
    def __init__(self):
        rospy.init_node('red_light_detection_node', anonymous=True)

        self.bridge = CvBridge()

        # Publishers.
        self.red_light_pub = rospy.Publisher('/red_light', Bool, queue_size=10)
        self.segmented_image_pub = rospy.Publisher('/segmented_image', Image, queue_size=10)
        
        # Subscriber.
        self.rgb_image_sub = rospy.Subscriber('/unity_ros/OurCar/Sensors/RGBCameraLeft/image_raw', Image, self.rgb_image_callback)
        self.semantic_image_sub = rospy.Subscriber('/unity_ros/OurCar/Sensors/SemanticCamera/image_raw', Image, self.semantic_image_callback)

        self.rgb_image = None
        self.semantic_image = None

    def rgb_image_callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def semantic_image_callback(self, data):
        try:
            self.semantic_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def process_images(self):

        if self.rgb_image is None or self.semantic_image is None:
            return
        
        # Segment traffic lights from the semantic image.
        traffic_light_mask = self.semantic_image == 215
        
        # Apply the mask to the RGB image.
        segmented_rgb_image = cv2.bitwise_and(self.rgb_image, self.rgb_image, mask=traffic_light_mask.astype(np.uint8))
        
        # Define range for red color and create a mask.
        # NOTE: This is encoded bgr8, so red is the last value.
        lower_red1 = np.array([0, 0, 250])
        upper_red1 = np.array([100, 100, 255])
        red_mask = cv2.inRange(segmented_rgb_image, lower_red1, upper_red1)

        # Check if there are any red pixels in the segmented image.
        red_detected = np.any(red_mask > 0)

        # Publish the result.
        self.red_light_pub.publish(Bool(red_detected))

        # Publish the segmented RGB image.
        try:
            segmented_rgb_msg = self.bridge.cv2_to_imgmsg(segmented_rgb_image, "bgr8")
            self.segmented_image_pub.publish(segmented_rgb_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.process_images()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = RedLightDetectionNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

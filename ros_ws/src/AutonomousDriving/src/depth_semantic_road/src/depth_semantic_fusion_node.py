#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import tf.transformations
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, Imu
from geometry_msgs.msg import TwistStamped
from sensor_msgs import point_cloud2
import cv_bridge

class DepthSemanticRoadNode:
    def __init__(self):
        rospy.init_node('depth_semantic_road_node')
        
        self.bridge = cv_bridge.CvBridge()

        # Subscribe to the depth and semantic cameras, to fuse the images into
        # a segmented road pointcloud.
        self.depth_sub = rospy.Subscriber('/unity_ros/OurCar/Sensors/DepthCamera/image_raw', Image, self.depth_callback)
        self.semantic_sub = rospy.Subscriber('/unity_ros/OurCar/Sensors/SemanticCamera/image_raw', Image, self.semantic_callback)

        # Subscribe to the camera info to use the camera parameters for correct
        # distance estimation.
        self.camera_info_sub = rospy.Subscriber('unity_ros/OurCar/Sensors/DepthCamera/camera_info', CameraInfo, self.camera_info_callback)

        # Subscribe to the imu twist to compensate movement of the car, which
        # would interfere with the correct position estimation due to tilt and
        # elevation changes of the cameras.
        self.imu_twist_sub = rospy.Subscriber('/unity_ros/OurCar/Sensors/IMU/twist', TwistStamped, self.imu_twist_callback)


        # Publish the point cloud representing the road as free space and the
        # obstacles as occupied space. This will then be subscribed to by the
        # octree server node which will build an octogrid
        # from it.
        # TODO: Think about the 'queue_size' parameter.
        self.point_cloud_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
        
        # Intit intrinsic params.
        self.depth_image = None
        self.semantic_image = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

    def depth_callback(self, msg):
        """Parse depth image."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

    def semantic_callback(self, msg):
        """Parse semantic image."""
        try:
            # NOTE: With the mono8 encoding, gray has a value of 127, yellow is
            #   	215 and red is 76.
            self.semantic_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

    def camera_info_callback(self, msg):
        """Parse camera params."""
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def imu_twist_callback(self, msg):
        """Parse IMU twist data."""
        self.orientation = np.array([msg.twist.angular.x, msg.twist.angular.z, msg.twist.angular.y, 1.0])

    def create_point_cloud(self):
        """Segment depth image with semantic camera into road pointcloud."""

        if self.depth_image is None or self.semantic_image is None:
            return
        
        points = []
        
        # Fuse both images and apply camera params for correct depth.
        # Iterate over all pixels.
        for v in range(self.depth_image.shape[0]):
            for u in range(self.depth_image.shape[1]):

                # Grey color (127) represents the road and red color (76)
                # represents obstacles.
                if self.semantic_image[v, u] == 127 or \
                                            self.semantic_image[v, u] == 76:
                    
                    # Convert from mm to meters.
                    z = self.depth_image[v, u] / 1000.0

                    # Avoid points with zero depth.
                    if z > 0:

                        x = (u - self.depth_image.shape[1] / 2) * z / self.fx
                        y = (v - self.depth_image.shape[0] / 2) * z / self.fy

                        # Swap y- and z-axis for correct orientation of the
                        # pointcloud since its not actually the z-axis we're
                        # looking at.
                        if self.semantic_image[v, u] == 127:

                            # NOTE: Fix the height coordinate at 0, because the
                            #       height component is inaccurate due to
                            #       movement of the car anyways.
                            # Intesity of -1 represents free space.
                            points.append([x, z, 0.1, -1])
                        elif self.semantic_image[v, u] == 76:

                            # NOTE: Height also doesn't matter here but it
                            #       looks cooler like this :D
                            # Intensity of 1 represents occupied space.
                            points.append([x, z, y, 1])

        # Build header, points should be attached to 'true_body'.
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'true_body'

        # Publish pointcloud.
        point_cloud_msg = point_cloud2.create_cloud(header, [
            point_cloud2.PointField('x', 0, point_cloud2.PointField.FLOAT32, 1),
            point_cloud2.PointField('y', 4, point_cloud2.PointField.FLOAT32, 1),
            point_cloud2.PointField('z', 8, point_cloud2.PointField.FLOAT32, 1),
            point_cloud2.PointField('intensity', 12, point_cloud2.PointField.FLOAT32, 1),
        ], points)
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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

ros::Publisher transformed_cloud_pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert the sensor_msgs/PointCloud2 to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Define a rotation matrix to transform the point cloud
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // Rotate around the x-axis to swap y and z
    transform(0, 0) = 1;  // Flip the x-axis
    transform(0, 1) = 0;
    transform(0, 2) = 0;
    transform(1, 0) = 0;
    transform(1, 1) = 0;
    transform(1, 2) = 1;
    transform(2, 0) = 0;
    transform(2, 1) = -1;
    transform(2, 2) = 0;

    // Apply the transformation
    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    pcl::transformPointCloud(cloud, transformed_cloud, transform);

    // Convert the transformed pcl::PointCloud back to sensor_msgs/PointCloud2
    sensor_msgs::PointCloud2 transformed_cloud_msg;
    pcl::toROSMsg(transformed_cloud, transformed_cloud_msg);
    transformed_cloud_msg.header = cloud_msg->header;

    // Publish the transformed point cloud
    transformed_cloud_pub.publish(transformed_cloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_pointcloud_node");
    ros::NodeHandle nh;

    // Subscribe to the input point cloud topic
    ros::Subscriber sub = nh.subscribe("/depth_registered/points", 1, pointCloudCallback);

    // Advertise the transformed point cloud topic
    transformed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/depth_registered/transformed_points", 1);

    ros::spin();

    return 0;
}

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>

class ObstacleDetector {
public:
  ObstacleDetector(ros::NodeHandle& nh) {
    semantic_sub = nh.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 1, &ObstacleDetector::semanticCallback, this);
    obstacle_pub = nh.advertise<std_msgs::Bool>("obstacle_detected", 1);
  }

  void semanticCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;
    bool obstacle = detectObstacles(frame);

    std_msgs::Bool obstacle_msg;
    obstacle_msg.data = obstacle;
    obstacle_pub.publish(obstacle_msg);
  }

private:
  ros::Subscriber semantic_sub;
  ros::Publisher obstacle_pub;

  bool detectObstacles(const cv::Mat& frame) {
    // Assume that the semantic segmentation labels obstacles with a specific color or label.
    // For example, obstacles are labeled with red color in the segmentation image.
    cv::Mat obstacle_mask;
    cv::inRange(frame, cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 255), obstacle_mask); // Detect red color

    // Check if there are any red pixels indicating obstacles
    return cv::countNonZero(obstacle_mask) > 0;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_detector");
  ros::NodeHandle nh;
  ObstacleDetector obstacle_detector(nh);
  ros::spin();
  return 0;
}

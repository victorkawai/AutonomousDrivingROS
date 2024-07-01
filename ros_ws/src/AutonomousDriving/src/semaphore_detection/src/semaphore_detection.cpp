#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

class SemaphoreDetector {
public:
  SemaphoreDetector(ros::NodeHandle& nh) {
    semantic_sub = nh.subscribe("/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 1, &SemaphoreDetector::semanticCallback, this);
    semaphore_pub = nh.advertise<std_msgs::String>("semaphore_color", 1);
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
    cv::Mat traffic_light_mask;
    
    // Use the exact BGR value for the traffic light label with a small range
    cv::Scalar lower_yellow(245, 225, 0);
    cv::Scalar upper_yellow(265, 245, 8);
    cv::inRange(frame, lower_yellow, upper_yellow, traffic_light_mask);

    // Find the bounding box of the traffic light region
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(traffic_light_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (contours.empty()) {
      ROS_WARN("No traffic light detected in semantic image.");
      return;
    }

    // Filter contours based on size and position to reduce false positives
    std::vector<cv::Rect> traffic_light_bboxes;
    for (const auto& contour : contours) {
      cv::Rect bbox = cv::boundingRect(contour);
      // Example size and position filtering
      if (bbox.width > 10 && bbox.height > 10 && bbox.width < 100 && bbox.height < 100 && bbox.y < frame.rows / 2) {
        traffic_light_bboxes.push_back(bbox);
      }
    }

    if (traffic_light_bboxes.empty()) {
      ROS_WARN("No valid traffic light detected in semantic image.");
      return;
    }

    // Assuming the largest bounding box is the traffic light
    cv::Rect largest_bbox = *std::max_element(traffic_light_bboxes.begin(), traffic_light_bboxes.end(),
                                              [](const cv::Rect& a, const cv::Rect& b) { return a.area() < b.area(); });

    // Extract the RoI from the original frame
    cv::Mat traffic_light_roi = frame(largest_bbox);

    // Detect the color of the traffic light within the RoI
    std::string color = detectSemaphoreColor(traffic_light_roi);

    std_msgs::String color_msg;
    color_msg.data = color;
    semaphore_pub.publish(color_msg);
  }

private:
  ros::Subscriber semantic_sub;
  ros::Publisher semaphore_pub;

  std::string detectSemaphoreColor(const cv::Mat& frame) {
    // Define the BGR values for red, green, and yellow
    cv::Scalar lower_red(245, 60, 59);
    cv::Scalar upper_red(265, 80, 79);
    cv::Scalar lower_green(38, 245, 43);
    cv::Scalar upper_green(58, 265, 63);
    cv::Scalar lower_yellow(245, 225, 0);
    cv::Scalar upper_yellow(265, 245, 8);

    // Threshold the image to get only the specified colors
    cv::Mat mask_red, mask_green, mask_yellow;
    cv::inRange(frame, lower_red, upper_red, mask_red);
    cv::inRange(frame, lower_green, upper_green, mask_green);
    cv::inRange(frame, lower_yellow, upper_yellow, mask_yellow);

    // Check if there are any red pixels
    if (cv::countNonZero(mask_red) > 0) {
      return "RED";
    }

    // Check if there are any green pixels
    if (cv::countNonZero(mask_green) > 0) {
      return "GREEN";
    }

    // Check if there are any yellow pixels
    if (cv::countNonZero(mask_yellow) > 0) {
      return "YELLOW";
    }

    return "UNKNOWN";
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "semaphore_detector");
  ros::NodeHandle nh;
  SemaphoreDetector semaphore_detector(nh);
  ros::spin();
  return 0;
}

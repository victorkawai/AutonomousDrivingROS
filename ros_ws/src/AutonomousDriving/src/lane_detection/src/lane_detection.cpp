#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <opencv2/opencv.hpp>

class LaneDetector {
public:
  LaneDetector(ros::NodeHandle& nh) {
    image_sub = nh.subscribe("/unity_ros/OurCar/Sensors/RGBCameraLeft/image_raw", 1, &LaneDetector::imageCallback, this);
    lane_offset_pub = nh.advertise<std_msgs::Float64>("lane_offset", 1);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;
    double lane_center_offset = findLaneCenter(frame);

    std_msgs::Float64 offset_msg;
    offset_msg.data = lane_center_offset;
    lane_offset_pub.publish(offset_msg);
  }

private:
  ros::Subscriber image_sub;
  ros::Publisher lane_offset_pub;

  double findLaneCenter(const cv::Mat& frame) {
    cv::Mat gray, blurred, edges;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    cv::Canny(blurred, edges, 50, 150);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, 50, 50, 10);

    std::vector<double> slopes;
    std::vector<cv::Point> points;
    for (const auto& line : lines) {
      double slope = static_cast<double>(line[3] - line[1]) / (line[2] - line[0]);
      slopes.push_back(slope);
      points.push_back(cv::Point(line[0], line[1]));
      points.push_back(cv::Point(line[2], line[3]));
    }

    cv::Point lane_center(0, 0);
    if (!points.empty()) {
      for (const auto& point : points) {
        lane_center += point;
      }
      lane_center.x /= points.size();
      lane_center.y /= points.size();
    }

    double image_center = frame.cols / 2.0;
    return (lane_center.x - image_center) / image_center;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lane_detector");
  ros::NodeHandle nh;
  LaneDetector lane_detector(nh);
  ros::spin();
  return 0;
}

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>

class IntegratedController {
public:
  IntegratedController(ros::NodeHandle& nh) :
    wheel_base(1.0),
    max_steering_angle(0.5),
    k_p(1.0),
    next_checkpoint(0),
    stop(false),
    slow_down(false)
  {
    lane_offset_sub = nh.subscribe("lane_offset", 1, &IntegratedController::laneOffsetCallback, this);
    obstacle_sub = nh.subscribe("obstacle_detected", 1, &IntegratedController::obstacleCallback, this);
    semaphore_sub = nh.subscribe("semaphore_color", 1, &IntegratedController::semaphoreCallback, this);
    // odom_sub = nh.subscribe("/unity_ros/OurCar/Sensors/IMU/pose", 1, &IntegratedController::odomCallback, this);
    cmd_pub = nh.advertise<mav_msgs::Actuators>("car_commands", 1);

    // Define checkpoints
    checkpoints = {
      {1.0, 1.0}, {2.0, 1.0}, {2.0, 2.0}, {3.0, 2.0}, {3.0, 3.0}, {4.0, 3.0}
    };
  }

  void laneOffsetCallback(const std_msgs::Float64::ConstPtr& msg) {
    lane_center_offset = msg->data;
    publishControlCommand();
  }

  void obstacleCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
      // Obstacle detected, change path to avoid collision
      avoidObstacle();
    } else {
      // No obstacle, continue on current path
      moveForward();
    }
  }

  void semaphoreCallback(const std_msgs::String::ConstPtr& msg) {
    std::string color = msg->data;
    if (color == "RED") {
      stop = true;
      slow_down = false;
    } else if (color == "YELLOW") {
      stop = false;
      slow_down = true;
    } else if (color == "GREEN") {
      stop = false;
      slow_down = false;
    }
    publishControlCommand();
  }

  // void odomCallback(const nav_msgs::Odometry& msg) {
  //   double x = msg.pose.pose.position.x;
  //   double y = msg.pose.pose.position.y;
  //   if (next_checkpoint < checkpoints.size()) {
  //     double cp_x = checkpoints[next_checkpoint].first;
  //     double cp_y = checkpoints[next_checkpoint].second;
  //     double dist = sqrt(pow(cp_x - x, 2) + pow(cp_y - y, 2));
  //     if (dist < 0.5) { // Threshold to consider reaching a checkpoint
  //       ++next_checkpoint;
  //     }
  //   }
  // }

private:
  ros::Subscriber lane_offset_sub;
  ros::Subscriber obstacle_sub;
  ros::Subscriber semaphore_sub;
  // ros::Subscriber odom_sub;
  ros::Publisher cmd_pub;

  std::vector<std::pair<double, double>> checkpoints;
  size_t next_checkpoint;
  double wheel_base;
  double max_steering_angle;
  double k_p;
  bool stop;
  bool slow_down;
  double lane_center_offset;

  void avoidObstacle() {
    mav_msgs::Actuators actuators_msg;

    // Logic to change path to avoid obstacle
    actuators_msg.angular_velocities.resize(4);
    actuators_msg.angular_velocities[0] = 0.0; // No forward motion
    actuators_msg.angular_velocities[1] = 1.0; // Turn to avoid obstacle
    actuators_msg.angular_velocities[2] = 0.0; // No braking
    actuators_msg.angular_velocities[3] = 0.0; // Unused

    cmd_pub.publish(actuators_msg);
  }

  void moveForward() {
    mav_msgs::Actuators actuators_msg;

    // Logic to move forward
    actuators_msg.angular_velocities.resize(4);
    actuators_msg.angular_velocities[0] = 0.5; // Forward motion
    actuators_msg.angular_velocities[1] = 0.0; // No turning
    actuators_msg.angular_velocities[2] = 0.0; // No braking
    actuators_msg.angular_velocities[3] = 0.0; // Unused

    cmd_pub.publish(actuators_msg);
  }

  void publishControlCommand() {
    mav_msgs::Actuators actuators_msg;
    actuators_msg.angular_velocities.resize(4);

    if (stop) {
      actuators_msg.angular_velocities[0] = 0.0;
      actuators_msg.angular_velocities[1] = 0.0;
      actuators_msg.angular_velocities[2] = 0.0;
      actuators_msg.angular_velocities[3] = 0.0;
    } else if (slow_down) {
      actuators_msg.angular_velocities[0] = 0.2; // Slow down
      actuators_msg.angular_velocities[1] = -lane_center_offset * 0.1;
      actuators_msg.angular_velocities[2] = 0.0; // No braking
      actuators_msg.angular_velocities[3] = 0.0; // Unused
    } else {
      actuators_msg.angular_velocities[0] = 1.0;
      actuators_msg.angular_velocities[1] = -lane_center_offset * 0.1;
      actuators_msg.angular_velocities[2] = 0.0; // No braking
      actuators_msg.angular_velocities[3] = 0.0; // Unused
    }
    cmd_pub.publish(actuators_msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "integrated_controller");
  ros::NodeHandle nh;
  IntegratedController integrated_controller(nh);
  ros::spin();
  return 0;
}

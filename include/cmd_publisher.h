#ifndef ROS2_TUTORIAL_CMD_PUBLISHER_H
#define ROS2_TUTORIAL_CMD_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

const double PI = 3.14159265358979323846;

class CmdPublisher : public rclcpp::Node {
public:
  CmdPublisher();

private:
  void timer_tf_callback();

  void timer_cmd_callback();

  // double G1[3];
  // double G2[3];
  double G1[6];
  std::string robot_name;

  int a = 0;

  double real_x = 0;
  double real_y = 0;
  double real_th = 0;
  
  double dt = 0.01;

  double goal_x, goal_y, goal_th;

  double prev_error_d = 0;
  double prev_error_th = 0;

  double i_error_d = 0;
  double i_error_th = 0;

  double k_d[3] = {0.12, 0.05, 0.001}; // {0.2, 0.05, 0.001}
  double k_th[3] = {0.3, 0.03, 0.001}; // {0.3, 0.03, 0.001}

  bool tf_flag = false;

  rclcpp::TimerBase::SharedPtr timer_cmd, timer_tf;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_pose;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};

#endif //ROS2_TUTORIAL_CMD_PUBLISHER_H

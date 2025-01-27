#include "cmd_publisher.h"


using namespace std::chrono_literals;
using std::placeholders::_1;

CmdPublisher::CmdPublisher() : Node("cmd_publisher") {
  // Publisher
  pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // TF listener
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Timer
  timer_tf = this->create_wall_timer(
      10ms, std::bind(&CmdPublisher::timer_tf_callback, this));
  timer_cmd = this->create_wall_timer(
      10ms, std::bind(&CmdPublisher::timer_cmd_callback, this));

  // Parameters
  this->declare_parameter<std::vector<double>>("goal1", {0, 0, 0, 0, 0, 0});
  std::vector<double> goal1_param = this->get_parameter("goal1").as_double_array();
  if (goal1_param.size() == 6) {
    std::copy(goal1_param.begin(), goal1_param.end(), G1);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Goal1 must have exactly 6 elements.");
  }

  // this->declare_parameter<std::vector<double>>("goal2", {0, 0, 0});
  // std::vector<double> goal2_param = this->get_parameter("goal2").as_double_array();
  // if (goal2_param.size() == 3) {
  //   std::copy(goal2_param.begin(), goal2_param.end(), G2);
  // } else {
  //   RCLCPP_ERROR(this->get_logger(), "Goal2 must have exactly 3 elements.");
  // }

  this->declare_parameter<std::string>("robot_name", "robot_name");
  this->get_parameter("robot_name", robot_name);

}

void CmdPublisher::timer_tf_callback() {
  //TODO: implement this!
  geometry_msgs::msg::TransformStamped t;

  try {
    t = tf_buffer->lookupTransform(
      "map", robot_name + "_imu_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s",
      ex.what());
    return;
  }

  double z = t.transform.rotation.z;
  double w = t.transform.rotation.w;

  real_x = t.transform.translation.x;
  real_y = t.transform.translation.y;
  real_th = 2 * atan2(z,w);

  tf_flag = true;

}

void CmdPublisher::timer_cmd_callback() {
  if (!tf_flag){
    return;
  }
  geometry_msgs::msg::Twist cmd_vel;

  if (a==0){
    goal_x = G1[0];
    goal_y = G1[1];
  }
  else if (a==1){
    goal_x = G1[3];
    goal_y = G1[4];
  }

  double d_x = goal_x - real_x;
  double d_y = goal_y - real_y;

  double goal_th = atan2(d_y,d_x);

  double error_th = goal_th - real_th;
    if(error_th >  PI){
      error_th -= 2 * PI;
    }
    else if(error_th < -PI){
      error_th += 2 * PI;
    }
  
  double error_d = sqrt(d_x * d_x + d_y * d_y);
  if(abs(error_th) > 0.75 * PI)
      error_d = - sqrt(d_x * d_x + d_y * d_y);
  
  double d_error_d = (error_d - prev_error_d) / dt;
  double d_error_th = (error_th - prev_error_th) / dt;

  i_error_d = i_error_d + error_d * dt; 
  i_error_th = i_error_th + error_th * dt;

  
  cmd_vel.linear.x = k_d[0] * error_d + k_d[1] * d_error_d + k_d[2] * i_error_d;
  cmd_vel.angular.z =  k_th[0] * error_th + k_th[1] * d_error_th + k_th[2] * i_error_th;

  if (error_th < 0.5)
    cmd_vel.angular.z = 0;

  if (error_d < 0.1){
    RCLCPP_WARN(this->get_logger(), "distance arrived");
    if (a==0)
      goal_th = G1[2];
    else
      goal_th = G1[5];

    double error_th = goal_th - real_th;

    if(error_th >  PI)
      error_th -= 2 * PI;
    else if(error_th < -PI)
      error_th += 2 * PI;
 
    double d_error_th = (error_th - prev_error_th) / dt;
    i_error_th = i_error_th + error_th * dt;
    
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z =  k_th[0] * error_th + k_th[1] * d_error_th + k_th[2] * i_error_th;

    if (abs(error_th) < 0.1){
      RCLCPP_WARN(this->get_logger(), "angular arrived"); 
      cmd_vel.angular.z = 0;
      a++;
      a = a % 2;
      i_error_th = 0;
    }
  }

  if (cmd_vel.linear.x > 0.2)
    cmd_vel.linear.x = 0.2;
  else if (cmd_vel.linear.x < -0.2)
    cmd_vel.linear.x = -0.2;
  
  if (cmd_vel.angular.z > 2.84)
    cmd_vel.angular.z = 2.84;
  else if (cmd_vel.angular.z < -2.84)
    cmd_vel.angular.z = -2.84; 
  
  RCLCPP_INFO(this->get_logger(), "a & destination [%d, %f, %f]", a, goal_x, goal_y);
  RCLCPP_INFO(this->get_logger(), "error_d [%f, %f]", error_d, error_th);
  RCLCPP_INFO(this->get_logger(), "cmd_vel: [%f, %f]", cmd_vel.linear.x, cmd_vel.angular.z);

  prev_error_d = error_d;
  prev_error_th = error_th;

  pub_cmd->publish(cmd_vel);
}

#include "control_node.hpp"

ControlNode::ControlNode() 
  : Node("control"), control_(robot::ControlCore(this->get_logger())) {

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Control loop runs at 20 Hz (50 ms)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&ControlNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Control Node has initialized.");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
  has_path_ = !current_path_.poses.empty();
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = extractYaw(msg->pose.pose.orientation);
  has_odom_ = true;
}

void ControlNode::controlLoop() {
  if (!has_odom_) {
    return;
  }

  if (!has_path_) {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  auto cmd = control_.computeVelocityCommand(current_path_, robot_x_, robot_y_, robot_yaw_);
  if (cmd) {
    cmd_vel_pub_->publish(*cmd);
    // If commanded stop at goal reached, clear path
    if (cmd->linear.x == 0.0 && cmd->angular.z == 0.0) {
      has_path_ = false;
    }
  }
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
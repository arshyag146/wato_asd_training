#include "planner_node.hpp"

PlannerNode::PlannerNode() 
  : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer loop running at 2 Hz
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Planner Node has initialized.");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  odom_received_ = true;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  current_goal_ = msg->point;
  state_ = PlannerState::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal accepted: (%.2f, %.2f)", current_goal_.x, current_goal_.y);

  // Immediately plan and publish path
  timerCallback();
}

void PlannerNode::timerCallback() {
  if (state_ == PlannerState::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (!map_received_ || !odom_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for initial map and odom...");
      return;
    }

    double dist_to_goal = std::hypot(current_goal_.x - current_x_, current_goal_.y - current_y_);
    if (dist_to_goal <= goal_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Goal reached! Returning to WAITING_FOR_GOAL.");
      state_ = PlannerState::WAITING_FOR_GOAL;

      // Publish empty path to clear line in Foxglove
      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = this->get_clock()->now();
      empty_path.header.frame_id = "sim_world";
      path_pub_->publish(empty_path);
      return;
    }

    // Run A*
    auto points = planner_.planPath(current_map_, current_x_, current_y_, current_goal_.x, current_goal_.y);

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "sim_world";

    for (const auto &pt : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position = pt;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
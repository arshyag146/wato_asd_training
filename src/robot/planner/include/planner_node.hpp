#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "planner_core.hpp"

enum class PlannerState {
  WAITING_FOR_GOAL,
  WAITING_FOR_ROBOT_TO_REACH_GOAL
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void timerCallback();

    robot::PlannerCore planner_;
    PlannerState state_ = PlannerState::WAITING_FOR_GOAL;

    // ROS constructs
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data buffers
    nav_msgs::msg::OccupancyGrid current_map_;
    bool map_received_ = false;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    bool odom_received_ = false;

    geometry_msgs::msg::Point current_goal_;
    const double goal_tolerance_ = 0.3; // 30cm arrival radius
};

#endif
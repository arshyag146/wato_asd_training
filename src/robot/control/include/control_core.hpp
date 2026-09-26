#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <vector>
#include <cmath>
#include <optional>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    // Pure Pursuit parameters
    static constexpr double LOOKAHEAD_DISTANCE = 0.6; // meters ahead to track
    static constexpr double LINEAR_VELOCITY = 0.5;    // cruising speed m/s
    static constexpr double GOAL_TOLERANCE = 0.25;    // meters to consider arrived
    static constexpr double MAX_ANGULAR_VEL = 1.5;    // max rad/s steering rate

    std::optional<geometry_msgs::msg::Twist> computeVelocityCommand(
      const nav_msgs::msg::Path &path,
      double robot_x, double robot_y, double robot_yaw);

  private:
    std::optional<geometry_msgs::msg::Point> findLookaheadPoint(
      const nav_msgs::msg::Path &path,
      double robot_x, double robot_y);

    rclcpp::Logger logger_;
};

}

#endif
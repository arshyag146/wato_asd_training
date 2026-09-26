#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

std::optional<geometry_msgs::msg::Point> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path &path,
  double robot_x, double robot_y)
{
  if (path.poses.empty()) {
    return std::nullopt;
  }

  // Iterate forward along the path to find the first point at or beyond lookahead distance
  for (const auto &pose_stamped : path.poses) {
    double dx = pose_stamped.pose.position.x - robot_x;
    double dy = pose_stamped.pose.position.y - robot_y;
    double dist = std::hypot(dx, dy);

    if (dist >= LOOKAHEAD_DISTANCE) {
      return pose_stamped.pose.position;
    }
  }

  // If all remaining points are closer than the lookahead distance, target the final waypoint
  return path.poses.back().pose.position;
}

std::optional<geometry_msgs::msg::Twist> ControlCore::computeVelocityCommand(
  const nav_msgs::msg::Path &path,
  double robot_x, double robot_y, double robot_yaw)
{
  geometry_msgs::msg::Twist cmd;

  if (path.poses.empty()) {
    return std::nullopt;
  }

  // Check distance to final goal waypoint
  const auto &final_goal = path.poses.back().pose.position;
  double dist_to_goal = std::hypot(final_goal.x - robot_x, final_goal.y - robot_y);

  if (dist_to_goal <= GOAL_TOLERANCE) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  // Select target lookahead waypoint
  auto target_pt = findLookaheadPoint(path, robot_x, robot_y);
  if (!target_pt) {
    return std::nullopt;
  }

  // Transform target point to robot's local body frame
  double dx = target_pt->x - robot_x;
  double dy = target_pt->y - robot_y;

  // Local coordinates: lx = forward, ly = lateral (left/right)
  double lx =  std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  double ly = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  double ld_sq = lx * lx + ly * ly;
  if (ld_sq < 1e-4) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  // Pure Pursuit curvature: kappa = 2 * y / Ld^2
  double curvature = 2.0 * ly / ld_sq;

  // Compute steering commands
  cmd.linear.x = LINEAR_VELOCITY;
  cmd.angular.z = cmd.linear.x * curvature;

  // Clamp angular velocity to prevent aggressive spins
  cmd.angular.z = std::clamp(cmd.angular.z, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

  return cmd;
}

}
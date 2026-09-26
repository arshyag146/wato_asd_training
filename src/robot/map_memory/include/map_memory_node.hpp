#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

    robot::MapMemoryCore map_memory_;

    // Subscribers & Publishers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State Tracking
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_updated_ = false;
    bool should_update_map_ = true; // Set to true so initial map publishes immediately

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;

    double last_x_ = 0.0;
    double last_y_ = 0.0;
    const double distance_threshold_ = 1.2; // Update map every 1.2 meters of movement
};

#endif
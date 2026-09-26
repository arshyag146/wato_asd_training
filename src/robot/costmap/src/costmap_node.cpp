#include <chrono>
#include <memory>
#include <cmath> 

#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  // Subscribe to /lidar
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // Publisher for /costmap
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  RCLCPP_INFO(this->get_logger(), "Costmap Node has initialized.");
}
 void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_.resetGrid();

  // Convert polar coordinates to Cartesian (x, y)
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double r = scan->ranges[i];

    if (r >= scan->range_min && r <= scan->range_max && !std::isinf(r) && !std::isnan(r)) {
      double angle = scan->angle_min + i * scan->angle_increment;
      double x = r * std::cos(angle);
      double y = r * std::sin(angle);
      costmap_.addLaserPoint(x, y);
    }
  }

  // Inflate obstacle borders
  costmap_.inflateObstacles();

  // Populate and publish the OccupancyGrid
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = this->get_clock()->now();
  grid_msg.header.frame_id = scan->header.frame_id; // "robot/laser_link" or "laser_frame"

  grid_msg.info.resolution = robot::CostmapCore::RESOLUTION;
  grid_msg.info.width = robot::CostmapCore::WIDTH;
  grid_msg.info.height = robot::CostmapCore::HEIGHT;
  grid_msg.info.origin.position.x = robot::CostmapCore::ORIGIN_X;
  grid_msg.info.origin.position.y = robot::CostmapCore::ORIGIN_Y;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  grid_msg.data = costmap_.getGridData();

  costmap_pub_->publish(grid_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() 
  : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

  // Subscribers
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Timer: Checks once a second whether the global map should be published
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

  RCLCPP_INFO(this->get_logger(), "Map Memory Node has initialized.");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  current_yaw_ = extractYaw(msg->pose.pose.orientation);

  // Calculate distance traveled since last update
  double dist = std::sqrt(std::pow(current_x_ - last_x_, 2) + std::pow(current_y_ - last_y_, 2));
  if (dist >= distance_threshold_) {
    last_x_ = current_x_;
    last_y_ = current_y_;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {
    // Fuse local costmap snapshot into global map canvas
    map_memory_.updateMap(latest_costmap_, current_x_, current_y_, current_yaw_);

    // Build OccupancyGrid message
    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.stamp = this->get_clock()->now();
    map_msg.header.frame_id = "sim_world";

    map_msg.info.resolution = robot::MapMemoryCore::RESOLUTION;
    map_msg.info.width = robot::MapMemoryCore::WIDTH;
    map_msg.info.height = robot::MapMemoryCore::HEIGHT;
    map_msg.info.origin.position.x = robot::MapMemoryCore::ORIGIN_X;
    map_msg.info.origin.position.y = robot::MapMemoryCore::ORIGIN_Y;
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;

    map_msg.data = map_memory_.getMapData();

    map_pub_->publish(map_msg);
    should_update_map_ = false;
  }
}

double MapMemoryNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  // Convert quaternion to yaw angle around the Z axis
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
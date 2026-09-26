#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {
  // Initialize entire global map with 0 (unexplored/free space)
  global_grid_.resize(WIDTH * HEIGHT, 0);
}

void MapMemoryCore::updateMap(const nav_msgs::msg::OccupancyGrid &costmap, double robot_x, double robot_y, double robot_yaw) {
  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);

  int costmap_w = costmap.info.width;
  int costmap_h = costmap.info.height;
  double costmap_res = costmap.info.resolution;
  double costmap_ox = costmap.info.origin.position.x;
  double costmap_oy = costmap.info.origin.position.y;

  // Iterate over every cell in the local costmap
  for (int cy = 0; cy < costmap_h; ++cy) {
    for (int cx = 0; cx < costmap_w; ++cx) {
      int c_idx = cy * costmap_w + cx;
      int8_t cost = costmap.data[c_idx];

      // Ignore zero or untouched cells so we do not clear out past obstacles
      if (cost <= 0) {
        continue;
      }

      // Convert local costmap grid coordinate to robot-relative meters
      double lx = costmap_ox + (cx + 0.5) * costmap_res;
      double ly = costmap_oy + (cy + 0.5) * costmap_res;

      // Rotate and translate from robot frame into the global world frame
      double gx = robot_x + (lx * cos_yaw - ly * sin_yaw);
      double gy = robot_y + (lx * sin_yaw + ly * cos_yaw);

      // Check bounds within global map
      if (gx < ORIGIN_X || gx >= ORIGIN_X + (WIDTH * RESOLUTION) ||
          gy < ORIGIN_Y || gy >= ORIGIN_Y + (HEIGHT * RESOLUTION)) {
        continue;
      }

      // world coordinate into global map index
      int mx = static_cast<int>((gx - ORIGIN_X) / RESOLUTION);
      int my = static_cast<int>((gy - ORIGIN_Y) / RESOLUTION);

      if (mx >= 0 && mx < WIDTH && my >= 0 && my < HEIGHT) {
        int m_idx = my * WIDTH + mx;
        // Retain the highest cost recorded for that cell
        if (cost > global_grid_[m_idx]) {
          global_grid_[m_idx] = cost;
        }
      }
    }
  }
}

const std::vector<int8_t>& MapMemoryCore::getMapData() const {
  return global_grid_;
}

}
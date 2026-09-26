#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Global map configuration (30m x 30m arena, centered)
    static constexpr double RESOLUTION = 0.1; // 0.1m per cell
    static constexpr int WIDTH = 300;         // 300 cells = 30m
    static constexpr int HEIGHT = 300;        // 300 cells = 30m
    static constexpr double ORIGIN_X = -15.0; // World coords from -15m to +15m
    static constexpr double ORIGIN_Y = -15.0;

    void updateMap(const nav_msgs::msg::OccupancyGrid &costmap, double robot_x, double robot_y, double robot_yaw);
    const std::vector<int8_t>& getMapData() const;

  private:
    rclcpp::Logger logger_;
    std::vector<int8_t> global_grid_;
};

}

#endif
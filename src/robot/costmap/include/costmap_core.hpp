#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    // Grid configuration constants
    static constexpr double RESOLUTION = 0.1; // 0.1m per cell (10cm)
    static constexpr int WIDTH = 100;         // 100 cells = 10m wide
    static constexpr int HEIGHT = 100;        // 100 cells = 10m high
    static constexpr double ORIGIN_X = -5.0;  // Centered at robot (-5m to +5m)
    static constexpr double ORIGIN_Y = -5.0;

    // Inflation settings
    static constexpr double INFLATION_RADIUS = 0.8; // 0.8m safety buffer
    static constexpr int8_t MAX_COST = 100;

    void resetGrid();
    void addLaserPoint(double x, double y);
    void inflateObstacles();
    const std::vector<int8_t>& getGridData() const;

  private:
    bool worldToGrid(double wx, double wy, int &gx, int &gy) const;

    rclcpp::Logger logger_;
    std::vector<int8_t> grid_;
    std::vector<std::pair<int, int>> obstacle_cells_;
};

}  

#endif  
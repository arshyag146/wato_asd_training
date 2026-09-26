#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace robot
{

struct PointInt {
  int x;
  int y;

  bool operator==(const PointInt &other) const {
    return x == other.x && y == other.y;
  }
};

struct AStarNode {
  PointInt pos;
  double g_cost;
  double h_cost;
  double f_cost() const { return g_cost + h_cost; }

  bool operator>(const AStarNode &other) const {
    return f_cost() > other.f_cost();
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    std::vector<geometry_msgs::msg::Point> planPath(
      const nav_msgs::msg::OccupancyGrid &map,
      double start_x, double start_y,
      double goal_x, double goal_y);

  private:
    bool worldToGrid(const nav_msgs::msg::OccupancyGrid &map, double wx, double wy, int &gx, int &gy) const;
    void gridToWorld(const nav_msgs::msg::OccupancyGrid &map, int gx, int gy, double &wx, double &wy) const;
    double heuristic(int x1, int y1, int x2, int y2) const;

    rclcpp::Logger logger_;
    static constexpr int8_t OCCUPIED_THRESHOLD = 50;
};

}

#endif
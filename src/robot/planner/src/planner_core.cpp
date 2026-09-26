#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid &map, double wx, double wy, int &gx, int &gy) const {
  double ox = map.info.origin.position.x;
  double oy = map.info.origin.position.y;
  double res = map.info.resolution;

  if (wx < ox || wy < oy) return false;

  gx = static_cast<int>((wx - ox) / res);
  gy = static_cast<int>((wy - oy) / res);

  return (gx >= 0 && gx < static_cast<int>(map.info.width) &&
          gy >= 0 && gy < static_cast<int>(map.info.height));
}

void PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid &map, int gx, int gy, double &wx, double &wy) const {
  wx = map.info.origin.position.x + (gx + 0.5) * map.info.resolution;
  wy = map.info.origin.position.y + (gy + 0.5) * map.info.resolution;
}

double PlannerCore::heuristic(int x1, int y1, int x2, int y2) const {
  return std::hypot(x1 - x2, y1 - y2);
}

std::vector<geometry_msgs::msg::Point> PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid &map,
  double start_x, double start_y,
  double goal_x, double goal_y) 
{
  std::vector<geometry_msgs::msg::Point> waypoints;

  int sx, sy, gx, gy;
  if (!worldToGrid(map, start_x, start_y, sx, sy) || !worldToGrid(map, goal_x, goal_y, gx, gy)) {
    RCLCPP_WARN(logger_, "Start or Goal outside map boundaries.");
    return waypoints;
  }

  int width = map.info.width;
  int height = map.info.height;

  // Check if goal itself is inside an occupied/inflated cell
  if (map.data[gy * width + gx] >= OCCUPIED_THRESHOLD) {
    RCLCPP_WARN(logger_, "Requested goal is inside an obstacle.");
    return waypoints;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
  std::vector<double> g_score(width * height, std::numeric_limits<double>::infinity());
  std::vector<int> parent(width * height, -1);
  std::vector<bool> closed(width * height, false);

  int start_idx = sy * width + sx;
  int goal_idx = gy * width + gx;

  g_score[start_idx] = 0.0;
  open_set.push({ {sx, sy}, 0.0, heuristic(sx, sy, gx, gy) });

  // 8-connected grid motion
  const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  const double step_cost[] = {std::sqrt(2), 1.0, std::sqrt(2), 1.0, 1.0, std::sqrt(2), 1.0, std::sqrt(2)};

  bool goal_reached = false;

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    int cur_idx = current.pos.y * width + current.pos.x;
    if (closed[cur_idx]) continue;
    closed[cur_idx] = true;

    if (current.pos.x == gx && current.pos.y == gy) {
      goal_reached = true;
      break;
    }

    for (int i = 0; i < 8; ++i) {
      int nx = current.pos.x + dx[i];
      int ny = current.pos.y + dy[i];

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

      int next_idx = ny * width + nx;
      if (closed[next_idx]) continue;

      int8_t cell_cost = map.data[next_idx];
      if (cell_cost >= OCCUPIED_THRESHOLD) continue;

      // Add small penalty for higher costmap values to steer away from edges
      double penalty = (cell_cost > 0) ? (cell_cost / 10.0) : 0.0;
      double tentative_g = g_score[cur_idx] + step_cost[i] + penalty;

      if (tentative_g < g_score[next_idx]) {
        g_score[next_idx] = tentative_g;
        parent[next_idx] = cur_idx;
        open_set.push({ {nx, ny}, tentative_g, heuristic(nx, ny, gx, gy) });
      }
    }
  }

  if (!goal_reached) {
    RCLCPP_WARN(logger_, "No path could be found to goal.");
    return waypoints;
  }

  // Backtrack path
  int curr = goal_idx;
  std::vector<PointInt> path_cells;
  while (curr != -1) {
    path_cells.push_back({curr % width, curr / width});
    curr = parent[curr];
  }
  std::reverse(path_cells.begin(), path_cells.end());

  for (const auto &cell : path_cells) {
    geometry_msgs::msg::Point pt;
    gridToWorld(map, cell.x, cell.y, pt.x, pt.y);
    pt.z = 0.0;
    waypoints.push_back(pt);
  }

  return waypoints;
}

}
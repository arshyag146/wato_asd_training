#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
  grid_.resize(WIDTH * HEIGHT, 0);
}

void CostmapCore::resetGrid() {
  std::fill(grid_.begin(), grid_.end(), 0);
  obstacle_cells_.clear();
}

bool CostmapCore::worldToGrid(double wx, double wy, int &gx, int &gy) const {
  if (wx < ORIGIN_X || wx >= ORIGIN_X + (WIDTH * RESOLUTION) ||
      wy < ORIGIN_Y || wy >= ORIGIN_Y + (HEIGHT * RESOLUTION)) {
    return false;
  }
  gx = static_cast<int>((wx - ORIGIN_X) / RESOLUTION);
  gy = static_cast<int>((wy - ORIGIN_Y) / RESOLUTION);
  return (gx >= 0 && gx < WIDTH && gy >= 0 && gy < HEIGHT);
}

void CostmapCore::addLaserPoint(double x, double y) {
  int gx, gy;
  if (worldToGrid(x, y, gx, gy)) {
    grid_[gy * WIDTH + gx] = MAX_COST;
    obstacle_cells_.push_back({gx, gy});
  }
}

void CostmapCore::inflateObstacles() {
  int cell_radius = static_cast<int>(std::ceil(INFLATION_RADIUS / RESOLUTION));

  for (const auto &cell : obstacle_cells_) {
    int ox = cell.first;
    int oy = cell.second;

    for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        int nx = ox + dx;
        int ny = oy + dy;

        if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT) {
          double dist = std::sqrt(dx * dx + dy * dy) * RESOLUTION;
          if (dist <= INFLATION_RADIUS) {
            int8_t cost = static_cast<int8_t>(MAX_COST * (1.0 - (dist / INFLATION_RADIUS)));
            int idx = ny * WIDTH + nx;
            if (cost > grid_[idx]) {
              grid_[idx] = cost;
            }
          }
        }
      }
    }
  }
}

const std::vector<int8_t>& CostmapCore::getGridData() const {
  return grid_;
}

}
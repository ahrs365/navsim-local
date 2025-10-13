#include "core/planning_context.hpp"
#include <cmath>

namespace navsim {
namespace planning {

// ========== OccupancyGrid 工具函数 ==========

bool OccupancyGrid::isOccupied(int x, int y, uint8_t threshold) const {
  if (x < 0 || x >= config.width || y < 0 || y >= config.height) {
    return true;  // 边界外视为占据
  }

  int index = y * config.width + x;
  return data[index] >= threshold;
}

Point2d OccupancyGrid::cellToWorld(int x, int y) const {
  return {
    config.origin.x + x * config.resolution,
    config.origin.y + y * config.resolution
  };
}

std::pair<int, int> OccupancyGrid::worldToCell(const Point2d& point) const {
  int x = static_cast<int>((point.x - config.origin.x) / config.resolution);
  int y = static_cast<int>((point.y - config.origin.y) / config.resolution);
  return {x, y};
}

} // namespace planning
} // namespace navsim
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "../plugins/perception/esdf_builder/include/esdf_map.hpp"

using namespace navsim::perception;

// 打印 ESDF 地图（ASCII 可视化）
void printESDFMap(const ESDFMap& esdf_map, int width, int height) {
  std::cout << "\n========== ESDF Map Visualization ==========\n";
  std::cout << "Legend: . = free (>2m), o = medium (1-2m), O = near (0.5-1m), X = very near (<0.5m), # = obstacle\n\n";
  
  for (int y = height - 1; y >= 0; --y) {
    std::cout << std::setw(3) << y << " | ";
    for (int x = 0; x < width; ++x) {
      Eigen::Vector2i idx(x, y);
      double dist = esdf_map.getDistance(idx);
      
      char symbol;
      if (esdf_map.isOccupied(idx)) {
        symbol = '#';
      } else if (dist < 0.5) {
        symbol = 'X';
      } else if (dist < 1.0) {
        symbol = 'O';
      } else if (dist < 2.0) {
        symbol = 'o';
      } else {
        symbol = '.';
      }
      std::cout << symbol << ' ';
    }
    std::cout << '\n';
  }
  
  std::cout << "    +";
  for (int x = 0; x < width; ++x) {
    std::cout << "--";
  }
  std::cout << "\n      ";
  for (int x = 0; x < width; ++x) {
    if (x % 5 == 0) {
      std::cout << std::setw(2) << x;
    } else {
      std::cout << "  ";
    }
  }
  std::cout << "\n";
}

// 打印距离值（数值）
void printDistanceValues(const ESDFMap& esdf_map, int width, int height) {
  std::cout << "\n========== Distance Values (meters) ==========\n\n";
  
  for (int y = height - 1; y >= 0; --y) {
    std::cout << std::setw(3) << y << " | ";
    for (int x = 0; x < width; ++x) {
      Eigen::Vector2i idx(x, y);
      double dist = esdf_map.getDistance(idx);
      
      if (esdf_map.isOccupied(idx)) {
        std::cout << " #### ";
      } else {
        std::cout << std::setw(5) << std::fixed << std::setprecision(2) << dist << " ";
      }
    }
    std::cout << '\n';
  }
  
  std::cout << "    +";
  for (int x = 0; x < width; ++x) {
    std::cout << "------";
  }
  std::cout << "\n      ";
  for (int x = 0; x < width; ++x) {
    if (x % 5 == 0) {
      std::cout << std::setw(6) << x;
    } else {
      std::cout << "      ";
    }
  }
  std::cout << "\n";
}

// 测试用例 1：单个障碍物
void test_single_obstacle() {
  std::cout << "\n" << std::string(60, '=') << "\n";
  std::cout << "TEST 1: Single Obstacle at Center\n";
  std::cout << std::string(60, '=') << "\n";
  
  // 创建 20x20 的地图，分辨率 0.1m
  ESDFMap::Config config;
  config.resolution = 0.1;
  config.map_width = 2.0;   // 20 * 0.1 = 2.0m
  config.map_height = 2.0;
  config.max_distance = 5.0;
  
  ESDFMap esdf_map;
  esdf_map.initialize(config);
  
  // 创建占据栅格：中心有一个障碍物
  int width = 20;
  int height = 20;
  std::vector<uint8_t> occupancy_grid(width * height, 0);
  
  // 在中心 (10, 10) 放置障碍物
  occupancy_grid[10 * width + 10] = 100;
  
  // 构建 ESDF
  Eigen::Vector2d origin(0.0, 0.0);
  esdf_map.buildFromOccupancyGrid(occupancy_grid, origin);
  esdf_map.computeESDF();
  
  // 打印可视化
  printESDFMap(esdf_map, width, height);
  
  // 验证距离
  std::cout << "\n========== Distance Verification ==========\n";
  
  // 障碍物位置
  Eigen::Vector2i obstacle(10, 10);
  std::cout << "Obstacle at (" << obstacle.x() << ", " << obstacle.y() << "): "
            << "distance = " << esdf_map.getDistance(obstacle) << " (should be 0.0)\n";
  
  // 相邻格子（1 格 = 0.1m）
  Eigen::Vector2i neighbor1(11, 10);
  double dist1 = esdf_map.getDistance(neighbor1);
  std::cout << "Neighbor at (" << neighbor1.x() << ", " << neighbor1.y() << "): "
            << "distance = " << dist1 << " (should be ~0.1m)\n";
  
  // 对角相邻（sqrt(2) * 0.1 ≈ 0.14m）
  Eigen::Vector2i neighbor2(11, 11);
  double dist2 = esdf_map.getDistance(neighbor2);
  std::cout << "Diagonal neighbor at (" << neighbor2.x() << ", " << neighbor2.y() << "): "
            << "distance = " << dist2 << " (should be ~0.14m)\n";
  
  // 5 格距离（0.5m）
  Eigen::Vector2i far1(15, 10);
  double dist3 = esdf_map.getDistance(far1);
  std::cout << "5 cells away at (" << far1.x() << ", " << far1.y() << "): "
            << "distance = " << dist3 << " (should be ~0.5m)\n";
  
  // 10 格距离（1.0m）
  Eigen::Vector2i far2(20, 10);
  if (far2.x() < width) {
    double dist4 = esdf_map.getDistance(far2);
    std::cout << "10 cells away at (" << far2.x() << ", " << far2.y() << "): "
              << "distance = " << dist4 << " (should be ~1.0m)\n";
  }
  
  // 统计距离分布
  std::cout << "\n========== Distance Distribution ==========\n";
  int count_0_05 = 0, count_05_1 = 0, count_1_2 = 0, count_2_plus = 0;
  double min_dist = 1000.0, max_dist = -1000.0;
  
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      Eigen::Vector2i idx(x, y);
      if (esdf_map.isOccupied(idx)) continue;
      
      double dist = esdf_map.getDistance(idx);
      min_dist = std::min(min_dist, dist);
      max_dist = std::max(max_dist, dist);
      
      if (dist < 0.5) count_0_05++;
      else if (dist < 1.0) count_05_1++;
      else if (dist < 2.0) count_1_2++;
      else count_2_plus++;
    }
  }
  
  std::cout << "Distance < 0.5m:  " << count_0_05 << " cells\n";
  std::cout << "Distance 0.5-1m:  " << count_05_1 << " cells\n";
  std::cout << "Distance 1-2m:    " << count_1_2 << " cells\n";
  std::cout << "Distance > 2m:    " << count_2_plus << " cells\n";
  std::cout << "Min distance:     " << min_dist << "m\n";
  std::cout << "Max distance:     " << max_dist << "m\n";
}

// 测试用例 2：多个障碍物
void test_multiple_obstacles() {
  std::cout << "\n" << std::string(60, '=') << "\n";
  std::cout << "TEST 2: Multiple Obstacles\n";
  std::cout << std::string(60, '=') << "\n";
  
  ESDFMap::Config config;
  config.resolution = 0.2;  // 更大的分辨率，便于观察
  config.map_width = 4.0;   // 20 * 0.2 = 4.0m
  config.map_height = 4.0;
  config.max_distance = 5.0;
  
  ESDFMap esdf_map;
  esdf_map.initialize(config);
  
  int width = 20;
  int height = 20;
  std::vector<uint8_t> occupancy_grid(width * height, 0);
  
  // 放置 3 个障碍物
  occupancy_grid[5 * width + 5] = 100;   // 左下
  occupancy_grid[5 * width + 15] = 100;  // 右下
  occupancy_grid[15 * width + 10] = 100; // 上中
  
  Eigen::Vector2d origin(0.0, 0.0);
  esdf_map.buildFromOccupancyGrid(occupancy_grid, origin);
  esdf_map.computeESDF();
  
  printESDFMap(esdf_map, width, height);
  printDistanceValues(esdf_map, width, height);
  
  // 验证中心点（应该距离最近的障碍物）
  Eigen::Vector2i center(10, 10);
  double center_dist = esdf_map.getDistance(center);
  std::cout << "\nCenter point (" << center.x() << ", " << center.y() << "): "
            << "distance = " << center_dist << "m\n";
  
  // 计算到最近障碍物的理论距离
  double dist_to_top = std::sqrt(std::pow((10 - 10) * 0.2, 2) + std::pow((10 - 15) * 0.2, 2));
  std::cout << "Theoretical distance to top obstacle: " << dist_to_top << "m\n";
}

// 测试用例 3：验证与原始实现的一致性
void test_consistency_check() {
  std::cout << "\n" << std::string(60, '=') << "\n";
  std::cout << "TEST 3: Consistency Check\n";
  std::cout << std::string(60, '=') << "\n";
  
  ESDFMap::Config config;
  config.resolution = 0.1;
  config.map_width = 3.0;
  config.map_height = 3.0;
  config.max_distance = 5.0;
  
  ESDFMap esdf_map;
  esdf_map.initialize(config);
  
  int width = 30;
  int height = 30;
  std::vector<uint8_t> occupancy_grid(width * height, 0);
  
  // 创建一个 5x5 的障碍物块
  for (int y = 12; y <= 17; ++y) {
    for (int x = 12; x <= 17; ++x) {
      occupancy_grid[y * width + x] = 100;
    }
  }
  
  Eigen::Vector2d origin(0.0, 0.0);
  esdf_map.buildFromOccupancyGrid(occupancy_grid, origin);
  esdf_map.computeESDF();
  
  printDistanceValues(esdf_map, width, height);
  
  // 检查关键点
  std::cout << "\n========== Key Points Check ==========\n";
  
  // 障碍物内部
  Eigen::Vector2i inside(15, 15);
  std::cout << "Inside obstacle (" << inside.x() << ", " << inside.y() << "): "
            << esdf_map.getDistance(inside) << "m (should be negative)\n";
  
  // 障碍物边界
  Eigen::Vector2i edge(11, 15);
  std::cout << "Edge of obstacle (" << edge.x() << ", " << edge.y() << "): "
            << esdf_map.getDistance(edge) << "m (should be ~0.1m)\n";
  
  // 远离障碍物
  Eigen::Vector2i far(0, 0);
  std::cout << "Far from obstacle (" << far.x() << ", " << far.y() << "): "
            << esdf_map.getDistance(far) << "m (should be >1m)\n";
}

int main() {
  std::cout << "\n";
  std::cout << "╔════════════════════════════════════════════════════════════╗\n";
  std::cout << "║          ESDF Map Calculation Test Suite                  ║\n";
  std::cout << "╚════════════════════════════════════════════════════════════╝\n";
  
  test_single_obstacle();
  test_multiple_obstacles();
  test_consistency_check();
  
  std::cout << "\n";
  std::cout << "╔════════════════════════════════════════════════════════════╗\n";
  std::cout << "║                    Tests Completed                         ║\n";
  std::cout << "╚════════════════════════════════════════════════════════════╝\n";
  std::cout << "\n";
  
  return 0;
}


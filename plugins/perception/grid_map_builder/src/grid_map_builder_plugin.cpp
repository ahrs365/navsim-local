#include "grid_map_builder_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <cmath>
#include <chrono>
#include <iostream>

namespace navsim {
namespace plugins {
namespace perception {

// ========== Config ==========

GridMapBuilderPlugin::Config GridMapBuilderPlugin::Config::fromJson(
    const nlohmann::json& json) {
  Config config;
  
  if (json.contains("resolution")) {
    config.resolution = json["resolution"].get<double>();
  }
  if (json.contains("map_width")) {
    config.map_width = json["map_width"].get<double>();
  }
  if (json.contains("map_height")) {
    config.map_height = json["map_height"].get<double>();
  }
  if (json.contains("obstacle_cost")) {
    config.obstacle_cost = json["obstacle_cost"].get<uint8_t>();
  }
  if (json.contains("inflation_radius")) {
    config.inflation_radius = json["inflation_radius"].get<double>();
  }
  
  return config;
}

// ========== GridMapBuilderPlugin ==========

GridMapBuilderPlugin::GridMapBuilderPlugin()
    : config_(Config{}) {
}

GridMapBuilderPlugin::GridMapBuilderPlugin(const Config& config)
    : config_(config) {
}

plugin::PerceptionPluginMetadata GridMapBuilderPlugin::getMetadata() const {
  plugin::PerceptionPluginMetadata metadata;
  metadata.name = "GridMapBuilder";
  metadata.version = "1.0.0";
  metadata.description = "Build occupancy grid map from BEV obstacles";
  metadata.author = "NavSim Team";
  metadata.requires_raw_data = false;
  metadata.output_data_types = {"occupancy_grid"};
  return metadata;
}

bool GridMapBuilderPlugin::initialize(const nlohmann::json& config) {
  try {
    config_ = Config::fromJson(config);
    
    std::cout << "[GridMapBuilderPlugin] Initialized with config:" << std::endl;
    std::cout << "  - resolution: " << config_.resolution << " m/cell" << std::endl;
    std::cout << "  - map_size: " << config_.map_width << "x" << config_.map_height << " m" << std::endl;
    std::cout << "  - inflation_radius: " << config_.inflation_radius << " m" << std::endl;
    
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[GridMapBuilderPlugin] Failed to initialize: " << e.what() << std::endl;
    return false;
  }
}

bool GridMapBuilderPlugin::process(const plugin::PerceptionInput& input,
                                   planning::PlanningContext& context) {
  auto start_time = std::chrono::steady_clock::now();
  
  // 创建栅格地图
  auto grid = std::make_unique<planning::OccupancyGrid>();
  
  // 配置地图参数
  grid->config.resolution = config_.resolution;
  grid->config.width = static_cast<int>(config_.map_width / config_.resolution);
  grid->config.height = static_cast<int>(config_.map_height / config_.resolution);
  
  // 以自车为中心的地图
  grid->config.origin.x = input.ego.pose.x - config_.map_width / 2.0;
  grid->config.origin.y = input.ego.pose.y - config_.map_height / 2.0;
  
  // 初始化栅格数据
  grid->data.resize(grid->config.width * grid->config.height, 0);
  
  // 添加 BEV 障碍物
  addBEVObstacles(input.bev_obstacles, *grid);
  
  // 膨胀处理
  inflateObstacles(*grid);
  
  // 保存到上下文
  context.occupancy_grid = std::move(grid);
  
  // 更新统计信息
  auto end_time = std::chrono::steady_clock::now();
  double elapsed_ms = 
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  stats_.total_processed++;
  stats_.average_time_ms = 
      (stats_.average_time_ms * (stats_.total_processed - 1) + elapsed_ms) / 
      stats_.total_processed;
  
  return true;
}

void GridMapBuilderPlugin::reset() {
  stats_ = Statistics();
}

nlohmann::json GridMapBuilderPlugin::getStatistics() const {
  nlohmann::json stats;
  stats["total_processed"] = stats_.total_processed;
  stats["total_obstacles"] = stats_.total_obstacles;
  stats["average_time_ms"] = stats_.average_time_ms;
  return stats;
}

bool GridMapBuilderPlugin::isAvailable() const {
  return true;
}

// ========== Private Methods ==========

void GridMapBuilderPlugin::addBEVObstacles(const planning::BEVObstacles& bev_obstacles,
                                          planning::OccupancyGrid& grid) {
  // 添加圆形障碍物
  for (const auto& circle : bev_obstacles.circles) {
    addCircleObstacle(circle, grid);
    stats_.total_obstacles++;
  }
  
  // 添加矩形障碍物
  for (const auto& rect : bev_obstacles.rectangles) {
    addRectangleObstacle(rect, grid);
    stats_.total_obstacles++;
  }
  
  // 添加多边形障碍物
  for (const auto& polygon : bev_obstacles.polygons) {
    addPolygonObstacle(polygon, grid);
    stats_.total_obstacles++;
  }
}

void GridMapBuilderPlugin::addCircleObstacle(
    const planning::BEVObstacles::Circle& circle,
    planning::OccupancyGrid& grid) {
  // 计算圆形在栅格中的范围
  int center_x, center_y;
  if (!worldToGrid(circle.center.x, circle.center.y, grid, center_x, center_y)) {
    return;  // 圆心不在地图范围内
  }
  
  int radius_cells = static_cast<int>(std::ceil(circle.radius / grid.config.resolution));
  
  // 遍历圆形范围内的所有栅格
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      // 检查是否在圆内
      if (dx * dx + dy * dy <= radius_cells * radius_cells) {
        setGridCell(center_x + dx, center_y + dy, config_.obstacle_cost, grid);
      }
    }
  }
}

void GridMapBuilderPlugin::addRectangleObstacle(
    const planning::BEVObstacles::Rectangle& rect,
    planning::OccupancyGrid& grid) {
  // 简化实现：将矩形转换为圆形（使用对角线的一半作为半径）
  double radius = std::sqrt(rect.width * rect.width + rect.height * rect.height) / 2.0;
  
  planning::BEVObstacles::Circle circle;
  circle.center.x = rect.pose.x;
  circle.center.y = rect.pose.y;
  circle.radius = radius;
  circle.confidence = rect.confidence;
  
  addCircleObstacle(circle, grid);
}

void GridMapBuilderPlugin::addPolygonObstacle(
    const planning::BEVObstacles::Polygon& polygon,
    planning::OccupancyGrid& grid) {
  if (polygon.vertices.empty()) return;
  
  // 简化实现：计算多边形的包围圆
  double center_x = 0.0, center_y = 0.0;
  for (const auto& vertex : polygon.vertices) {
    center_x += vertex.x;
    center_y += vertex.y;
  }
  center_x /= polygon.vertices.size();
  center_y /= polygon.vertices.size();
  
  // 计算最大半径
  double max_radius = 0.0;
  for (const auto& vertex : polygon.vertices) {
    double dx = vertex.x - center_x;
    double dy = vertex.y - center_y;
    double dist = std::sqrt(dx * dx + dy * dy);
    max_radius = std::max(max_radius, dist);
  }
  
  planning::BEVObstacles::Circle circle;
  circle.center.x = center_x;
  circle.center.y = center_y;
  circle.radius = max_radius;
  circle.confidence = polygon.confidence;
  
  addCircleObstacle(circle, grid);
}

void GridMapBuilderPlugin::inflateObstacles(planning::OccupancyGrid& grid) {
  if (config_.inflation_radius <= 0.0) return;
  
  int inflation_cells = static_cast<int>(std::ceil(config_.inflation_radius / grid.config.resolution));
  std::vector<uint8_t> inflated_data = grid.data;
  
  for (int y = 0; y < grid.config.height; ++y) {
    for (int x = 0; x < grid.config.width; ++x) {
      int index = y * grid.config.width + x;
      if (grid.data[index] >= config_.obstacle_cost) {
        // 膨胀障碍物
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            if (dx * dx + dy * dy <= inflation_cells * inflation_cells) {
              int nx = x + dx;
              int ny = y + dy;
              if (nx >= 0 && nx < grid.config.width && ny >= 0 && ny < grid.config.height) {
                int nindex = ny * grid.config.width + nx;
                inflated_data[nindex] = std::max(inflated_data[nindex],
                                                static_cast<uint8_t>(config_.obstacle_cost / 2));
              }
            }
          }
        }
      }
    }
  }
  
  grid.data = std::move(inflated_data);
}

bool GridMapBuilderPlugin::worldToGrid(double world_x, double world_y,
                                      const planning::OccupancyGrid& grid,
                                      int& grid_x, int& grid_y) const {
  grid_x = static_cast<int>((world_x - grid.config.origin.x) / grid.config.resolution);
  grid_y = static_cast<int>((world_y - grid.config.origin.y) / grid.config.resolution);
  
  return (grid_x >= 0 && grid_x < grid.config.width &&
          grid_y >= 0 && grid_y < grid.config.height);
}

void GridMapBuilderPlugin::setGridCell(int grid_x, int grid_y, uint8_t value,
                                      planning::OccupancyGrid& grid) {
  if (grid_x >= 0 && grid_x < grid.config.width &&
      grid_y >= 0 && grid_y < grid.config.height) {
    int index = grid_y * grid.config.width + grid_x;
    grid.data[index] = std::max(grid.data[index], value);
  }
}

} // namespace perception
} // namespace plugins
} // namespace navsim

// 注册插件
namespace {
static navsim::plugin::PerceptionPluginRegistrar<navsim::plugins::perception::GridMapBuilderPlugin>
    grid_map_builder_registrar("GridMapBuilder");
}


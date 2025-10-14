#include "plugin/preprocessing/preprocessing.hpp"
#include <cmath>
#include <iostream>

namespace navsim {
namespace perception {

std::unique_ptr<planning::BEVObstacles> BEVExtractor::extract(
    const proto::WorldTick& world_tick) {
  auto obstacles = std::make_unique<planning::BEVObstacles>();

  // 🔍 调试日志：输入数据检查
  std::cout << "[BEVExtractor] ========== Extract called ==========" << std::endl;
  std::cout << "[BEVExtractor] WorldTick tick_id: " << world_tick.tick_id() << std::endl;
  std::cout << "[BEVExtractor] Has static_map: " << world_tick.has_static_map() << std::endl;
  std::cout << "[BEVExtractor] Dynamic obstacles count: " << world_tick.dynamic_obstacles_size() << std::endl;

  if (world_tick.has_static_map()) {
    const auto& static_map = world_tick.static_map();
    std::cout << "[BEVExtractor] StaticMap circles: " << static_map.circles_size() << std::endl;
    std::cout << "[BEVExtractor] StaticMap polygons: " << static_map.polygons_size() << std::endl;
  }

  // 提取静态障碍物
  extractStaticObstacles(world_tick, *obstacles);

  // 🔧 修复问题2：不要在这里提取动态障碍物！
  // 动态障碍物应该由 DynamicObstaclePredictor 处理
  // extractDynamicObstacles(world_tick, *obstacles);  // ← 删除此调用

  total_extractions_++;

  // 🔍 调试日志：输出结果
  std::cout << "[BEVExtractor] ========== Extract result ==========" << std::endl;
  std::cout << "[BEVExtractor] Extracted circles: " << obstacles->circles.size() << std::endl;
  std::cout << "[BEVExtractor] Extracted rectangles: " << obstacles->rectangles.size() << std::endl;
  std::cout << "[BEVExtractor] Extracted polygons: " << obstacles->polygons.size() << std::endl;
  std::cout << "[BEVExtractor] ======================================" << std::endl;

  return obstacles;
}

void BEVExtractor::extractStaticObstacles(const proto::WorldTick& world_tick,
                                         planning::BEVObstacles& obstacles) {
  // 更新静态地图缓存（如果这个tick包含静态地图）
  if (world_tick.has_static_map()) {
    cached_static_map_ = world_tick.static_map();
    has_cached_static_map_ = true;
    std::cout << "[BEVExtractor] Updated static map cache" << std::endl;
  }

  // 如果没有缓存的静态地图，则跳过
  if (!has_cached_static_map_) {
    std::cout << "[BEVExtractor] No cached static map, skipping static obstacles" << std::endl;
    return;
  }

  const auto& static_map = cached_static_map_;
  const auto& ego_pose = world_tick.ego().pose();

  std::cout << "[BEVExtractor] Processing static obstacles..." << std::endl;
  std::cout << "[BEVExtractor]   Ego position: (" << ego_pose.x() << ", " << ego_pose.y() << ")" << std::endl;
  std::cout << "[BEVExtractor]   Detection range: " << config_.detection_range << " m" << std::endl;
  std::cout << "[BEVExtractor]   Cached circles: " << static_map.circles_size() << std::endl;
  std::cout << "[BEVExtractor]   Cached polygons: " << static_map.polygons_size() << std::endl;

  // 处理静态圆形障碍物
  int circles_in_range = 0;
  for (const auto& circle : static_map.circles()) {
    // 检查距离是否在检测范围内
    double dx = circle.x() - ego_pose.x();
    double dy = circle.y() - ego_pose.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= config_.detection_range) {
      planning::BEVObstacles::Circle circle_obs;
      circle_obs.center.x = circle.x();
      circle_obs.center.y = circle.y();
      circle_obs.radius = circle.r();
      circle_obs.confidence = 1.0;  // 静态障碍物置信度为1.0

      obstacles.circles.push_back(circle_obs);
      circles_in_range++;
    }
  }
  std::cout << "[BEVExtractor]   Static circles in range: " << circles_in_range << std::endl;

  // 处理静态多边形障碍物
  int polygons_in_range = 0;
  for (const auto& polygon : static_map.polygons()) {
    if (polygon.points().empty()) continue;

    // 检查多边形中心点是否在检测范围内
    double center_x = 0.0, center_y = 0.0;
    for (const auto& point : polygon.points()) {
      center_x += point.x();
      center_y += point.y();
    }
    center_x /= polygon.points().size();
    center_y /= polygon.points().size();

    double dx = center_x - ego_pose.x();
    double dy = center_y - ego_pose.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= config_.detection_range) {
      planning::BEVObstacles::Polygon poly_obs;
      poly_obs.confidence = 1.0;  // 静态障碍物置信度为1.0

      for (const auto& point : polygon.points()) {
        planning::Point2d vertex;
        vertex.x = point.x();
        vertex.y = point.y();
        poly_obs.vertices.push_back(vertex);
      }

      obstacles.polygons.push_back(poly_obs);
      polygons_in_range++;
    }
  }
  std::cout << "[BEVExtractor]   Static polygons in range: " << polygons_in_range << std::endl;
}

void BEVExtractor::extractDynamicObstacles(const proto::WorldTick& world_tick,
                                          planning::BEVObstacles& obstacles) {
  const auto& ego_pose = world_tick.ego().pose();

  // 处理动态障碍物
  for (const auto& dyn_obs : world_tick.dynamic_obstacles()) {
    // 检查距离是否在检测范围内
    double dx = dyn_obs.pose().x() - ego_pose.x();
    double dy = dyn_obs.pose().y() - ego_pose.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= config_.detection_range) {
      // 根据形状类型处理
      if (dyn_obs.shape().has_circle()) {
        const auto& circle = dyn_obs.shape().circle();
        planning::BEVObstacles::Circle circle_obs;
        circle_obs.center.x = dyn_obs.pose().x();
        circle_obs.center.y = dyn_obs.pose().y();
        circle_obs.radius = circle.r();
        circle_obs.confidence = 0.9;  // 动态障碍物置信度稍低

        obstacles.circles.push_back(circle_obs);
      } else if (dyn_obs.shape().has_rectangle()) {
        const auto& rect = dyn_obs.shape().rectangle();
        planning::BEVObstacles::Rectangle rect_obs;
        rect_obs.pose.x = dyn_obs.pose().x();
        rect_obs.pose.y = dyn_obs.pose().y();
        rect_obs.pose.yaw = dyn_obs.pose().yaw();
        rect_obs.width = rect.w();
        rect_obs.height = rect.h();
        rect_obs.confidence = 0.9;  // 动态障碍物置信度稍低

        obstacles.rectangles.push_back(rect_obs);
      }
    }
  }
}

void BEVExtractor::reset() {
  total_extractions_ = 0;
  has_cached_static_map_ = false;
}

} // namespace perception
} // namespace navsim


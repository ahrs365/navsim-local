#include "astar_planner_plugin.hpp"
#include "plugin/plugin_registry.hpp"
#include <cmath>
#include <chrono>
#include <iostream>
#include <algorithm>

namespace navsim {
namespace plugins {
namespace planning {

// ========== Config ==========

AStarPlannerPlugin::Config AStarPlannerPlugin::Config::fromJson(
    const nlohmann::json& json) {
  Config config;
  
  if (json.contains("time_step")) {
    config.time_step = json["time_step"].get<double>();
  }
  if (json.contains("heuristic_weight")) {
    config.heuristic_weight = json["heuristic_weight"].get<double>();
  }
  if (json.contains("step_size")) {
    config.step_size = json["step_size"].get<double>();
  }
  if (json.contains("max_iterations")) {
    config.max_iterations = json["max_iterations"].get<int>();
  }
  if (json.contains("goal_tolerance")) {
    config.goal_tolerance = json["goal_tolerance"].get<double>();
  }
  if (json.contains("default_velocity")) {
    config.default_velocity = json["default_velocity"].get<double>();
  }
  
  return config;
}

// ========== AStarPlannerPlugin ==========

AStarPlannerPlugin::AStarPlannerPlugin()
    : config_(Config{}) {
}

AStarPlannerPlugin::AStarPlannerPlugin(const Config& config)
    : config_(config) {
}

plugin::PlannerPluginMetadata AStarPlannerPlugin::getMetadata() const {
  plugin::PlannerPluginMetadata metadata;
  metadata.name = "AStarPlanner";
  metadata.version = "1.0.0";
  metadata.description = "A* search-based path planner";
  metadata.author = "NavSim Team";
  metadata.type = "search";
  metadata.required_perception_data = {"occupancy_grid"};
  metadata.can_be_fallback = false;
  return metadata;
}

bool AStarPlannerPlugin::initialize(const nlohmann::json& config) {
  try {
    config_ = Config::fromJson(config);
    
    std::cout << "[AStarPlanner] Initialized with config:" << std::endl;
    std::cout << "  - heuristic_weight: " << config_.heuristic_weight << std::endl;
    std::cout << "  - step_size: " << config_.step_size << " m" << std::endl;
    std::cout << "  - max_iterations: " << config_.max_iterations << std::endl;
    std::cout << "  - goal_tolerance: " << config_.goal_tolerance << " m" << std::endl;
    
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[AStarPlanner] Failed to initialize: " << e.what() << std::endl;
    return false;
  }
}

bool AStarPlannerPlugin::plan(const navsim::planning::PlanningContext& context,
                              std::chrono::milliseconds deadline,
                              plugin::PlanningResult& result) {
  auto start_time = std::chrono::steady_clock::now();
  
  stats_.total_plans++;
  
  // 检查是否有栅格地图
  if (!context.occupancy_grid) {
    result.success = false;
    result.failure_reason = "No occupancy grid available";
    stats_.failed_no_grid++;
    return false;
  }
  
  // 执行 A* 搜索
  std::vector<navsim::planning::Point2d> path = searchPath(context);
  
  if (path.empty()) {
    result.success = false;
    result.failure_reason = "No path found";
    stats_.failed_no_path++;
    return false;
  }
  
  // 转换为轨迹
  result.trajectory = pathToTrajectory(path, context);
  
  // 设置结果
  result.success = true;
  result.planner_name = "AStarPlanner";
  
  // 计算路径长度
  double path_length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    double dx = path[i].x - path[i-1].x;
    double dy = path[i].y - path[i-1].y;
    path_length += std::sqrt(dx * dx + dy * dy);
  }
  
  result.metadata["path_length"] = path_length;
  result.metadata["num_waypoints"] = static_cast<double>(path.size());
  
  // 计算耗时
  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_ms = 
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  // 更新统计信息
  stats_.successful_plans++;
  stats_.average_time_ms = 
      (stats_.average_time_ms * (stats_.successful_plans - 1) + result.computation_time_ms) / 
      stats_.successful_plans;
  stats_.average_path_length = 
      (stats_.average_path_length * (stats_.successful_plans - 1) + path_length) / 
      stats_.successful_plans;
  
  return true;
}

std::pair<bool, std::string> AStarPlannerPlugin::isAvailable(const navsim::planning::PlanningContext& context) const {
  if (!context.occupancy_grid) {
    return {false, "No occupancy grid available"};
  }
  return {true, "Available"};
}

void AStarPlannerPlugin::reset() {
  stats_ = Statistics();
}

nlohmann::json AStarPlannerPlugin::getStatistics() const {
  nlohmann::json stats;
  stats["total_plans"] = stats_.total_plans;
  stats["successful_plans"] = stats_.successful_plans;
  stats["failed_no_grid"] = stats_.failed_no_grid;
  stats["failed_no_path"] = stats_.failed_no_path;
  stats["success_rate"] = stats_.total_plans > 0 ? 
      static_cast<double>(stats_.successful_plans) / stats_.total_plans : 0.0;
  stats["average_time_ms"] = stats_.average_time_ms;
  stats["average_path_length"] = stats_.average_path_length;
  return stats;
}

// ========== Private Methods ==========

std::vector<navsim::planning::Point2d> AStarPlannerPlugin::searchPath(
    const navsim::planning::PlanningContext& context) {
  const auto& grid = *context.occupancy_grid;
  navsim::planning::Point2d start = {context.ego.pose.x, context.ego.pose.y};
  navsim::planning::Point2d goal = {context.task.goal_pose.x, context.task.goal_pose.y};
  
  // 转换为栅格坐标
  int start_x = static_cast<int>((start.x - grid.config.origin.x) / grid.config.resolution);
  int start_y = static_cast<int>((start.y - grid.config.origin.y) / grid.config.resolution);
  int goal_x = static_cast<int>((goal.x - grid.config.origin.x) / grid.config.resolution);
  int goal_y = static_cast<int>((goal.y - grid.config.origin.y) / grid.config.resolution);
  
  // 检查起点和终点是否有效
  auto isOccupied = [&](int x, int y) {
    if (x < 0 || x >= grid.config.width || y < 0 || y >= grid.config.height) {
      return true;
    }
    int index = y * grid.config.width + x;
    return grid.data[index] > 50;  // 阈值：50
  };
  
  if (isOccupied(start_x, start_y) || isOccupied(goal_x, goal_y)) {
    return {};  // 起点或终点被占据
  }
  
  // A* 搜索数据结构
  auto compare = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
    return a->f_cost > b->f_cost;
  };
  
  std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(compare)> open_set(compare);
  std::unordered_set<int> closed_set;
  
  // 起始节点
  auto start_node = std::make_shared<Node>();
  start_node->position.x = grid.config.origin.x + start_x * grid.config.resolution;
  start_node->position.y = grid.config.origin.y + start_y * grid.config.resolution;
  start_node->g_cost = 0.0;
  start_node->h_cost = heuristic(start_node->position, goal);
  start_node->f_cost = start_node->g_cost + start_node->h_cost;
  
  open_set.push(start_node);
  
  // 搜索方向 (8-连通)
  std::vector<std::pair<int, int>> directions = {
    {-1, -1}, {-1, 0}, {-1, 1},
    {0, -1},           {0, 1},
    {1, -1},  {1, 0},  {1, 1}
  };
  
  int iterations = 0;
  while (!open_set.empty() && iterations < config_.max_iterations) {
    auto current = open_set.top();
    open_set.pop();
    
    int curr_x = static_cast<int>((current->position.x - grid.config.origin.x) / grid.config.resolution);
    int curr_y = static_cast<int>((current->position.y - grid.config.origin.y) / grid.config.resolution);
    int curr_key = curr_y * grid.config.width + curr_x;
    
    if (closed_set.count(curr_key)) continue;
    closed_set.insert(curr_key);
    
    // 检查是否到达目标
    if (heuristic(current->position, goal) < config_.goal_tolerance) {
      // 重构路径
      std::vector<navsim::planning::Point2d> path;
      auto node = current;
      while (node) {
        path.push_back(node->position);
        node = node->parent;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    
    // 扩展邻居
    for (const auto& dir : directions) {
      int new_x = curr_x + dir.first;
      int new_y = curr_y + dir.second;
      
      if (new_x < 0 || new_x >= grid.config.width ||
          new_y < 0 || new_y >= grid.config.height) continue;
      
      if (isOccupied(new_x, new_y)) continue;
      
      int neighbor_key = new_y * grid.config.width + new_x;
      if (closed_set.count(neighbor_key)) continue;
      
      auto neighbor = std::make_shared<Node>();
      neighbor->position.x = grid.config.origin.x + new_x * grid.config.resolution;
      neighbor->position.y = grid.config.origin.y + new_y * grid.config.resolution;
      neighbor->parent = current;
      
      double step_cost = (dir.first == 0 || dir.second == 0) ? grid.config.resolution :
                         grid.config.resolution * std::sqrt(2.0);
      neighbor->g_cost = current->g_cost + step_cost;
      neighbor->h_cost = heuristic(neighbor->position, goal) * config_.heuristic_weight;
      neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
      
      open_set.push(neighbor);
    }
    
    iterations++;
  }
  
  return {};  // 未找到路径
}

double AStarPlannerPlugin::heuristic(const navsim::planning::Point2d& a,
                                    const navsim::planning::Point2d& b) const {
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool AStarPlannerPlugin::isCollisionFree(const navsim::planning::Point2d& point,
                                        const navsim::planning::PlanningContext& context) const {
  if (!context.occupancy_grid) return true;
  
  const auto& grid = *context.occupancy_grid;
  int x = static_cast<int>((point.x - grid.config.origin.x) / grid.config.resolution);
  int y = static_cast<int>((point.y - grid.config.origin.y) / grid.config.resolution);
  
  if (x < 0 || x >= grid.config.width || y < 0 || y >= grid.config.height) {
    return false;
  }
  
  int index = y * grid.config.width + x;
  return grid.data[index] <= 50;  // 阈值：50
}

std::vector<plugin::TrajectoryPoint> AStarPlannerPlugin::pathToTrajectory(
    const std::vector<navsim::planning::Point2d>& path,
    const navsim::planning::PlanningContext& context) const {
  std::vector<plugin::TrajectoryPoint> trajectory;
  trajectory.reserve(path.size());
  
  double total_distance = 0.0;
  
  for (size_t i = 0; i < path.size(); ++i) {
    plugin::TrajectoryPoint point;
    point.pose.x = path[i].x;
    point.pose.y = path[i].y;
    
    // 计算朝向
    if (i + 1 < path.size()) {
      double dx = path[i + 1].x - path[i].x;
      double dy = path[i + 1].y - path[i].y;
      point.pose.yaw = std::atan2(dy, dx);
      
      double segment_distance = std::sqrt(dx * dx + dy * dy);
      total_distance += segment_distance;
      point.twist.vx = config_.default_velocity;
    } else {
      // 最后一个点
      point.pose.yaw = context.task.goal_pose.yaw;
      point.twist.vx = 0.0;
    }
    
    point.twist.vy = 0.0;
    point.twist.omega = 0.0;
    point.acceleration = 0.0;
    point.time_from_start = i * config_.time_step;
    point.path_length = total_distance;
    
    trajectory.push_back(point);
  }
  
  return trajectory;
}

} // namespace planning
} // namespace plugins
} // namespace navsim

// 注册插件
namespace {
static navsim::plugin::PlannerPluginRegistrar<navsim::plugins::planning::AStarPlannerPlugin>
    astar_planner_registrar("AStarPlanner");
}


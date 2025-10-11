#include "planner_interface.hpp"
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_set>

namespace navsim {
namespace planning {

// ========== StraightLinePlanner ==========

StraightLinePlanner::StraightLinePlanner() : config_(Config{}) {}

StraightLinePlanner::StraightLinePlanner(const Config& config)
    : config_(config) {}

bool StraightLinePlanner::plan(const PlanningContext& context,
                              std::chrono::milliseconds deadline,
                              PlanningResult& result) {
  auto start_time = std::chrono::steady_clock::now();

  // 清空结果
  result.trajectory.clear();
  result.status = PlanningResult::Status::SUCCESS;
  result.diagnostics.iterations = 1;
  result.diagnostics.converged = true;

  // 计算到目标的距离和方向
  const auto& ego_pose = context.ego.pose;
  const auto& goal_pose = context.task.goal_pose;

  double dx = goal_pose.x - ego_pose.x;
  double dy = goal_pose.y - ego_pose.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // 检查是否已经到达目标
  if (distance < config_.arrival_tolerance) {
    // 生成停留在当前位置的轨迹
    TrajectoryPoint point;
    point.x = ego_pose.x;
    point.y = ego_pose.y;
    point.yaw = ego_pose.yaw;
    point.v = 0.0;
    point.a = 0.0;
    point.t = 0.0;
    point.s = 0.0;
    result.trajectory.push_back(point);

    result.cost.total = 0.0;
    return true;
  }

  // 计算轨迹方向
  double trajectory_yaw = std::atan2(dy, dx);

  // 计算总时间和步数
  double travel_time = distance / config_.default_velocity;
  int num_steps = static_cast<int>(std::ceil(travel_time / config_.time_step));

  // 确保有足够的点
  num_steps = std::max(num_steps, 2);

  // 生成轨迹点
  for (int i = 0; i <= num_steps; ++i) {
    double t = i * config_.time_step;
    double ratio = (num_steps > 0) ? static_cast<double>(i) / num_steps : 1.0;

    TrajectoryPoint point;
    point.x = ego_pose.x + ratio * dx;
    point.y = ego_pose.y + ratio * dy;
    point.t = t;
    point.s = ratio * distance;

    // 设置朝向和速度
    if (i == num_steps) {
      // 最后一个点使用目标朝向和零速度
      point.yaw = goal_pose.yaw;
      point.v = 0.0;
      point.a = -config_.default_velocity / travel_time; // 减速到0
    } else {
      // 中间点使用轨迹方向
      point.yaw = trajectory_yaw;
      point.v = config_.default_velocity;
      point.a = 0.0;
    }

    point.kappa = 0.0; // 直线的曲率为0

    result.trajectory.push_back(point);

    // 检查超时
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - start_time > deadline) {
      result.status = PlanningResult::Status::TIMEOUT;
      break;
    }
  }

  // 设置控制指令
  result.control_cmd.steering = trajectory_yaw;
  result.control_cmd.acceleration = 0.0;
  result.control_cmd.velocity = config_.default_velocity;

  // 计算代价
  result.cost.goal = distance;
  result.cost.total = result.cost.goal;

  // 记录计算时间
  auto end_time = std::chrono::steady_clock::now();
  result.diagnostics.computation_time_ms =
    std::chrono::duration<double, std::milli>(end_time - start_time).count();

  return true;
}

std::pair<bool, std::string> StraightLinePlanner::isAvailable(const PlanningContext& context) const {
  // 直线规划器总是可用
  return {true, "Available"};
}

std::unordered_map<std::string, double> StraightLinePlanner::getParameters() const {
  return {
    {"time_step", config_.time_step},
    {"default_velocity", config_.default_velocity},
    {"max_acceleration", config_.max_acceleration},
    {"arrival_tolerance", config_.arrival_tolerance}
  };
}

bool StraightLinePlanner::setParameter(const std::string& name, double value) {
  if (name == "time_step" && value > 0.0) {
    config_.time_step = value;
    return true;
  } else if (name == "default_velocity" && value > 0.0) {
    config_.default_velocity = value;
    return true;
  } else if (name == "max_acceleration" && value > 0.0) {
    config_.max_acceleration = value;
    return true;
  } else if (name == "arrival_tolerance" && value > 0.0) {
    config_.arrival_tolerance = value;
    return true;
  }
  return false;
}

// ========== AStarPlanner ==========

AStarPlanner::AStarPlanner() : config_(Config{}) {}

AStarPlanner::AStarPlanner(const Config& config)
    : config_(config) {}

bool AStarPlanner::plan(const PlanningContext& context,
                       std::chrono::milliseconds deadline,
                       PlanningResult& result) {
  auto start_time = std::chrono::steady_clock::now();

  // 清空结果
  result.trajectory.clear();
  result.status = PlanningResult::Status::SUCCESS;

  // 检查是否有栅格地图
  if (!context.occupancy_grid) {
    result.status = PlanningResult::Status::INVALID_INPUT;
    result.diagnostics.failure_reason = "No occupancy grid available";
    return false;
  }

  // 执行A*搜索
  std::vector<Point2d> path = searchPath(context);

  if (path.empty()) {
    result.status = PlanningResult::Status::NO_SOLUTION;
    result.diagnostics.failure_reason = "No path found";
    return false;
  }

  // 转换为轨迹
  double total_distance = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    TrajectoryPoint point;
    point.x = path[i].x;
    point.y = path[i].y;
    point.t = i * config_.time_step;
    point.s = total_distance;

    // 计算朝向
    if (i + 1 < path.size()) {
      double dx = path[i + 1].x - path[i].x;
      double dy = path[i + 1].y - path[i].y;
      point.yaw = std::atan2(dy, dx);

      double segment_distance = std::sqrt(dx * dx + dy * dy);
      total_distance += segment_distance;
      point.v = segment_distance / config_.time_step;
    } else {
      // 最后一个点
      point.yaw = context.task.goal_pose.yaw;
      point.v = 0.0;
    }

    point.a = 0.0;
    point.kappa = 0.0;

    result.trajectory.push_back(point);

    // 检查超时
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - start_time > deadline) {
      result.status = PlanningResult::Status::TIMEOUT;
      break;
    }
  }

  // 设置控制指令
  if (!result.trajectory.empty()) {
    result.control_cmd.steering = result.trajectory[0].yaw;
    result.control_cmd.velocity = result.trajectory[0].v;
  }

  // 计算代价
  result.cost.goal = total_distance;
  result.cost.total = result.cost.goal;

  // 记录计算时间
  auto end_time = std::chrono::steady_clock::now();
  result.diagnostics.computation_time_ms =
    std::chrono::duration<double, std::milli>(end_time - start_time).count();

  return result.status == PlanningResult::Status::SUCCESS;
}

std::pair<bool, std::string> AStarPlanner::isAvailable(const PlanningContext& context) const {
  if (!context.occupancy_grid) {
    return {false, "No occupancy grid available"};
  }
  return {true, "Available"};
}

std::unordered_map<std::string, double> AStarPlanner::getParameters() const {
  return {
    {"time_step", config_.time_step},
    {"heuristic_weight", config_.heuristic_weight},
    {"step_size", config_.step_size},
    {"max_iterations", static_cast<double>(config_.max_iterations)},
    {"goal_tolerance", config_.goal_tolerance}
  };
}

bool AStarPlanner::setParameter(const std::string& name, double value) {
  if (name == "time_step" && value > 0.0) {
    config_.time_step = value;
    return true;
  } else if (name == "heuristic_weight" && value >= 0.0) {
    config_.heuristic_weight = value;
    return true;
  } else if (name == "step_size" && value > 0.0) {
    config_.step_size = value;
    return true;
  } else if (name == "max_iterations" && value > 0) {
    config_.max_iterations = static_cast<int>(value);
    return true;
  } else if (name == "goal_tolerance" && value > 0.0) {
    config_.goal_tolerance = value;
    return true;
  }
  return false;
}

std::vector<Point2d> AStarPlanner::searchPath(const PlanningContext& context) {
  // 简化的A*实现
  const auto& grid = *context.occupancy_grid;
  Point2d start = {context.ego.pose.x, context.ego.pose.y};
  Point2d goal = {context.task.goal_pose.x, context.task.goal_pose.y};

  // 转换为栅格坐标
  auto [start_x, start_y] = grid.worldToCell(start);
  auto [goal_x, goal_y] = grid.worldToCell(goal);

  // 检查起点和终点是否有效
  if (grid.isOccupied(start_x, start_y) || grid.isOccupied(goal_x, goal_y)) {
    return {}; // 起点或终点被占据
  }

  // A*搜索数据结构
  auto compare = [](const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
    return a->f_cost > b->f_cost;
  };

  std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, decltype(compare)> open_set(compare);
  std::unordered_set<int> closed_set;

  // 起始节点
  auto start_node = std::make_shared<Node>();
  start_node->position = grid.cellToWorld(start_x, start_y);
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

    auto [curr_x, curr_y] = grid.worldToCell(current->position);
    int curr_key = curr_y * grid.config.width + curr_x;

    if (closed_set.count(curr_key)) continue;
    closed_set.insert(curr_key);

    // 检查是否到达目标
    if (heuristic(current->position, goal) < config_.goal_tolerance) {
      // 重构路径
      std::vector<Point2d> path;
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

      if (grid.isOccupied(new_x, new_y)) continue;

      int neighbor_key = new_y * grid.config.width + new_x;
      if (closed_set.count(neighbor_key)) continue;

      auto neighbor = std::make_shared<Node>();
      neighbor->position = grid.cellToWorld(new_x, new_y);
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

  return {}; // 未找到路径
}

double AStarPlanner::heuristic(const Point2d& a, const Point2d& b) const {
  // 欧几里得距离
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool AStarPlanner::isCollisionFree(const Point2d& point, const PlanningContext& context) const {
  if (!context.occupancy_grid) return true;

  const auto& grid = *context.occupancy_grid;
  auto [x, y] = grid.worldToCell(point);
  return !grid.isOccupied(x, y);
}

// ========== OptimizationPlanner ==========

OptimizationPlanner::OptimizationPlanner() : config_(Config{}) {}

OptimizationPlanner::OptimizationPlanner(const Config& config)
    : config_(config) {}

bool OptimizationPlanner::plan(const PlanningContext& context,
                              std::chrono::milliseconds deadline,
                              PlanningResult& result) {
  // TODO: 实现基于优化的规划算法
  // 当前简化为直线规划器的行为
  StraightLinePlanner fallback;
  return fallback.plan(context, deadline, result);
}

std::pair<bool, std::string> OptimizationPlanner::isAvailable(const PlanningContext& context) const {
  return {true, "Available (fallback to straight line)"};
}

std::unordered_map<std::string, double> OptimizationPlanner::getParameters() const {
  return {
    {"time_step", config_.time_step},
    {"max_iterations", static_cast<double>(config_.max_iterations)},
    {"convergence_tolerance", config_.convergence_tolerance},
    {"smoothness_weight", config_.smoothness_weight},
    {"obstacle_weight", config_.obstacle_weight},
    {"goal_weight", config_.goal_weight}
  };
}

bool OptimizationPlanner::setParameter(const std::string& name, double value) {
  // TODO: 实现参数设置
  return false;
}

// ========== PlannerManager ==========

PlannerManager::PlannerManager() : config_(Config{}) {
  if (config_.enable_visualization) {
    visualizer_ = std::make_unique<visualization::WebSocketVisualizer>(config_.viz_config);
  }
}

PlannerManager::PlannerManager(const Config& config)
    : config_(config) {
  if (config_.enable_visualization) {
    visualizer_ = std::make_unique<visualization::WebSocketVisualizer>(config_.viz_config);
  }
}

void PlannerManager::registerPlanner(std::unique_ptr<PlannerInterface> planner) {
  std::string name = planner->getName();
  planners_[name] = std::move(planner);

  // 如果这是第一个规划器，设为活跃规划器
  if (active_planner_.empty()) {
    active_planner_ = name;
  }
}

bool PlannerManager::setActivePlanner(const std::string& name) {
  if (planners_.count(name)) {
    active_planner_ = name;
    return true;
  }
  return false;
}

void PlannerManager::setBridge(Bridge* bridge) {
  if (visualizer_) {
    visualizer_->setBridge(bridge);
  }
}

bool PlannerManager::plan(const PlanningContext& context,
                         std::chrono::milliseconds deadline,
                         PlanningResult& result) {
  stats_.total_plans++;


  // 尝试主规划器
  if (!active_planner_.empty() && planners_.count(active_planner_)) {
    auto& planner = planners_[active_planner_];
    auto [available, reason] = planner->isAvailable(context);

    if (available) {
      bool success = planner->plan(context, deadline, result);
      if (success) {
        stats_.successful_plans++;
        return true;
      }
    }
  }

  // 降级到备用规划器
  if (config_.enable_fallback &&
      !config_.fallback_planner.empty() &&
      planners_.count(config_.fallback_planner) &&
      config_.fallback_planner != active_planner_) {

    auto fallback_deadline = std::chrono::milliseconds(
      static_cast<int>(deadline.count() * config_.fallback_timeout_ratio));

    auto& fallback = planners_[config_.fallback_planner];
    auto [available, reason] = fallback->isAvailable(context);

    if (available) {
      bool success = fallback->plan(context, fallback_deadline, result);
      if (success) {
        stats_.fallback_plans++;
        return true;
      }
    }
  }

  result.status = PlanningResult::Status::FAILED;
  result.diagnostics.failure_reason = "All planners failed";
  return false;
}

std::vector<std::string> PlannerManager::getAvailablePlanners() const {
  std::vector<std::string> names;
  for (const auto& [name, planner] : planners_) {
    names.push_back(name);
  }
  return names;
}

std::unordered_map<std::string, bool> PlannerManager::getPlannerStatus(const PlanningContext& context) const {
  std::unordered_map<std::string, bool> status;
  for (const auto& [name, planner] : planners_) {
    auto [available, reason] = planner->isAvailable(context);
    status[name] = available;
  }
  return status;
}

// ========== ProtocolConverter ==========

bool ProtocolConverter::convertToPlanUpdate(const PlanningResult& result,
                                           uint64_t tick_id,
                                           double timestamp,
                                           proto::PlanUpdate& plan_update) {
  plan_update.Clear();
  plan_update.set_tick_id(tick_id);
  plan_update.set_stamp(timestamp);

  // 设置状态 (当前protobuf定义中没有status字段)
  // TODO: 更新protobuf定义以包含status字段
  // if (result.status == PlanningResult::Status::SUCCESS) {
  //   plan_update.set_status("ok");
  // } else {
  //   plan_update.set_status("failed");
  // }

  // 转换轨迹
  for (const auto& point : result.trajectory) {
    auto* traj_point = plan_update.add_trajectory();
    traj_point->set_x(point.x);
    traj_point->set_y(point.y);
    traj_point->set_yaw(point.yaw);
    traj_point->set_t(point.t);
    // 注意：protobuf中可能没有v, a, kappa字段，根据实际proto定义调整
  }

  // 设置代价信息 (如果protobuf支持)
  // plan_update.mutable_cost()->set_total(result.cost.total);

  // 设置诊断信息 (如果protobuf支持)
  // auto* diag = plan_update.mutable_diag();
  // diag->set_iters(result.diagnostics.iterations);
  // diag->set_conv(result.diagnostics.converged);

  return true;
}

bool ProtocolConverter::convertToEgoCmd(const PlanningResult& result,
                                       uint64_t tick_id,
                                       double timestamp,
                                       proto::EgoCmd& ego_cmd) {
  ego_cmd.Clear();
  // 当前protobuf定义中没有tick_id和stamp字段
  // TODO: 更新protobuf定义或适配当前定义
  // ego_cmd.set_tick_id(tick_id);
  // ego_cmd.set_stamp(timestamp);

  // 设置控制指令
  ego_cmd.set_acceleration(result.control_cmd.acceleration);
  ego_cmd.set_steering(result.control_cmd.steering);

  return true;
}

} // namespace planning
} // namespace navsim
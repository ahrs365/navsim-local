#include "straight_line_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <cmath>
#include <chrono>
#include <iostream>

namespace navsim {
namespace plugins {
namespace planning {

// ========== Config ==========

StraightLinePlannerPlugin::Config StraightLinePlannerPlugin::Config::fromJson(
    const nlohmann::json& json) {
  Config config;
  
  if (json.contains("default_velocity")) {
    config.default_velocity = json["default_velocity"].get<double>();
  }
  if (json.contains("time_step")) {
    config.time_step = json["time_step"].get<double>();
  }
  if (json.contains("planning_horizon")) {
    config.planning_horizon = json["planning_horizon"].get<double>();
  }
  if (json.contains("use_trapezoidal_profile")) {
    config.use_trapezoidal_profile = json["use_trapezoidal_profile"].get<bool>();
  }
  if (json.contains("max_acceleration")) {
    config.max_acceleration = json["max_acceleration"].get<double>();
  }
  
  return config;
}

// ========== StraightLinePlannerPlugin ==========

StraightLinePlannerPlugin::StraightLinePlannerPlugin()
    : config_(Config{}) {
}

StraightLinePlannerPlugin::StraightLinePlannerPlugin(const Config& config)
    : config_(config) {
}

plugin::PlannerPluginMetadata StraightLinePlannerPlugin::getMetadata() const {
  plugin::PlannerPluginMetadata metadata;
  metadata.name = "StraightLinePlanner";
  metadata.version = "1.0.0";
  metadata.description = "Simple geometric planner for fallback";
  metadata.author = "NavSim Team";
  metadata.type = "geometric";
  metadata.required_perception_data = {};  // 不需要感知数据
  metadata.can_be_fallback = true;
  return metadata;
}

bool StraightLinePlannerPlugin::initialize(const nlohmann::json& config) {
  try {
    config_ = Config::fromJson(config);
    
    std::cout << "[StraightLinePlanner] Initialized with config:" << std::endl;
    std::cout << "  - default_velocity: " << config_.default_velocity << " m/s" << std::endl;
    std::cout << "  - planning_horizon: " << config_.planning_horizon << " s" << std::endl;
    std::cout << "  - use_trapezoidal_profile: " << config_.use_trapezoidal_profile << std::endl;
    
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[StraightLinePlanner] Failed to initialize: " << e.what() << std::endl;
    return false;
  }
}

bool StraightLinePlannerPlugin::plan(const navsim::planning::PlanningContext& context,
                                    std::chrono::milliseconds deadline,
                                    plugin::PlanningResult& result) {
  auto start_time = std::chrono::steady_clock::now();
  
  stats_.total_plans++;
  
  // 计算起点和终点
  const auto& start = context.ego.pose;
  const auto& goal = context.task.goal_pose;
  
  // 计算距离
  double dx = goal.x - start.x;
  double dy = goal.y - start.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  
  // 计算轨迹点数量
  int num_points = static_cast<int>(config_.planning_horizon / config_.time_step);
  
  // 生成直线轨迹
  result.trajectory = generateStraightLine(start, goal, num_points);
  
  // 计算速度曲线
  if (config_.use_trapezoidal_profile) {
    computeTrapezoidalProfile(result.trajectory, distance);
  } else {
    computeVelocityProfile(result.trajectory, distance);
  }
  
  // 设置结果
  result.success = true;
  result.planner_name = "StraightLinePlanner";
  
  // 计算耗时
  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_ms = 
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  // 更新统计信息
  stats_.successful_plans++;
  stats_.average_time_ms = 
      (stats_.average_time_ms * (stats_.successful_plans - 1) + result.computation_time_ms) / 
      stats_.successful_plans;
  
  return true;
}

std::pair<bool, std::string> StraightLinePlannerPlugin::isAvailable(const navsim::planning::PlanningContext& context) const {
  // 直线规划器总是可用
  return {true, "Available"};
}

void StraightLinePlannerPlugin::reset() {
  stats_ = Statistics();
}

nlohmann::json StraightLinePlannerPlugin::getStatistics() const {
  nlohmann::json stats;
  stats["total_plans"] = stats_.total_plans;
  stats["successful_plans"] = stats_.successful_plans;
  stats["success_rate"] = stats_.total_plans > 0 ? 
      static_cast<double>(stats_.successful_plans) / stats_.total_plans : 0.0;
  stats["average_time_ms"] = stats_.average_time_ms;
  return stats;
}

// ========== Private Methods ==========

std::vector<plugin::TrajectoryPoint> StraightLinePlannerPlugin::generateStraightLine(
    const navsim::planning::Pose2d& start,
    const navsim::planning::Pose2d& goal,
    int num_points) const {
  std::vector<plugin::TrajectoryPoint> trajectory;
  trajectory.reserve(num_points);
  
  double dx = goal.x - start.x;
  double dy = goal.y - start.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  
  // 计算朝向
  double heading = std::atan2(dy, dx);
  
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    
    plugin::TrajectoryPoint point;
    point.pose.x = start.x + dx * t;
    point.pose.y = start.y + dy * t;
    point.pose.yaw = heading;
    
    point.time_from_start = i * config_.time_step;
    point.path_length = distance * t;
    
    trajectory.push_back(point);
  }
  
  return trajectory;
}

void StraightLinePlannerPlugin::computeVelocityProfile(
    std::vector<plugin::TrajectoryPoint>& trajectory,
    double total_distance) const {
  // 简单的匀速运动
  for (auto& point : trajectory) {
    point.twist.vx = config_.default_velocity;
    point.twist.vy = 0.0;
    point.twist.omega = 0.0;
    point.acceleration = 0.0;
  }
}

void StraightLinePlannerPlugin::computeTrapezoidalProfile(
    std::vector<plugin::TrajectoryPoint>& trajectory,
    double total_distance) const {
  if (trajectory.empty()) return;
  
  double v_max = config_.default_velocity;
  double a_max = config_.max_acceleration;
  
  // 计算加速、匀速、减速的距离
  double t_accel = v_max / a_max;
  double d_accel = 0.5 * a_max * t_accel * t_accel;
  
  // 如果距离太短，无法达到最大速度
  if (2 * d_accel > total_distance) {
    // 三角形速度曲线
    double v_peak = std::sqrt(a_max * total_distance);
    
    for (auto& point : trajectory) {
      double s = point.path_length;
      
      if (s < total_distance / 2.0) {
        // 加速阶段
        point.twist.vx = std::sqrt(2 * a_max * s);
        point.acceleration = a_max;
      } else {
        // 减速阶段
        double s_remaining = total_distance - s;
        point.twist.vx = std::sqrt(2 * a_max * s_remaining);
        point.acceleration = -a_max;
      }
    }
  } else {
    // 梯形速度曲线
    double d_cruise = total_distance - 2 * d_accel;
    
    for (auto& point : trajectory) {
      double s = point.path_length;
      
      if (s < d_accel) {
        // 加速阶段
        point.twist.vx = std::sqrt(2 * a_max * s);
        point.acceleration = a_max;
      } else if (s < d_accel + d_cruise) {
        // 匀速阶段
        point.twist.vx = v_max;
        point.acceleration = 0.0;
      } else {
        // 减速阶段
        double s_remaining = total_distance - s;
        point.twist.vx = std::sqrt(2 * a_max * s_remaining);
        point.acceleration = -a_max;
      }
    }
  }
  
  // 设置其他速度分量
  for (auto& point : trajectory) {
    point.twist.vy = 0.0;
    point.twist.omega = 0.0;
  }
}

} // namespace planning
} // namespace plugins
} // namespace navsim

// 注册插件
namespace {
static navsim::plugin::PlannerPluginRegistrar<navsim::plugins::planning::StraightLinePlannerPlugin>
    straight_line_planner_registrar("StraightLinePlanner");
}


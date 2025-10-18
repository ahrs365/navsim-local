#include "straight_line_plugin.hpp"
#include <iostream>

namespace straight_line {
namespace adapter {

StraightLinePlugin::StraightLinePlugin() {
  planner_ = std::make_unique<algorithm::StraightLine>();
}

navsim::plugin::PlannerPluginMetadata StraightLinePlugin::getMetadata() const {
  navsim::plugin::PlannerPluginMetadata metadata;
  metadata.name = "StraightLine";
  metadata.version = "1.0.0";
  metadata.author = "NavSim Team";
  metadata.description = "Simple straight-line path planner for fallback";
  metadata.type = "geometric";  // 几何规划器

  // 直线规划器不需要感知数据
  metadata.required_perception_data = {};

  metadata.can_be_fallback = true;  // 可以作为降级规划器

  return metadata;
}

bool StraightLinePlugin::initialize(const nlohmann::json& config) {
  try {
    // 从 JSON 解析配置（在 adapter 层完成）
    config_ = parseConfig(config);

    // 设置算法配置
    planner_->setConfig(config_);

    // 打印配置信息
    std::cout << "[StraightLine] Initialized with config:" << std::endl;
    std::cout << "  - default_velocity: " << config_.default_velocity << " m/s" << std::endl;
    std::cout << "  - planning_horizon: " << config_.planning_horizon << " s" << std::endl;
    std::cout << "  - use_trapezoidal_profile: " << config_.use_trapezoidal_profile << std::endl;
    std::cout << "  - max_acceleration: " << config_.max_acceleration << " m/s²" << std::endl;

    initialized_ = true;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "[StraightLine] Failed to initialize: " << e.what() << std::endl;
    return false;
  }
}

bool StraightLinePlugin::plan(
    const navsim::planning::PlanningContext& context,
    std::chrono::milliseconds deadline,
    navsim::plugin::PlanningResult& result) {

  if (!initialized_) {
    result.success = false;
    result.failure_reason = "Plugin not initialized";
    return false;
  }

  // TODO: 如果需要感知数据，在这里检查
  // if (!context.occupancy_grid) {
  //   result.success = false;
  //   result.failure_reason = "Missing occupancy grid";
  //   return false;
  // }

  // 转换输入数据
  Eigen::Vector3d start = convertPose(context.ego.pose);
  Eigen::Vector3d goal = convertPose(context.task.goal_pose);

  // 调用算法
  auto algo_result = planner_->plan(start, goal);

  // 转换输出数据
  result.success = algo_result.success;
  result.failure_reason = algo_result.failure_reason;
  result.computation_time_ms = algo_result.computation_time_ms;
  result.planner_name = "StraightLine";

  if (algo_result.success) {
    // 转换路径为轨迹点
    result.trajectory.reserve(algo_result.path.size());

    for (const auto& wp : algo_result.path) {
      navsim::plugin::TrajectoryPoint point;
      point.pose.x = wp.position.x();
      point.pose.y = wp.position.y();
      point.pose.yaw = wp.position.z();
      point.twist.vx = wp.velocity;
      point.twist.vy = 0.0;
      point.twist.omega = 0.0;
      point.acceleration = wp.acceleration;
      point.time_from_start = wp.timestamp;
      point.path_length = wp.path_length;

      result.trajectory.push_back(point);
    }
  }

  return result.success;
}

std::pair<bool, std::string> StraightLinePlugin::isAvailable(
    const navsim::planning::PlanningContext& context) const {

  // TODO: 根据需要检查必需的感知数据
  // if (!context.occupancy_grid) {
  //   return {false, "Missing occupancy grid"};
  // }

  // 简单的规划器不需要任何感知数据
  return {true, ""};
}

void StraightLinePlugin::reset() {
  if (planner_) {
    planner_->reset();
  }
}

// ========== 私有辅助函数 ==========

Eigen::Vector3d StraightLinePlugin::convertPose(
    const navsim::planning::Pose2d& pose) const {
  return Eigen::Vector3d(pose.x, pose.y, pose.yaw);
}

algorithm::StraightLine::Config StraightLinePlugin::parseConfig(
    const nlohmann::json& json) const {

  algorithm::StraightLine::Config config;

  // 从 JSON 解析配置参数
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

} // namespace adapter
} // namespace straight_line


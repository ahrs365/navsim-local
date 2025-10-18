#include "{{PLUGIN_NAME_SNAKE}}_plugin.hpp"
#include <iostream>

namespace {{NAMESPACE}} {
namespace adapter {

{{PLUGIN_NAME}}Plugin::{{PLUGIN_NAME}}Plugin() {
  planner_ = std::make_unique<algorithm::{{PLUGIN_NAME}}>();
}

navsim::plugin::PlannerPluginMetadata {{PLUGIN_NAME}}Plugin::getMetadata() const {
  navsim::plugin::PlannerPluginMetadata metadata;
  metadata.name = "{{PLUGIN_NAME}}";
  metadata.version = "1.0.0";
  metadata.author = "{{AUTHOR}}";
  metadata.description = "{{DESCRIPTION}}";
  metadata.type = "search";  // TODO: 修改为实际类型 ("search", "optimization", "sampling", "learning")

  // TODO: 根据需要声明必需的感知数据
  // metadata.required_perception_data = {"occupancy_grid"};  // 需要栅格地图
  // metadata.required_perception_data = {"esdf_map"};        // 需要 ESDF 地图
  // metadata.required_perception_data = {};                  // 不需要感知数据

  metadata.can_be_fallback = false;  // TODO: 是否可以作为降级规划器

  return metadata;
}

bool {{PLUGIN_NAME}}Plugin::initialize(const nlohmann::json& config) {
  try {
    // 从 JSON 解析配置（在 adapter 层完成）
    config_ = parseConfig(config);

    // 设置算法配置
    planner_->setConfig(config_);

    // 打印配置信息
    std::cout << "[{{PLUGIN_NAME}}] Initialized with config:" << std::endl;
    std::cout << "  - max_velocity: " << config_.max_velocity << " m/s" << std::endl;
    std::cout << "  - max_acceleration: " << config_.max_acceleration << " m/s²" << std::endl;
    std::cout << "  - step_size: " << config_.step_size << " m" << std::endl;
    std::cout << "  - max_iterations: " << config_.max_iterations << std::endl;

    initialized_ = true;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "[{{PLUGIN_NAME}}] Failed to initialize: " << e.what() << std::endl;
    return false;
  }
}

bool {{PLUGIN_NAME}}Plugin::plan(
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
  result.planner_name = "{{PLUGIN_NAME}}";

  if (algo_result.success) {
    // 转换路径为轨迹点
    result.trajectory_points.reserve(algo_result.path.size());

    for (const auto& wp : algo_result.path) {
      navsim::plugin::TrajectoryPoint point;
      point.pose.x = wp.position.x();
      point.pose.y = wp.position.y();
      point.pose.yaw = wp.position.z();
      point.velocity = wp.velocity;
      point.timestamp = wp.timestamp;

      result.trajectory_points.push_back(point);
    }
  }

  return result.success;
}

std::pair<bool, std::string> {{PLUGIN_NAME}}Plugin::isAvailable(
    const navsim::planning::PlanningContext& context) const {

  // TODO: 根据需要检查必需的感知数据
  // if (!context.occupancy_grid) {
  //   return {false, "Missing occupancy grid"};
  // }

  // 简单的规划器不需要任何感知数据
  return {true, ""};
}

void {{PLUGIN_NAME}}Plugin::reset() {
  if (planner_) {
    planner_->reset();
  }
}

// ========== 私有辅助函数 ==========

Eigen::Vector3d {{PLUGIN_NAME}}Plugin::convertPose(
    const navsim::planning::Pose2d& pose) const {
  return Eigen::Vector3d(pose.x, pose.y, pose.yaw);
}

algorithm::{{PLUGIN_NAME}}::Config {{PLUGIN_NAME}}Plugin::parseConfig(
    const nlohmann::json& json) const {

  algorithm::{{PLUGIN_NAME}}::Config config;

  // 从 JSON 解析配置参数
  if (json.contains("max_velocity")) {
    config.max_velocity = json["max_velocity"].get<double>();
  }
  if (json.contains("max_acceleration")) {
    config.max_acceleration = json["max_acceleration"].get<double>();
  }
  if (json.contains("step_size")) {
    config.step_size = json["step_size"].get<double>();
  }
  if (json.contains("max_iterations")) {
    config.max_iterations = json["max_iterations"].get<int>();
  }

  return config;
}

} // namespace adapter
} // namespace {{NAMESPACE}}


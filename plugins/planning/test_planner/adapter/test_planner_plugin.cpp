#include "test_planner_plugin.hpp"
#include <iostream>

namespace test_planner {
namespace adapter {

TestPlannerPlugin::TestPlannerPlugin() {
  // JPS 需要 ESDF 地图，在 initialize() 中创建
  planner_ = nullptr;
}

navsim::plugin::PlannerPluginMetadata TestPlannerPlugin::getMetadata() const {
  navsim::plugin::PlannerPluginMetadata metadata;
  metadata.name = "TestPlanner";
  metadata.version = "1.0.0";
  metadata.author = "NavSim Team";
  metadata.description = "Test planner for scaffolding tool validation (using JPS algorithm)";
  metadata.type = "search";

  // JPS 需要 ESDF 地图
  metadata.required_perception_data = {"esdf_map"};

  metadata.can_be_fallback = false;

  return metadata;
}

bool TestPlannerPlugin::initialize(const nlohmann::json& config) {
  try {
    // 从 JSON 解析配置（在 adapter 层完成）
    config_ = parseConfig(config);

    // 打印配置信息
    std::cout << "[TestPlanner] Initialized with JPS algorithm config:" << std::endl;
    std::cout << "  - safe_dis: " << config_.safe_dis << " m" << std::endl;
    std::cout << "  - max_jps_dis: " << config_.max_jps_dis << " m" << std::endl;
    std::cout << "  - max_vel: " << config_.max_vel << " m/s" << std::endl;
    std::cout << "  - max_acc: " << config_.max_acc << " m/s²" << std::endl;

    initialized_ = true;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "[TestPlanner] Failed to initialize: " << e.what() << std::endl;
    return false;
  }
}

bool TestPlannerPlugin::plan(
    const navsim::planning::PlanningContext& context,
    std::chrono::milliseconds deadline,
    navsim::plugin::PlanningResult& result) {

  auto start_time = std::chrono::steady_clock::now();

  if (!initialized_) {
    result.success = false;
    result.failure_reason = "Plugin not initialized";
    return false;
  }

  // 获取 perception::ESDFMap from context custom_data
  auto esdf_map_ptr = context.getCustomData<navsim::perception::ESDFMap>("perception_esdf_map");

  if (!esdf_map_ptr) {
    result.success = false;
    result.failure_reason = "Perception ESDF map not available";
    return false;
  }

  // 创建 JPS 规划器实例（每次规划时创建，因为需要最新的 ESDF 地图）
  planner_ = std::make_unique<JPS::JPSPlanner>(esdf_map_ptr);
  planner_->setConfig(config_);

  // 转换输入数据
  Eigen::Vector3d start = convertPose(context.ego.pose);
  Eigen::Vector3d goal = convertPose(context.task.goal_pose);

  // 调用 JPS 算法
  bool success = planner_->plan(start, goal);

  // 转换输出数据
  result.success = success;
  result.planner_name = "TestPlanner";

  if (success) {
    // 从 JPS 获取轨迹数据
    const auto& flat_traj = planner_->getFlatTraj();

    // 转换为轨迹点
    result.trajectory.reserve(flat_traj.UnOccupied_traj_pts.size());

    for (size_t i = 0; i < flat_traj.UnOccupied_traj_pts.size(); ++i) {
      navsim::plugin::TrajectoryPoint point;
      point.pose.x = flat_traj.UnOccupied_traj_pts[i].x();
      point.pose.y = flat_traj.UnOccupied_traj_pts[i].y();
      point.pose.yaw = flat_traj.UnOccupied_traj_pts[i].z();  // yaw 在 z 分量
      point.twist.vx = 0.0;  // JPS 不提供速度信息，使用默认值
      point.time_from_start = flat_traj.UnOccupied_initT + i * config_.sample_time;

      result.trajectory.push_back(point);
    }
  } else {
    result.failure_reason = "JPS planning failed";
  }

  // 计算耗时
  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_ms =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();

  return result.success;
}

std::pair<bool, std::string> TestPlannerPlugin::isAvailable(
    const navsim::planning::PlanningContext& context) const {

  // JPS 需要 perception::ESDFMap
  if (!context.hasCustomData("perception_esdf_map")) {
    return {false, "Perception ESDF map not available"};
  }

  return {true, ""};
}

void TestPlannerPlugin::reset() {
  // JPS 规划器每次规划时都会重新创建，不需要 reset
  planner_.reset();
}

// ========== 私有辅助函数 ==========

Eigen::Vector3d TestPlannerPlugin::convertPose(
    const navsim::planning::Pose2d& pose) const {
  return Eigen::Vector3d(pose.x, pose.y, pose.yaw);
}

JPS::JPSConfig TestPlannerPlugin::parseConfig(
    const nlohmann::json& json) const {

  JPS::JPSConfig config;

  // 从 JSON 解析 JPS 配置参数（使用实际的字段名）
  if (json.contains("safe_dis")) {
    config.safe_dis = json["safe_dis"].get<double>();
  }
  if (json.contains("max_jps_dis")) {
    config.max_jps_dis = json["max_jps_dis"].get<double>();
  }
  if (json.contains("distance_weight")) {
    config.distance_weight = json["distance_weight"].get<double>();
  }
  if (json.contains("yaw_weight")) {
    config.yaw_weight = json["yaw_weight"].get<double>();
  }
  if (json.contains("traj_cut_length")) {
    config.traj_cut_length = json["traj_cut_length"].get<double>();
  }
  if (json.contains("max_vel")) {
    config.max_vel = json["max_vel"].get<double>();
  }
  if (json.contains("max_acc")) {
    config.max_acc = json["max_acc"].get<double>();
  }
  if (json.contains("max_omega")) {
    config.max_omega = json["max_omega"].get<double>();
  }
  if (json.contains("max_domega")) {
    config.max_domega = json["max_domega"].get<double>();
  }
  if (json.contains("sample_time")) {
    config.sample_time = json["sample_time"].get<double>();
  }
  if (json.contains("min_traj_num")) {
    config.min_traj_num = json["min_traj_num"].get<int>();
  }

  return config;
}

} // namespace adapter
} // namespace test_planner


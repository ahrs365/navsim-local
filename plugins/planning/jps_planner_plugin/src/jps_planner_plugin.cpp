/**
 * @file jps_planner_plugin.cpp
 * @brief Implementation of JPSPlannerPlugin
 */

#include "jps_planner_plugin.hpp"
#include <iostream>
#include <cmath>

using namespace navsim::plugins::planning;
using json = nlohmann::json;

// ============================================================================
// Plugin Metadata
// ============================================================================

navsim::plugin::PlannerPluginMetadata JPSPlannerPlugin::getMetadata() const {
  navsim::plugin::PlannerPluginMetadata metadata;
  metadata.name = "JpsPlanner";
  metadata.version = "1.0.0";
  metadata.description = "Jump Point Search (JPS) path planner with trajectory generation";
  metadata.author = "NavSim Team";
  metadata.type = "search";
  metadata.required_perception_data = {"esdf_map"};
  metadata.can_be_fallback = false;
  return metadata;
}

// ============================================================================
// Plugin Lifecycle
// ============================================================================

bool JPSPlannerPlugin::initialize(const json& config) {
  if (initialized_) {
    std::cerr << "[JPSPlannerPlugin] Already initialized!" << std::endl;
    return false;
  }

  // Load configuration
  if (!loadConfig(config)) {
    std::cerr << "[JPSPlannerPlugin] Failed to load configuration!" << std::endl;
    return false;
  }

  // Validate configuration
  if (!validateConfig()) {
    std::cerr << "[JPSPlannerPlugin] Invalid configuration!" << std::endl;
    return false;
  }

  // Note: ESDFMap will be obtained from PlanningContext during planning
  // We don't create the JPS planner here because we need the ESDFMap first

  initialized_ = true;

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Initialized successfully!" << std::endl;
    std::cout << "  - Safe distance: " << jps_config_.safe_dis << " m" << std::endl;
    std::cout << "  - Max velocity: " << jps_config_.max_vel << " m/s" << std::endl;
    std::cout << "  - Max acceleration: " << jps_config_.max_acc << " m/s^2" << std::endl;
    std::cout << "  - Max omega: " << jps_config_.max_omega << " rad/s" << std::endl;
  }

  return true;
}

void JPSPlannerPlugin::reset() {
  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Reset" << std::endl;
  }
  // Reset statistics
  total_plans_ = 0;
  successful_plans_ = 0;
  failed_plans_ = 0;
  total_planning_time_ms_ = 0.0;
}

nlohmann::json JPSPlannerPlugin::getStatistics() const {
  json stats;
  stats["total_plans"] = total_plans_;
  stats["successful_plans"] = successful_plans_;
  stats["failed_plans"] = failed_plans_;
  stats["success_rate"] = (total_plans_ > 0) ? (double)successful_plans_ / total_plans_ : 0.0;
  stats["avg_planning_time_ms"] = (total_plans_ > 0) ? total_planning_time_ms_ / total_plans_ : 0.0;
  return stats;
}

// ============================================================================
// Planning
// ============================================================================

std::pair<bool, std::string> JPSPlannerPlugin::isAvailable(
    const navsim::planning::PlanningContext& context) const {
  if (!initialized_) {
    return {false, "Plugin not initialized"};
  }

  // Check if ESDF map is available
  if (!context.esdf_map) {
    return {false, "ESDF map not available in context"};
  }

  return {true, ""};
}

bool JPSPlannerPlugin::plan(const navsim::planning::PlanningContext& context,
                             std::chrono::milliseconds deadline,
                             plugin::PlanningResult& result) {
  (void)deadline;  // Unused parameter
  auto start_time = std::chrono::steady_clock::now();
  total_plans_++;

  if (!initialized_) {
    std::cerr << "[JPSPlannerPlugin] Not initialized!" << std::endl;
    result.success = false;
    result.failure_reason = "Plugin not initialized";
    failed_plans_++;
    return false;
  }

  // Get perception::ESDFMap from context custom_data
  esdf_map_ = context.getCustomData<navsim::perception::ESDFMap>("perception_esdf_map");

  if (!esdf_map_) {
    std::cerr << "[JPSPlannerPlugin] Perception ESDF map not available in context!" << std::endl;
    result.success = false;
    result.failure_reason = "Perception ESDF map not available";
    failed_plans_++;
    return false;
  }

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] ESDF map pointer: " << esdf_map_.get() << std::endl;
    std::cout << "[JPSPlannerPlugin] ESDF map size: " << esdf_map_->GLX_SIZE_
              << " x " << esdf_map_->GLY_SIZE_ << std::endl;
  }

  // Create or update JPS planner
  if (!jps_planner_ || jps_planner_->getConfig().safe_dis != jps_config_.safe_dis) {
    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] Creating new JPS planner..." << std::endl;
    }
    jps_planner_ = std::make_unique<JPS::JPSPlanner>(esdf_map_);
    jps_planner_->setConfig(jps_config_);
    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] JPS planner created successfully" << std::endl;
    }
  }

  // Convert PlanningContext to JPS input
  Eigen::Vector3d start, goal;
  if (!convertContextToJPSInput(context, start, goal)) {
    std::cerr << "[JPSPlannerPlugin] Failed to convert context to JPS input!" << std::endl;
    result.success = false;
    result.failure_reason = "Failed to convert context to JPS input";
    failed_plans_++;
    return false;
  }

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Planning from " << start.transpose()
              << " to " << goal.transpose() << std::endl;
    std::cout << "[JPSPlannerPlugin] Calling jps_planner_->plan()..." << std::endl;
  }

  // Call JPS planner
  bool success = jps_planner_->plan(start, goal);

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] jps_planner_->plan() returned: " << success << std::endl;
    std::cout << "[JPSPlannerPlugin] Raw path size: " << jps_planner_->getRawPath().size() << std::endl;
    std::cout << "[JPSPlannerPlugin] Optimized path size: " << jps_planner_->getOptimizedPath().size() << std::endl;
  }

  if (!success) {
    std::cerr << "[JPSPlannerPlugin] JPS planning failed!" << std::endl;
    result.success = false;
    result.failure_reason = "JPS planning failed";
    failed_plans_++;
    return false;
  }

  // Convert JPS output to PlanningResult
  if (!convertJPSOutputToResult(*jps_planner_, result)) {
    std::cerr << "[JPSPlannerPlugin] Failed to convert JPS output to result!" << std::endl;
    result.success = false;
    result.failure_reason = "Failed to convert JPS output";
    failed_plans_++;
    return false;
  }

  // Update statistics
  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  double planning_time_ms = duration.count() / 1000.0;
  total_planning_time_ms_ += planning_time_ms;
  successful_plans_++;

  result.success = true;
  result.planner_name = "JPSPlanner";
  result.computation_time_ms = planning_time_ms;

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Planning succeeded! Trajectory length: "
              << result.trajectory.size() << " points, time: " << planning_time_ms << " ms" << std::endl;
  }

  return true;
}

// ============================================================================
// Configuration
// ============================================================================

bool JPSPlannerPlugin::loadConfig(const json& config) {
  try {
    // Load JPS configuration (config is the planner-specific config from default.json)
    jps_config_.safe_dis = config.value("safe_dis", 0.3);
    jps_config_.max_jps_dis = config.value("max_jps_dis", 10.0);
    jps_config_.distance_weight = config.value("distance_weight", 1.0);
    jps_config_.yaw_weight = config.value("yaw_weight", 1.0);
    jps_config_.traj_cut_length = config.value("traj_cut_length", 5.0);
    jps_config_.max_vel = config.value("max_vel", 1.0);
    jps_config_.max_acc = config.value("max_acc", 1.0);
    jps_config_.max_omega = config.value("max_omega", 1.0);
    jps_config_.max_domega = config.value("max_domega", 1.0);
    jps_config_.sample_time = config.value("sample_time", 0.1);
    jps_config_.min_traj_num = config.value("min_traj_num", 10);
    jps_config_.jps_truncation_time = config.value("jps_truncation_time", 5.0);

    // Load plugin configuration
    verbose_ = config.value("verbose", false);

    return true;
  } catch (const std::exception& e) {
    std::cerr << "[JPSPlannerPlugin] Exception while loading config: " << e.what() << std::endl;
    return false;
  }
}

bool JPSPlannerPlugin::validateConfig() const {
  if (jps_config_.safe_dis <= 0.0) {
    std::cerr << "[JPSPlannerPlugin] Invalid safe_dis: " << jps_config_.safe_dis << std::endl;
    return false;
  }

  if (jps_config_.max_vel <= 0.0) {
    std::cerr << "[JPSPlannerPlugin] Invalid max_vel: " << jps_config_.max_vel << std::endl;
    return false;
  }

  if (jps_config_.max_acc <= 0.0) {
    std::cerr << "[JPSPlannerPlugin] Invalid max_acc: " << jps_config_.max_acc << std::endl;
    return false;
  }

  if (jps_config_.max_omega <= 0.0) {
    std::cerr << "[JPSPlannerPlugin] Invalid max_omega: " << jps_config_.max_omega << std::endl;
    return false;
  }

  return true;
}

// ============================================================================
// Data Conversion
// ============================================================================

bool JPSPlannerPlugin::convertContextToJPSInput(const navsim::planning::PlanningContext& context,
                                                 Eigen::Vector3d& start,
                                                 Eigen::Vector3d& goal) const {
  // Extract start from ego vehicle pose
  start.x() = context.ego.pose.x;
  start.y() = context.ego.pose.y;
  start.z() = context.ego.pose.yaw;

  // Extract goal from task
  goal.x() = context.task.goal_pose.x;
  goal.y() = context.task.goal_pose.y;
  goal.z() = context.task.goal_pose.yaw;

  return true;
}

bool JPSPlannerPlugin::convertJPSOutputToResult(const JPS::JPSPlanner& jps_planner,
                                                 plugin::PlanningResult& result) const {
  // Get optimized path
  const auto& path = jps_planner.getOptimizedPath();

  if (path.empty()) {
    std::cerr << "[JPSPlannerPlugin] JPS returned empty path!" << std::endl;
    return false;
  }

  // Convert to PlanningResult format (trajectory points)
  result.trajectory.clear();
  result.trajectory.reserve(path.size());

  double cumulative_time = 0.0;
  double cumulative_length = 0.0;

  for (size_t i = 0; i < path.size(); ++i) {
    plugin::TrajectoryPoint traj_pt;

    // Set pose
    traj_pt.pose.x = path[i].x();
    traj_pt.pose.y = path[i].y();

    // ✅ Calculate yaw from path direction
    if (i + 1 < path.size()) {
      // Use direction to next point
      double dx = path[i + 1].x() - path[i].x();
      double dy = path[i + 1].y() - path[i].y();
      traj_pt.pose.yaw = std::atan2(dy, dx);
    } else if (i > 0) {
      // Last point: use direction from previous point
      double dx = path[i].x() - path[i - 1].x();
      double dy = path[i].y() - path[i - 1].y();
      traj_pt.pose.yaw = std::atan2(dy, dx);
    } else {
      // Single point: use goal yaw (fallback)
      traj_pt.pose.yaw = 0.0;
    }

    // Set velocity (constant velocity for now)
    traj_pt.twist.vx = jps_config_.max_vel;
    traj_pt.twist.vy = 0.0;
    traj_pt.twist.omega = 0.0;

    // Calculate path length
    if (i > 0) {
      double dx = path[i].x() - path[i-1].x();
      double dy = path[i].y() - path[i-1].y();
      double segment_length = std::sqrt(dx*dx + dy*dy);
      cumulative_length += segment_length;
      cumulative_time += segment_length / jps_config_.max_vel;
    }

    traj_pt.time_from_start = cumulative_time;
    traj_pt.path_length = cumulative_length;

    result.trajectory.push_back(traj_pt);
  }

  // Get flat trajectory data (optional, for advanced use)
  const auto& flat_traj = jps_planner.getFlatTraj();

  // Store trajectory data in result metadata (if needed)
  // This can be used by downstream modules for trajectory tracking
  result.metadata["has_trajectory"] = !flat_traj.UnOccupied_traj_pts.empty() ? 1.0 : 0.0;
  result.metadata["trajectory_cut"] = flat_traj.if_cut ? 1.0 : 0.0;
  result.metadata["path_length"] = static_cast<double>(path.size());

  return true;
}




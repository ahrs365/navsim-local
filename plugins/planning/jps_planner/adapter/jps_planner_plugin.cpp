/**
 * @file jps_planner_plugin.cpp
 * @brief Implementation of JPSPlannerPlugin
 */

#include "jps_planner_plugin.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <ctime>

namespace jps_planner {
namespace adapter {

using json = nlohmann::json;

// ============================================================================
// Plugin Metadata
// ============================================================================

navsim::plugin::PlannerPluginMetadata JpsPlannerPlugin::getMetadata() const {
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

bool JpsPlannerPlugin::initialize(const json& config) {
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
    std::cout << "  - Safe distance: " << jps_config_.safe_dis << " m" << std::endl;
    std::cout << "  - Max velocity: " << jps_config_.max_vel << " m/s" << std::endl;
    std::cout << "  - Max acceleration: " << jps_config_.max_acc << " m/s^2" << std::endl;
    std::cout << "  - Max omega: " << jps_config_.max_omega << " rad/s" << std::endl;
  }

  return true;
}

void JpsPlannerPlugin::reset() {
  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Reset" << std::endl;
  }
  // Reset statistics
  total_plans_ = 0;
  successful_plans_ = 0;
  failed_plans_ = 0;
  total_planning_time_ms_ = 0.0;
}

nlohmann::json JpsPlannerPlugin::getStatistics() const {
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

std::pair<bool, std::string> JpsPlannerPlugin::isAvailable(
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

bool JpsPlannerPlugin::plan(const navsim::planning::PlanningContext& context,
                             std::chrono::milliseconds deadline,
                             navsim::plugin::PlanningResult& result) {
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

  // 🔧 从场景配置更新优化器配置（运动学约束、ICR、checkpoint）
  updateOptimizerConfigFromChassis(context.ego, jps_config_.optimizer);

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

  // Create or update MSPlanner (trajectory optimizer)
  // 🔧 注意：每次都重新创建 MSPlanner，因为优化器配置可能已更新
  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Creating MSPlanner (trajectory optimizer) with updated config..." << std::endl;
  }
  msplanner_ = std::make_shared<JPS::MSPlanner>(jps_config_.optimizer, esdf_map_);
  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] MSPlanner created successfully" << std::endl;
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
  if(!success) {
    std::cerr << "[JPSPlannerPlugin] JPS planning failed!" << std::endl;
    result.success = false;
    result.failure_reason = "JPS planning failed";
    failed_plans_++;
    return false;
  } 

  // Trajectory optimization
  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Running trajectory optimization..." << std::endl;
  }

  bool optimize_result = msplanner_->minco_plan(jps_planner_->flat_traj_);
  std::string optimization_status;
  if(!optimize_result) {
    std::cerr << "[JPSPlannerPlugin] Optimization failed!" << std::endl;
    optimization_status = "Optimization failed - using JPS path only";
  } else {
    // Get trajectory total time
    Traj_total_time_ = msplanner_->final_traj_.getTotalDuration();
    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] Optimization succeeded! Total time: " << Traj_total_time_ << " s" << std::endl;
    }
    optimization_status = "Optimization succeeded";
  }


  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] jps_planner_->plan() returned: " << success << std::endl;

    // Debug Path 1: Raw JPS path
    const auto& raw_path = jps_planner_->getRawPath();
    std::cout << "[JPSPlannerPlugin] === PATH 1: Raw JPS path size: " << raw_path.size() << std::endl;
    if (!raw_path.empty()) {
      std::cout << "  First point: " << raw_path[0].transpose() << std::endl;
      std::cout << "  Last point: " << raw_path.back().transpose() << std::endl;
      for (size_t i = 0; i < std::min(raw_path.size(), size_t(5)); ++i) {
        std::cout << "  [" << i << "]: " << raw_path[i].transpose() << std::endl;
      }
    }

    // Debug Path 2: Optimized path (after removeCornerPts)
    const auto& opt_path = jps_planner_->getOptimizedPath();
    std::cout << "[JPSPlannerPlugin] === PATH 2: Optimized path size: " << opt_path.size() << std::endl;
    if (!opt_path.empty()) {
      std::cout << "  First point: " << opt_path[0].transpose() << std::endl;
      std::cout << "  Last point: " << opt_path.back().transpose() << std::endl;
      for (size_t i = 0; i < std::min(opt_path.size(), size_t(5)); ++i) {
        std::cout << "  [" << i << "]: " << opt_path[i].transpose() << std::endl;
      }
    }

    // Debug Path 3: Sampled trajectory (after getSampleTraj)
    const auto& sample_trajs = jps_planner_->getSampleTrajs();
    std::cout << "[JPSPlannerPlugin] === PATH 3: Sample trajectory size: " << sample_trajs.size() << std::endl;
    if (!sample_trajs.empty()) {
      std::cout << "  First sample: [x,y,yaw,dyaw,ds] = " << sample_trajs[0].transpose() << std::endl;
      std::cout << "  Last sample: [x,y,yaw,dyaw,ds] = " << sample_trajs.back().transpose() << std::endl;
      for (size_t i = 0; i < std::min(sample_trajs.size(), size_t(5)); ++i) {
        std::cout << "  [" << i << "]: " << sample_trajs[i].transpose() << std::endl;
      }
    }

    // Debug Path 4: Final FlatTrajData (after getTrajsWithTime)
    const auto& flat_traj = jps_planner_->getFlatTraj();
    std::cout << "[JPSPlannerPlugin] === PATH 4: FlatTrajData size: " << flat_traj.UnOccupied_traj_pts.size() << std::endl;
    std::cout << "  Sample time: " << flat_traj.UnOccupied_initT << std::endl;
    std::cout << "  If cut: " << flat_traj.if_cut << std::endl;
    if (!flat_traj.UnOccupied_traj_pts.empty()) {
      std::cout << "  First traj point [yaw,s,t]: " << flat_traj.UnOccupied_traj_pts[0].transpose() << std::endl;
      std::cout << "  Last traj point [yaw,s,t]: " << flat_traj.UnOccupied_traj_pts.back().transpose() << std::endl;
    }
    if (!flat_traj.UnOccupied_positions.empty()) {
      std::cout << "  First position [x,y,yaw]: " << flat_traj.UnOccupied_positions[0].transpose() << std::endl;
      std::cout << "  Last position [x,y,yaw]: " << flat_traj.UnOccupied_positions.back().transpose() << std::endl;
      for (size_t i = 0; i < std::min(flat_traj.UnOccupied_positions.size(), size_t(5)); ++i) {
        std::cout << "  [" << i << "]: " << flat_traj.UnOccupied_positions[i].transpose() << std::endl;
      }
    }
  }

  if (!success) {
    std::cerr << "[JPSPlannerPlugin] JPS planning failed!" << std::endl;
    result.success = false;
    result.failure_reason = "JPS planning failed";
    failed_plans_++;
    return false;
  }

  // Convert to PlanningResult
  // If optimization succeeded, use MINCO trajectory; otherwise use JPS trajectory
  if (optimize_result) {
    if (!convertMincoOutputToResult(context, result)) {
      std::cerr << "[JPSPlannerPlugin] Failed to convert MINCO output to result!" << std::endl;
      result.success = false;
      result.failure_reason = "Failed to convert MINCO output";
      failed_plans_++;
      return false;
    }
  } else {
    if (!convertJPSOutputToResult(*jps_planner_, result)) {
      std::cerr << "[JPSPlannerPlugin] Failed to convert JPS output to result!" << std::endl;
      result.success = false;
      result.failure_reason = "Failed to convert JPS output";
      failed_plans_++;
      return false;
    }
  }

  // Update statistics
  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  double planning_time_ms = duration.count() / 1000.0;
  total_planning_time_ms_ += planning_time_ms;

  // Set result based on optimization status
  result.success = optimize_result;  // Success only if optimization succeeded
  result.planner_name = "JPSPlanner";
  result.computation_time_ms = planning_time_ms;
  result.failure_reason = optimize_result ? "" : optimization_status;

  if (optimize_result) {
    successful_plans_++;
  } else {
    failed_plans_++;
  }

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Planning " << (optimize_result ? "succeeded" : "partially succeeded (JPS only)")
              << "! Trajectory length: " << result.trajectory.size() << " points, time: "
              << planning_time_ms << " ms" << std::endl;
    std::cout << "[JPSPlannerPlugin] Status: " << optimization_status << std::endl;
  }

  // Store debug paths in result for visualization
  result.metadata["has_debug_paths"] = 1.0;
  result.metadata["optimization_success"] = optimize_result ? 1.0 : 0.0;

  // Store debug paths using a global variable (temporary solution)
  // TODO: Improve this by using proper data structure in PlanningResult
  static std::vector<std::vector<navsim::planning::Pose2d>> global_debug_paths;
  global_debug_paths.clear();

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Preparing debug paths for visualization..." << std::endl;
    std::cout << "[JPSPlannerPlugin] Optimization status: " << optimization_status << std::endl;
  }

  // Collect Raw JPS path
  const auto& raw_path = jps_planner_->getRawPath();
  std::vector<navsim::planning::Pose2d> raw_poses;
  for (const auto& pt : raw_path) {
    navsim::planning::Pose2d pose;
    pose.x = pt.x();
    pose.y = pt.y();
    pose.yaw = 0.0;
    raw_poses.push_back(pose);
  }
  global_debug_paths.push_back(raw_poses);

  // Collect Optimized path
  const auto& opt_path = jps_planner_->getOptimizedPath();
  std::vector<navsim::planning::Pose2d> opt_poses;
  for (const auto& pt : opt_path) {
    navsim::planning::Pose2d pose;
    pose.x = pt.x();
    pose.y = pt.y();
    pose.yaw = 0.0;
    opt_poses.push_back(pose);
  }
  global_debug_paths.push_back(opt_poses);

  // Collect Sample trajectory
  const auto& sample_trajs = jps_planner_->getSampleTrajs();
  std::vector<navsim::planning::Pose2d> sample_poses;
  for (const auto& traj : sample_trajs) {
    if (traj.size() >= 3) {
      navsim::planning::Pose2d pose;
      pose.x = traj[0];
      pose.y = traj[1];
      pose.yaw = traj[2];
      sample_poses.push_back(pose);
    }
  }
  global_debug_paths.push_back(sample_poses);

  // Collect MINCO optimized trajectory (even if optimization failed, we can still visualize the attempt)
  if (msplanner_) {
    std::vector<navsim::plugin::TrajectoryPoint> minco_trajectory = extractMincoTrajectory();
    if (!minco_trajectory.empty()) {
      // Convert TrajectoryPoint to Pose2d for visualization
      std::vector<navsim::planning::Pose2d> minco_poses;
      minco_poses.reserve(minco_trajectory.size());
      for (const auto& traj_pt : minco_trajectory) {
        minco_poses.push_back(traj_pt.pose);
      }
      global_debug_paths.push_back(minco_poses);
      if (verbose_) {
        std::cout << "[JPSPlannerPlugin] === PATH 5: MINCO final trajectory size: " << minco_poses.size() << std::endl;
        if (!minco_poses.empty()) {
          std::cout << "  First point: (" << minco_poses[0].x << ", " << minco_poses[0].y << ")" << std::endl;
          std::cout << "  Last point: (" << minco_poses.back().x << ", " << minco_poses.back().y << ")" << std::endl;
        }
      }
    } else if (verbose_) {
      std::cout << "[JPSPlannerPlugin] === PATH 5: MINCO trajectory extraction failed (empty trajectory)" << std::endl;
    }

    // Collect preprocessing trajectory (Stage 1)
    if (verbose_) {
      Eigen::Vector3d start_state = msplanner_->get_current_iniStateXYTheta();
      std::cout << "[JPSPlannerPlugin] Start state for trajectory extraction: ("
                << start_state.x() << ", " << start_state.y() << ", " << start_state.z() << ")" << std::endl;
    }

    std::vector<navsim::planning::Pose2d> preprocessing_poses = extractPreprocessingTrajectory();
    if (!preprocessing_poses.empty()) {
      global_debug_paths.push_back(preprocessing_poses);
      if (verbose_) {
        std::cout << "[JPSPlannerPlugin] === PATH 6: MINCO preprocessing trajectory (Stage 1) size: "
                  << preprocessing_poses.size() << std::endl;
        if (!preprocessing_poses.empty()) {
          std::cout << "  First point: (" << preprocessing_poses[0].x << ", " << preprocessing_poses[0].y << ")" << std::endl;
          std::cout << "  Last point: (" << preprocessing_poses.back().x << ", " << preprocessing_poses.back().y << ")" << std::endl;
        }
      }
    }

    // Collect main optimization trajectory (Stage 2)
    std::vector<navsim::planning::Pose2d> optimization_poses = extractMainOptimizationTrajectory();
    if (!optimization_poses.empty()) {
      global_debug_paths.push_back(optimization_poses);
      if (verbose_) {
        std::cout << "[JPSPlannerPlugin] === PATH 7: MINCO main optimization trajectory (Stage 2) size: "
                  << optimization_poses.size() << std::endl;
        if (!optimization_poses.empty()) {
          std::cout << "  First point: (" << optimization_poses[0].x << ", " << optimization_poses[0].y << ")" << std::endl;
          std::cout << "  Last point: (" << optimization_poses.back().x << ", " << optimization_poses.back().y << ")" << std::endl;
        }
      }
    }
  }

  // Store a way for the main app to access this data
  result.metadata["debug_paths_ptr"] = static_cast<double>(reinterpret_cast<uintptr_t>(&global_debug_paths));

  return true;
}

// ============================================================================
// Configuration
// ============================================================================

bool JpsPlannerPlugin::loadConfig(const json& config) {
  try {

    // Load JPS configuration (config is the planner-specific config from default.json)
    jps_config_.safe_dis = config.value("safe_dis", 0.3);
    jps_config_.max_jps_dis = config.value("max_jps_dis", 10.0);
    jps_config_.distance_weight = config.value("distance_weight", 1.0);
    jps_config_.yaw_weight = config.value("yaw_weight", 1.0);
    jps_config_.traj_cut_length = std::max(config.value("traj_cut_length", 50.0), 25.0);  // Force minimum 25m to reach goal
    jps_config_.max_vel = config.value("max_vel", 1.0);
    jps_config_.max_acc = config.value("max_acc", 1.0);
    jps_config_.max_omega = config.value("max_omega", 1.0);
    jps_config_.max_domega = config.value("max_domega", 1.0);
    jps_config_.sample_time = config.value("sample_time", 0.1);
    jps_config_.min_traj_num = config.value("min_traj_num", 10);
    jps_config_.jps_truncation_time = config.value("jps_truncation_time", 5.0);

    // Load optimizer configuration
    if (config.contains("optimizer")) {
      const auto& opt_config = config["optimizer"];

      // 🔧 注意：运动学约束（max_vel, max_acc, max_omega）已移至从场景配置动态读取
      // 这些参数现在在 plan() 函数中通过 updateOptimizerConfigFromChassis() 更新
      // 这里只设置默认值，实际值会被场景配置覆盖

      // Kinematic constraints (will be overridden by chassis config)
      jps_config_.optimizer.max_domega = opt_config.value("max_domega", 50.0);
      jps_config_.optimizer.max_centripetal_acc = opt_config.value("max_centripetal_acc", 10000.0);
      jps_config_.optimizer.if_directly_constrain_v_omega = opt_config.value("if_directly_constrain_v_omega", false);

      // Optimizer parameters
      jps_config_.optimizer.mean_time_lowBound = opt_config.value("mean_time_lowBound", 0.5);
      jps_config_.optimizer.mean_time_uppBound = opt_config.value("mean_time_uppBound", 2.0);
      jps_config_.optimizer.smoothEps = opt_config.value("smoothEps", 0.01);
      jps_config_.optimizer.safeDis = opt_config.value("safeDis", 0.3);

      // Final collision check parameters
      jps_config_.optimizer.finalMinSafeDis = opt_config.value("finalMinSafeDis", 0.1);
      jps_config_.optimizer.finalSafeDisCheckNum = opt_config.value("finalSafeDisCheckNum", 16);
      jps_config_.optimizer.safeReplanMaxTime = opt_config.value("safeReplanMaxTime", 3);

      // Penalty weights
      jps_config_.optimizer.time_weight = opt_config.value("time_weight", 50.0);
      jps_config_.optimizer.acc_weight = opt_config.value("acc_weight", 300.0);
      jps_config_.optimizer.domega_weight = opt_config.value("domega_weight", 300.0);
      jps_config_.optimizer.collision_weight = opt_config.value("collision_weight", 500000.0);
      jps_config_.optimizer.moment_weight = opt_config.value("moment_weight", 300.0);
      jps_config_.optimizer.mean_time_weight = opt_config.value("mean_time_weight", 300.0);
      jps_config_.optimizer.cen_acc_weight = opt_config.value("cen_acc_weight", 300.0);

      // Path penalty weights
      jps_config_.optimizer.path_time_weight = opt_config.value("path_time_weight", 20.0);
      jps_config_.optimizer.path_bigpath_sdf_weight = opt_config.value("path_bigpath_sdf_weight", 200000.0);
      jps_config_.optimizer.path_moment_weight = opt_config.value("path_moment_weight", 1000.0);
      jps_config_.optimizer.path_mean_time_weight = opt_config.value("path_mean_time_weight", 100.0);
      jps_config_.optimizer.path_acc_weight = opt_config.value("path_acc_weight", 100.0);
      jps_config_.optimizer.path_domega_weight = opt_config.value("path_domega_weight", 100.0);

      // Energy weights
      if (opt_config.contains("energyWeights") && opt_config["energyWeights"].is_array()) {
        jps_config_.optimizer.energyWeights = opt_config["energyWeights"].get<std::vector<double>>();
      }

      // Augmented Lagrangian parameters
      if (opt_config.contains("EqualLambda") && opt_config["EqualLambda"].is_array()) {
        jps_config_.optimizer.EqualLambda = opt_config["EqualLambda"].get<std::vector<double>>();
      }
      if (opt_config.contains("EqualRho") && opt_config["EqualRho"].is_array()) {
        jps_config_.optimizer.EqualRho = opt_config["EqualRho"].get<std::vector<double>>();
      }
      if (opt_config.contains("EqualRhoMax") && opt_config["EqualRhoMax"].is_array()) {
        jps_config_.optimizer.EqualRhoMax = opt_config["EqualRhoMax"].get<std::vector<double>>();
      }
      if (opt_config.contains("EqualGamma") && opt_config["EqualGamma"].is_array()) {
        jps_config_.optimizer.EqualGamma = opt_config["EqualGamma"].get<std::vector<double>>();
      }
      if (opt_config.contains("EqualTolerance") && opt_config["EqualTolerance"].is_array()) {
        jps_config_.optimizer.EqualTolerance = opt_config["EqualTolerance"].get<std::vector<double>>();
      }

      // Cut trajectory Augmented Lagrangian parameters
      if (opt_config.contains("CutEqualLambda") && opt_config["CutEqualLambda"].is_array()) {
        jps_config_.optimizer.CutEqualLambda = opt_config["CutEqualLambda"].get<std::vector<double>>();
      }
      if (opt_config.contains("CutEqualRho") && opt_config["CutEqualRho"].is_array()) {
        jps_config_.optimizer.CutEqualRho = opt_config["CutEqualRho"].get<std::vector<double>>();
      }
      if (opt_config.contains("CutEqualRhoMax") && opt_config["CutEqualRhoMax"].is_array()) {
        jps_config_.optimizer.CutEqualRhoMax = opt_config["CutEqualRhoMax"].get<std::vector<double>>();
      }
      if (opt_config.contains("CutEqualGamma") && opt_config["CutEqualGamma"].is_array()) {
        jps_config_.optimizer.CutEqualGamma = opt_config["CutEqualGamma"].get<std::vector<double>>();
      }
      if (opt_config.contains("CutEqualTolerance") && opt_config["CutEqualTolerance"].is_array()) {
        jps_config_.optimizer.CutEqualTolerance = opt_config["CutEqualTolerance"].get<std::vector<double>>();
      }

      // LBFGS parameters for path pre-processing
      jps_config_.optimizer.path_lbfgs_mem_size = opt_config.value("path_lbfgs_mem_size", 256);
      jps_config_.optimizer.path_lbfgs_past = opt_config.value("path_lbfgs_past", 2);
      jps_config_.optimizer.path_lbfgs_g_epsilon = opt_config.value("path_lbfgs_g_epsilon", 0.0);
      jps_config_.optimizer.path_lbfgs_min_step = opt_config.value("path_lbfgs_min_step", 0.0);
      jps_config_.optimizer.path_lbfgs_delta = opt_config.value("path_lbfgs_delta", 0.05);
      jps_config_.optimizer.path_lbfgs_max_iterations = opt_config.value("path_lbfgs_max_iterations", 8000);
      jps_config_.optimizer.path_lbfgs_shot_path_past = opt_config.value("path_lbfgs_shot_path_past", 8.0);
      jps_config_.optimizer.path_lbfgs_shot_path_horizon = opt_config.value("path_lbfgs_shot_path_horizon", 0.5);

      // LBFGS parameters for main optimization
      jps_config_.optimizer.lbfgs_mem_size = opt_config.value("lbfgs_mem_size", 256);
      jps_config_.optimizer.lbfgs_past = opt_config.value("lbfgs_past", 3);
      jps_config_.optimizer.lbfgs_g_epsilon = opt_config.value("lbfgs_g_epsilon", 0.0);
      jps_config_.optimizer.lbfgs_min_step = opt_config.value("lbfgs_min_step", 1.0e-32);
      jps_config_.optimizer.lbfgs_delta = opt_config.value("lbfgs_delta", 0.0005);
      jps_config_.optimizer.lbfgs_max_iterations = opt_config.value("lbfgs_max_iterations", 8000);

      // Sampling parameters
      jps_config_.optimizer.sparseResolution = opt_config.value("sparseResolution", 8);
      jps_config_.optimizer.timeResolution = opt_config.value("timeResolution", 0.4);
      jps_config_.optimizer.mintrajNum = opt_config.value("mintrajNum", 3);
      jps_config_.optimizer.trajPredictResolution = opt_config.value("trajPredictResolution", 0.01);

      // Visualization
      jps_config_.optimizer.if_visual_optimization = opt_config.value("if_visual_optimization", false);

      // Horizon limitation
      jps_config_.optimizer.hrz_limited = opt_config.value("hrz_limited", false);
      jps_config_.optimizer.hrz_laser_range_dgr = opt_config.value("hrz_laser_range_dgr", 180.0);

      // 🔧 注意：if_standard_diff 和 ICR 参数已移至从场景配置动态读取
      // 这些参数现在在 plan() 函数中通过 updateOptimizerConfigFromChassis() 更新

      if (verbose_) {
        std::cout << "[JPSPlannerPlugin] Loaded optimizer configuration:" << std::endl;
        std::cout << "  - Collision weight: " << jps_config_.optimizer.collision_weight << std::endl;
        std::cout << "  - Safe distance: " << jps_config_.optimizer.safeDis << " m" << std::endl;
        std::cout << "  - Note: Kinematic constraints (max_vel, max_acc, max_omega) will be loaded from scenario" << std::endl;
      }
    }

    // Load plugin configuration
    verbose_ = true;  // Force enable for testing debug paths

    return true;
  } catch (const std::exception& e) {
    std::cerr << "[JPSPlannerPlugin] Exception while loading config: " << e.what() << std::endl;
    return false;
  }
}

bool JpsPlannerPlugin::validateConfig() const {
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
// Chassis Configuration Update
// ============================================================================

void JpsPlannerPlugin::updateOptimizerConfigFromChassis(
    const navsim::planning::EgoVehicle& ego,
    JPS::OptimizerConfig& config) const {

  // 🔧 从场景配置更新运动学约束
  config.max_vel = ego.limits.max_velocity;
  config.min_vel = -ego.limits.max_velocity;  // 假设对称
  config.max_acc = ego.limits.max_acceleration;
  config.max_omega = ego.limits.max_steer_rate;  // 🔧 使用 max_steer_rate 作为角速度限制

  // 🔧 从底盘类型判断是否为标准差分驱动
  config.if_standard_diff = (ego.chassis_model == "differential");

  // 🔧 计算 ICR (Instantaneous Center of Rotation) 参数
  if (config.if_standard_diff) {
    // 差分驱动底盘
    // ICR.x() = yl (左轮到车体中心的距离) = track_width / 2
    // ICR.y() = yr (右轮到车体中心的距离) = track_width / 2
    // ICR.z() = xv (驱动轮到车体中心的距离，差分驱动通常为 0)
    double half_track = ego.kinematics.track_width / 2.0;
    config.ICR = Eigen::Vector3d(half_track, half_track, 0.0);

    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] Differential drive detected:" << std::endl;
      std::cout << "  - track_width: " << ego.kinematics.track_width << " m" << std::endl;
      std::cout << "  - ICR: [" << config.ICR.x() << ", " << config.ICR.y()
                << ", " << config.ICR.z() << "]" << std::endl;
    }
  } else if (ego.chassis_model == "ackermann" || ego.chassis_model == "four_wheel") {
    // 阿克曼转向底盘
    // ICR 参数需要根据轴距和轮距计算
    double half_track = ego.kinematics.track_width / 2.0;
    double wheelbase = ego.kinematics.wheelbase;
    config.ICR = Eigen::Vector3d(half_track, half_track, wheelbase / 2.0);

    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] Ackermann drive detected:" << std::endl;
      std::cout << "  - wheelbase: " << wheelbase << " m" << std::endl;
      std::cout << "  - track_width: " << ego.kinematics.track_width << " m" << std::endl;
      std::cout << "  - ICR: [" << config.ICR.x() << ", " << config.ICR.y()
                << ", " << config.ICR.z() << "]" << std::endl;
    }
  } else {
    // 其他底盘类型，使用默认值
    config.ICR = Eigen::Vector3d(0.0, 0.0, 1.0);

    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] Unknown chassis type '" << ego.chassis_model
                << "', using default ICR" << std::endl;
    }
  }

  // 🔧 设置碰撞检测点 (checkpoint)
  // 检测车体中心点 + 四个角点，确保整个车体都满足安全距离
  // config.checkpoint.clear();
  // config.checkpoint.push_back(Eigen::Vector2d(0.0, 0.0));  // 车体中心

  // // 添加车体四角的检查点以提高碰撞检测精度
  // double half_length = ego.kinematics.body_length / 2.0;
  // double half_width = ego.kinematics.body_width / 2.0;

  // // 添加四个角点
  // config.checkpoint.push_back(Eigen::Vector2d(half_length, half_width));    // 右前
  // config.checkpoint.push_back(Eigen::Vector2d(half_length, -half_width));   // 左前
  // config.checkpoint.push_back(Eigen::Vector2d(-half_length, half_width));   // 右后
  // config.checkpoint.push_back(Eigen::Vector2d(-half_length, -half_width));  // 左后

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Updated optimizer config from chassis:" << std::endl;
    std::cout << "  - max_vel: " << config.max_vel << " m/s" << std::endl;
    std::cout << "  - max_acc: " << config.max_acc << " m/s^2" << std::endl;
    std::cout << "  - max_omega: " << config.max_omega << " rad/s" << std::endl;
    std::cout << "  - if_standard_diff: " << (config.if_standard_diff ? "true" : "false") << std::endl;
    std::cout << "  - checkpoint count: " << config.checkpoint.size() << std::endl;
  }
}

// ============================================================================
// Data Conversion
// ============================================================================

bool JpsPlannerPlugin::convertContextToJPSInput(const navsim::planning::PlanningContext& context,
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

std::vector<navsim::plugin::TrajectoryPoint> JpsPlannerPlugin::extractMincoTrajectory() const {
  std::vector<navsim::plugin::TrajectoryPoint> minco_trajectory;

  if (!msplanner_) {
    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] extractMincoTrajectory: msplanner_ is null" << std::endl;
    }
    return minco_trajectory;
  }

  const auto& final_traj = msplanner_->final_traj_;

  // Check if trajectory has pieces
  int TrajNum = final_traj.getPieceNum();
  if (TrajNum <= 0) {
    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] extractMincoTrajectory: final_traj has no pieces" << std::endl;
    }
    return minco_trajectory;
  }

  // Get start state from JPS planner
  const auto& flat_traj = jps_planner_->getFlatTraj();
  if (flat_traj.UnOccupied_positions.empty()) {
    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] extractMincoTrajectory: UnOccupied_positions is empty" << std::endl;
    }
    return minco_trajectory;
  }

  double ini_x = flat_traj.UnOccupied_positions[0].x();
  double ini_y = flat_traj.UnOccupied_positions[0].y();

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] extractMincoTrajectory: Starting extraction with "
              << TrajNum << " pieces, start=(" << ini_x << ", " << ini_y << ")" << std::endl;
  }

  // Extract trajectory using Simpson's rule integration (same as ROS version)
  // Reference: mincoPathPub() in optimizer.cpp
  int K = 50;  // Sampling resolution
  int SamNumEachPart = 2 * K;
  double sumT = 0.0;

  Eigen::VectorXd pieceTime = final_traj.getDurations();

  Eigen::Vector2d pos(ini_x, ini_y);
  double cumulative_time = 0.0;
  double cumulative_length = 0.0;

  // 🔧 添加起点（t=0）到轨迹
  {
    navsim::plugin::TrajectoryPoint initial_pt;

    // Get initial state from trajectory at t=0
    Eigen::Vector2d initial_pos = final_traj.getPos(0.0);  // [yaw, s]
    Eigen::Vector2d initial_vel = final_traj.getVel(0.0);  // [dyaw/dt, ds/dt]
    Eigen::Vector2d initial_acc = final_traj.getAcc(0.0);  // [d²yaw/dt², d²s/dt²]

    // Pose
    initial_pt.pose.x = ini_x;
    initial_pt.pose.y = ini_y;
    initial_pt.pose.yaw = initial_pos[0];  // Initial yaw

    // Twist (velocity in body frame)
    initial_pt.twist.vx = initial_vel[1];  // Initial linear velocity
    initial_pt.twist.vy = 0.0;             // No lateral velocity for differential drive
    initial_pt.twist.omega = initial_vel[0];  // Initial angular velocity

    // Acceleration
    initial_pt.acceleration = initial_acc[1];  // Initial linear acceleration

    // Time and path length
    initial_pt.time_from_start = 0.0;
    initial_pt.path_length = 0.0;

    // Curvature (κ = ω / v, avoid division by zero)
    if (std::abs(initial_pt.twist.vx) > 1e-6) {
      initial_pt.curvature = initial_pt.twist.omega / initial_pt.twist.vx;
    } else {
      initial_pt.curvature = 0.0;
    }

    minco_trajectory.push_back(initial_pt);

    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] Added initial point at t=0: "
                << "pos=(" << initial_pt.pose.x << ", " << initial_pt.pose.y << "), "
                << "vx=" << initial_pt.twist.vx << ", omega=" << initial_pt.twist.omega << std::endl;
    }
  }

  for(int i = 0; i < TrajNum; i++) {
    double step = pieceTime[i] / K;
    double halfstep = step / 2.0;
    double CoeffIntegral = pieceTime[i] / K / 6.0;

    Eigen::VectorXd IntegralX(K);
    IntegralX.setZero();
    Eigen::VectorXd IntegralY(K);
    IntegralY.setZero();
    Eigen::VectorXd Yaw(K);
    Yaw.setZero();
    Eigen::VectorXd Vel(K);      // Linear velocity (s)
    Vel.setZero();
    Eigen::VectorXd Omega(K);    // Angular velocity (yaw)
    Omega.setZero();
    Eigen::VectorXd Acc(K);      // Linear acceleration (s)
    Acc.setZero();

    double s1 = 0.0;
    for(int j = 0; j <= SamNumEachPart; j++) {
      if(j % 2 == 0) {
        Eigen::Vector2d currPos = final_traj.getPos(s1 + sumT);  // [yaw, s]
        Eigen::Vector2d currVel = final_traj.getVel(s1 + sumT);  // [dyaw/dt, ds/dt]
        Eigen::Vector2d currAcc = final_traj.getAcc(s1 + sumT);  // [d²yaw/dt², d²s/dt²]
        s1 += halfstep;

        if(j != 0) {
          IntegralX[j/2-1] += CoeffIntegral * currVel.y() * cos(currPos.x());
          IntegralY[j/2-1] += CoeffIntegral * currVel.y() * sin(currPos.x());
          Yaw[j/2-1] = currPos.x();
          Vel[j/2-1] = currVel.y();      // Linear velocity
          Omega[j/2-1] = currVel.x();    // Angular velocity
          Acc[j/2-1] = currAcc.y();      // Linear acceleration
        }
        if(j != SamNumEachPart) {
          IntegralX[j/2] += CoeffIntegral * currVel.y() * cos(currPos.x());
          IntegralY[j/2] += CoeffIntegral * currVel.y() * sin(currPos.x());
        }
      }
      else {
        Eigen::Vector2d currPos = final_traj.getPos(s1 + sumT);
        Eigen::Vector2d currVel = final_traj.getVel(s1 + sumT);
        s1 += halfstep;

        IntegralX[j/2] += 4.0 * CoeffIntegral * currVel.y() * cos(currPos.x());
        IntegralY[j/2] += 4.0 * CoeffIntegral * currVel.y() * sin(currPos.x());
      }
    }

    // Add points for this piece
    for(int j = 0; j < K; j++) {
      pos.x() += IntegralX[j];
      pos.y() += IntegralY[j];

      // Calculate path length increment
      double segment_length = std::sqrt(IntegralX[j]*IntegralX[j] + IntegralY[j]*IntegralY[j]);
      cumulative_length += segment_length;
      cumulative_time += step;

      // Create trajectory point with full dynamics
      navsim::plugin::TrajectoryPoint traj_pt;

      // Pose
      traj_pt.pose.x = pos.x();
      traj_pt.pose.y = pos.y();
      traj_pt.pose.yaw = Yaw[j];

      // Twist (velocity in body frame for differential drive)
      traj_pt.twist.vx = Vel[j];      // Linear velocity (forward)
      traj_pt.twist.vy = 0.0;         // No lateral velocity for differential drive
      traj_pt.twist.omega = Omega[j]; // Angular velocity

      // Acceleration
      traj_pt.acceleration = Acc[j];

      // Time and path length
      traj_pt.time_from_start = cumulative_time;
      traj_pt.path_length = cumulative_length;

      // Curvature (κ = ω / v, avoid division by zero)
      if (std::abs(Vel[j]) > 1e-6) {
        traj_pt.curvature = Omega[j] / Vel[j];
      } else {
        traj_pt.curvature = 0.0;
      }

      minco_trajectory.push_back(traj_pt);
    }

    sumT += pieceTime[i];
  }

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Extracted " << minco_trajectory.size()
              << " trajectory points with full dynamics" << std::endl;
  }

  return minco_trajectory;
}

std::vector<navsim::planning::Pose2d> JpsPlannerPlugin::extractPreprocessingTrajectory() const {
  std::vector<navsim::planning::Pose2d> preprocessing_poses;

  if (!msplanner_) {
    return preprocessing_poses;
  }

  // Extract from init_final_traj_ (preprocessing result, before main optimization)
  const auto& traj = msplanner_->init_final_traj_;
  int TrajNum = traj.getPieceNum();

  if (TrajNum == 0) {
    return preprocessing_poses;
  }

  // Get the correct start position from MSPlanner
  Eigen::Vector3d start_state_XYTheta = msplanner_->get_current_iniStateXYTheta();
  Eigen::Vector2d pos(start_state_XYTheta.x(), start_state_XYTheta.y());

  // Use the same Simpson's rule integration as extractMincoTrajectory()
  int K = 50;  // Sampling resolution
  int SamNumEachPart = 2 * K;
  double sumT = 0.0;

  Eigen::VectorXd pieceTime = traj.getDurations();

  for(int i = 0; i < TrajNum; i++) {
    double step = pieceTime[i] / K;
    double halfstep = step / 2.0;
    double CoeffIntegral = pieceTime[i] / K / 6.0;

    Eigen::VectorXd IntegralX(K);
    IntegralX.setZero();
    Eigen::VectorXd IntegralY(K);
    IntegralY.setZero();
    Eigen::VectorXd Yaw(K);
    Yaw.setZero();

    double s1 = 0.0;
    for(int j = 0; j <= SamNumEachPart; j++) {
      if(j % 2 == 0) {
        Eigen::Vector2d currPos = traj.getPos(s1 + sumT);
        Eigen::Vector2d currVel = traj.getVel(s1 + sumT);
        s1 += halfstep;

        if(j != 0) {
          IntegralX[j/2-1] += CoeffIntegral * currVel.y() * cos(currPos.x());
          IntegralY[j/2-1] += CoeffIntegral * currVel.y() * sin(currPos.x());
          Yaw[j/2-1] = currPos.x();
        }
        if(j != SamNumEachPart) {
          IntegralX[j/2] += CoeffIntegral * currVel.y() * cos(currPos.x());
          IntegralY[j/2] += CoeffIntegral * currVel.y() * sin(currPos.x());
        }
      }
      else {
        Eigen::Vector2d currPos = traj.getPos(s1 + sumT);
        Eigen::Vector2d currVel = traj.getVel(s1 + sumT);
        s1 += halfstep;

        IntegralX[j/2] += 4.0 * CoeffIntegral * currVel.y() * cos(currPos.x());
        IntegralY[j/2] += 4.0 * CoeffIntegral * currVel.y() * sin(currPos.x());
      }
    }

    // Add points for this piece
    for(int j = 0; j < K; j++) {
      pos.x() += IntegralX[j];
      pos.y() += IntegralY[j];

      navsim::planning::Pose2d pose;
      pose.x = pos.x();
      pose.y = pos.y();
      pose.yaw = Yaw[j];
      preprocessing_poses.push_back(pose);
    }

    sumT += pieceTime[i];
  }

  return preprocessing_poses;
}

std::vector<navsim::planning::Pose2d> JpsPlannerPlugin::extractMainOptimizationTrajectory() const {
  std::vector<navsim::planning::Pose2d> optimization_poses;

  if (!msplanner_) {
    return optimization_poses;
  }

  // Extract from optimizer_traj_ (main optimization result, before collision check)
  const auto& traj = msplanner_->optimizer_traj_;
  int TrajNum = traj.getPieceNum();

  if (TrajNum == 0) {
    return optimization_poses;
  }

  // Get the correct start position from MSPlanner
  Eigen::Vector3d start_state_XYTheta = msplanner_->get_current_iniStateXYTheta();
  Eigen::Vector2d pos(start_state_XYTheta.x(), start_state_XYTheta.y());

  // Use the same Simpson's rule integration as extractMincoTrajectory()
  int K = 50;  // Sampling resolution
  int SamNumEachPart = 2 * K;
  double sumT = 0.0;

  Eigen::VectorXd pieceTime = traj.getDurations();

  for(int i = 0; i < TrajNum; i++) {
    double step = pieceTime[i] / K;
    double halfstep = step / 2.0;
    double CoeffIntegral = pieceTime[i] / K / 6.0;

    Eigen::VectorXd IntegralX(K);
    IntegralX.setZero();
    Eigen::VectorXd IntegralY(K);
    IntegralY.setZero();
    Eigen::VectorXd Yaw(K);
    Yaw.setZero();

    double s1 = 0.0;
    for(int j = 0; j <= SamNumEachPart; j++) {
      if(j % 2 == 0) {
        Eigen::Vector2d currPos = traj.getPos(s1 + sumT);
        Eigen::Vector2d currVel = traj.getVel(s1 + sumT);
        s1 += halfstep;

        if(j != 0) {
          IntegralX[j/2-1] += CoeffIntegral * currVel.y() * cos(currPos.x());
          IntegralY[j/2-1] += CoeffIntegral * currVel.y() * sin(currPos.x());
          Yaw[j/2-1] = currPos.x();
        }
        if(j != SamNumEachPart) {
          IntegralX[j/2] += CoeffIntegral * currVel.y() * cos(currPos.x());
          IntegralY[j/2] += CoeffIntegral * currVel.y() * sin(currPos.x());
        }
      }
      else {
        Eigen::Vector2d currPos = traj.getPos(s1 + sumT);
        Eigen::Vector2d currVel = traj.getVel(s1 + sumT);
        s1 += halfstep;

        IntegralX[j/2] += 4.0 * CoeffIntegral * currVel.y() * cos(currPos.x());
        IntegralY[j/2] += 4.0 * CoeffIntegral * currVel.y() * sin(currPos.x());
      }
    }

    // Add points for this piece
    for(int j = 0; j < K; j++) {
      pos.x() += IntegralX[j];
      pos.y() += IntegralY[j];

      navsim::planning::Pose2d pose;
      pose.x = pos.x();
      pose.y = pos.y();
      pose.yaw = Yaw[j];
      optimization_poses.push_back(pose);
    }

    sumT += pieceTime[i];
  }

  return optimization_poses;
}

bool JpsPlannerPlugin::convertMincoOutputToResult(const navsim::planning::PlanningContext& context,
                                                   navsim::plugin::PlanningResult& result) const {
  // Extract MINCO trajectory with full dynamics
  std::vector<navsim::plugin::TrajectoryPoint> minco_trajectory = extractMincoTrajectory();

  if (minco_trajectory.empty()) {
    std::cerr << "[JPSPlannerPlugin] MINCO trajectory is empty!" << std::endl;
    return false;
  }

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] Using MINCO trajectory with "
              << minco_trajectory.size() << " points" << std::endl;
  }

  // Directly use the extracted trajectory (already has full dynamics)
  result.trajectory = minco_trajectory;

  // Store trajectory metadata
  result.metadata["has_trajectory"] = 1.0;
  result.metadata["trajectory_source"] = 1.0;  // 1.0 = MINCO, 0.0 = JPS
  result.metadata["path_length"] = static_cast<double>(minco_trajectory.size());

  if (verbose_) {
    std::cout << "[JPSPlannerPlugin] MINCO trajectory statistics:" << std::endl;
    std::cout << "  - Total points: " << minco_trajectory.size() << std::endl;
    std::cout << "  - Total time: " << result.getTotalTime() << " s" << std::endl;
    std::cout << "  - Total length: " << result.getTotalLength() << " m" << std::endl;
    if (!minco_trajectory.empty()) {
      std::cout << "  - First point: v=" << minco_trajectory.front().twist.vx
                << " m/s, ω=" << minco_trajectory.front().twist.omega << " rad/s" << std::endl;
      std::cout << "  - Last point: v=" << minco_trajectory.back().twist.vx
                << " m/s, ω=" << minco_trajectory.back().twist.omega << " rad/s" << std::endl;
    }

    // 🔍 详细打印前 5 个点的完整信息
    std::cout << "\n[JPSPlannerPlugin] First 5 trajectory points (detailed):" << std::endl;
    size_t num_to_print = std::min(size_t(5), minco_trajectory.size());
    for (size_t i = 0; i < num_to_print; ++i) {
      const auto& pt = minco_trajectory[i];
      std::cout << "  Point[" << i << "]: "
                << "pos=(" << pt.pose.x << ", " << pt.pose.y << ", yaw=" << pt.pose.yaw << "), "
                << "twist=(vx=" << pt.twist.vx << ", vy=" << pt.twist.vy << ", ω=" << pt.twist.omega << "), "
                << "acc=" << pt.acceleration << ", "
                << "curv=" << pt.curvature << ", "
                << "t=" << pt.time_from_start << ", "
                << "s=" << pt.path_length << std::endl;
    }

    // 🔍 详细打印最后 5 个点的完整信息
    std::cout << "\n[JPSPlannerPlugin] Last 5 trajectory points (detailed):" << std::endl;
    size_t start_idx = minco_trajectory.size() > 5 ? minco_trajectory.size() - 5 : 0;
    for (size_t i = start_idx; i < minco_trajectory.size(); ++i) {
      const auto& pt = minco_trajectory[i];
      std::cout << "  Point[" << i << "]: "
                << "pos=(" << pt.pose.x << ", " << pt.pose.y << ", yaw=" << pt.pose.yaw << "), "
                << "twist=(vx=" << pt.twist.vx << ", vy=" << pt.twist.vy << ", ω=" << pt.twist.omega << "), "
                << "acc=" << pt.acceleration << ", "
                << "curv=" << pt.curvature << ", "
                << "t=" << pt.time_from_start << ", "
                << "s=" << pt.path_length << std::endl;
    }

    // 🔍 统计速度、加速度、曲率的范围
    double min_vx = std::numeric_limits<double>::max();
    double max_vx = std::numeric_limits<double>::lowest();
    double min_omega = std::numeric_limits<double>::max();
    double max_omega = std::numeric_limits<double>::lowest();
    double min_acc = std::numeric_limits<double>::max();
    double max_acc = std::numeric_limits<double>::lowest();
    double min_curv = std::numeric_limits<double>::max();
    double max_curv = std::numeric_limits<double>::lowest();

    for (const auto& pt : minco_trajectory) {
      min_vx = std::min(min_vx, pt.twist.vx);
      max_vx = std::max(max_vx, pt.twist.vx);
      min_omega = std::min(min_omega, pt.twist.omega);
      max_omega = std::max(max_omega, pt.twist.omega);
      min_acc = std::min(min_acc, pt.acceleration);
      max_acc = std::max(max_acc, pt.acceleration);
      min_curv = std::min(min_curv, pt.curvature);
      max_curv = std::max(max_curv, pt.curvature);
    }

    std::cout << "\n[JPSPlannerPlugin] Dynamics ranges:" << std::endl;
    std::cout << "  - Velocity (vx): [" << min_vx << ", " << max_vx << "] m/s" << std::endl;
    std::cout << "  - Angular velocity (ω): [" << min_omega << ", " << max_omega << "] rad/s" << std::endl;
    std::cout << "  - Acceleration: [" << min_acc << ", " << max_acc << "] m/s²" << std::endl;
    std::cout << "  - Curvature: [" << min_curv << ", " << max_curv << "] 1/m" << std::endl;
    std::cout << std::endl;
  }

  // 📝 将整条轨迹写入日志文件（包含起点、终点、车辆参数、约束）
  // saveTrajectoryToLog(context, minco_trajectory, result);

  return true;
}

bool JpsPlannerPlugin::convertJPSOutputToResult(const JPS::JPSPlanner& jps_planner,
                                                 navsim::plugin::PlanningResult& result) const {
  // Get optimized path
  const auto& path = jps_planner.getOptimizedPath();

  if (path.empty()) {
    std::cerr << "[JPSPlannerPlugin] JPS returned empty path!" << std::endl;
    return false;
  }

  // Get flat trajectory data
  const auto& flat_traj = jps_planner.getFlatTraj();

  // Convert to PlanningResult format (trajectory points)
  result.trajectory.clear();

  // Use flat trajectory data if available and non-empty
  if (!flat_traj.UnOccupied_positions.empty()) {
    result.trajectory.reserve(flat_traj.UnOccupied_positions.size());

    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] Using FlatTrajData with "
                << flat_traj.UnOccupied_positions.size() << " points" << std::endl;
    }
  } else {
    result.trajectory.reserve(path.size());

    if (verbose_) {
      std::cout << "[JPSPlannerPlugin] Using optimized path with "
                << path.size() << " points" << std::endl;
    }
  }

  double cumulative_time = 0.0;
  double cumulative_length = 0.0;

  // Use FlatTrajData if available, otherwise fall back to path
  if (!flat_traj.UnOccupied_positions.empty()) {
    // Use time-parameterized trajectory from FlatTrajData
    for (size_t i = 0; i < flat_traj.UnOccupied_positions.size(); ++i) {
      navsim::plugin::TrajectoryPoint traj_pt;

      // Set pose from FlatTrajData positions (x, y, yaw)
      const auto& position = flat_traj.UnOccupied_positions[i];
      traj_pt.pose.x = position.x();    // x coordinate
      traj_pt.pose.y = position.y();    // y coordinate
      traj_pt.pose.yaw = position.z();  // yaw angle

      // Set velocity (constant velocity for now, can be improved with velocity profile)
      traj_pt.twist.vx = jps_config_.max_vel;
      traj_pt.twist.vy = 0.0;
      traj_pt.twist.omega = 0.0;

      // Time from FlatTrajData sampling
      traj_pt.time_from_start = i * flat_traj.UnOccupied_initT;

      // Calculate path length
      if (i > 0) {
        const auto& prev_position = flat_traj.UnOccupied_positions[i-1];
        double dx = position.x() - prev_position.x();
        double dy = position.y() - prev_position.y();
        double segment_length = std::sqrt(dx*dx + dy*dy);
        cumulative_length += segment_length;
      }
      traj_pt.path_length = cumulative_length;

      result.trajectory.push_back(traj_pt);
    }
  } else {
    // Fallback to optimized path
    for (size_t i = 0; i < path.size(); ++i) {
      navsim::plugin::TrajectoryPoint traj_pt;

      // Set pose
      traj_pt.pose.x = path[i].x();
      traj_pt.pose.y = path[i].y();

      // Calculate yaw from path direction
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
  }

  // Store trajectory data in result metadata (if needed)
  // This can be used by downstream modules for trajectory tracking
  result.metadata["has_trajectory"] = !flat_traj.UnOccupied_traj_pts.empty() ? 1.0 : 0.0;
  result.metadata["trajectory_cut"] = flat_traj.if_cut ? 1.0 : 0.0;
  result.metadata["path_length"] = static_cast<double>(path.size());

  return true;
}

// ============================================================================
// Trajectory Logging
// ============================================================================

void JpsPlannerPlugin::saveTrajectoryToLog(
    const navsim::planning::PlanningContext& context,
    const std::vector<navsim::plugin::TrajectoryPoint>& minco_trajectory,
    const navsim::plugin::PlanningResult& result) const {

  // 生成带时间戳的文件名
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  std::stringstream filename;
  filename << "minco_trajectory_"
           << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S")
           << "_" << std::setfill('0') << std::setw(3) << now_ms.count()
           << ".log";

  std::ofstream log_file(filename.str());
  if (!log_file.is_open()) {
    std::cerr << "[JPSPlannerPlugin] Failed to open log file for writing: " << filename.str() << std::endl;
    return;
  }

  // 写入文件头
  log_file << "# ============================================================\n";
  log_file << "# MINCO Trajectory Full Log\n";
  log_file << "# ============================================================\n";
  log_file << "#\n";

  // 📊 轨迹统计信息
  log_file << "# [Trajectory Statistics]\n";
  log_file << "# Total points: " << minco_trajectory.size() << "\n";
  log_file << "# Total time: " << result.getTotalTime() << " s\n";
  log_file << "# Total length: " << result.getTotalLength() << " m\n";
  log_file << "#\n";

  // 🚗 起点状态（当前车辆状态）
  log_file << "# [Start State]\n";
  log_file << "# start_x: " << context.ego.pose.x << " m\n";
  log_file << "# start_y: " << context.ego.pose.y << " m\n";
  log_file << "# start_yaw: " << context.ego.pose.yaw << " rad\n";
  log_file << "# start_vx: " << context.ego.twist.vx << " m/s\n";
  log_file << "# start_vy: " << context.ego.twist.vy << " m/s\n";
  log_file << "# start_omega: " << context.ego.twist.omega << " rad/s\n";
  log_file << "#\n";

  // 🎯 终点状态
  log_file << "# [Goal State]\n";
  log_file << "# goal_x: " << context.task.goal_pose.x << " m\n";
  log_file << "# goal_y: " << context.task.goal_pose.y << " m\n";
  log_file << "# goal_yaw: " << context.task.goal_pose.yaw << " rad\n";
  log_file << "#\n";

  // 🚙 车辆几何参数
  log_file << "# [Vehicle Geometry]\n";
  log_file << "# chassis_model: " << context.ego.chassis_model << "\n";
  log_file << "# wheelbase: " << context.ego.kinematics.wheelbase << " m\n";
  log_file << "# track_width: " << context.ego.kinematics.track_width << " m\n";
  log_file << "# front_overhang: " << context.ego.kinematics.front_overhang << " m\n";
  log_file << "# rear_overhang: " << context.ego.kinematics.rear_overhang << " m\n";
  log_file << "# body_length: " << context.ego.kinematics.body_length << " m\n";
  log_file << "# body_width: " << context.ego.kinematics.body_width << " m\n";
  log_file << "# wheel_radius: " << context.ego.kinematics.wheel_radius << " m\n";
  log_file << "#\n";

  // ⚙️ 车辆动力学约束
  log_file << "# [Vehicle Limits]\n";
  log_file << "# max_velocity: " << context.ego.limits.max_velocity << " m/s\n";
  log_file << "# max_acceleration: " << context.ego.limits.max_acceleration << " m/s²\n";
  log_file << "# max_deceleration: " << context.ego.limits.max_deceleration << " m/s²\n";
  log_file << "# max_steer_angle: " << context.ego.limits.max_steer_angle << " rad\n";
  log_file << "# max_steer_rate: " << context.ego.limits.max_steer_rate << " rad/s\n";
  log_file << "# max_jerk: " << context.ego.limits.max_jerk << " m/s³\n";
  log_file << "# max_curvature: " << context.ego.limits.max_curvature << " 1/m\n";
  log_file << "#\n";

  // 🔧 优化器约束
  log_file << "# [Optimizer Constraints]\n";
  log_file << "# max_vel: " << jps_config_.optimizer.max_vel << " m/s\n";
  log_file << "# min_vel: " << jps_config_.optimizer.min_vel << " m/s\n";
  log_file << "# max_acc: " << jps_config_.optimizer.max_acc << " m/s²\n";
  log_file << "# max_omega: " << jps_config_.optimizer.max_omega << " rad/s\n";
  log_file << "# max_domega: " << jps_config_.optimizer.max_domega << " rad/s²\n";
  log_file << "# max_centripetal_acc: " << jps_config_.optimizer.max_centripetal_acc << " m/s²\n";
  log_file << "# safe_distance: " << jps_config_.safe_dis << " m\n";
  log_file << "# final_min_safe_distance: " << jps_config_.optimizer.finalMinSafeDis << " m\n";
  log_file << "#\n";

  // 🎯 优化器权重
  log_file << "# [Optimizer Weights]\n";
  log_file << "# time_weight: " << jps_config_.optimizer.time_weight << "\n";
  log_file << "# acc_weight: " << jps_config_.optimizer.acc_weight << "\n";
  log_file << "# domega_weight: " << jps_config_.optimizer.domega_weight << "\n";
  log_file << "# collision_weight: " << jps_config_.optimizer.collision_weight << "\n";
  log_file << "# moment_weight: " << jps_config_.optimizer.moment_weight << "\n";
  log_file << "# mean_time_weight: " << jps_config_.optimizer.mean_time_weight << "\n";
  log_file << "#\n";

  // 📋 数据格式
  log_file << "# [Trajectory Data Format]\n";
  log_file << "# Columns: index, x, y, yaw, vx, vy, omega, acceleration, curvature, time, path_length\n";
  log_file << "# Units: -, m, m, rad, m/s, m/s, rad/s, m/s², 1/m, s, m\n";
  log_file << "#\n";
  log_file << "# ============================================================\n";
  log_file << "# [Trajectory Points]\n";
  log_file << "# ============================================================\n";

  log_file << std::fixed << std::setprecision(8);

  // 写入轨迹点数据
  for (size_t i = 0; i < minco_trajectory.size(); ++i) {
    const auto& pt = minco_trajectory[i];
    log_file << i << ", "
             << pt.pose.x << ", "
             << pt.pose.y << ", "
             << pt.pose.yaw << ", "
             << pt.twist.vx << ", "
             << pt.twist.vy << ", "
             << pt.twist.omega << ", "
             << pt.acceleration << ", "
             << pt.curvature << ", "
             << pt.time_from_start << ", "
             << pt.path_length << "\n";
  }

  log_file.close();
  std::cout << "[JPSPlannerPlugin] Full trajectory saved to: " << filename.str() << std::endl;
}

}  // namespace adapter
}  // namespace jps_planner




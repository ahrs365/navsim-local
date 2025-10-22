#pragma once

#include "plugin/framework/planner_plugin_interface.hpp"
#include "plugin/data/planning_result.hpp"
#include "core/planning_context.hpp"
#include "esdf_map.hpp"
#include "../algorithm/jps_planner.hpp"
#include "../algorithm/jps_data_structures.hpp"
#include "../algorithm/opt/optimizer.h"
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

namespace jps_planner {
namespace adapter {

/**
 * @brief JpsPlanner Plugin
 *
 * This plugin implements the PlannerPluginInterface and provides
 * Jump Point Search (JPS) path planning functionality.
 * It uses the ESDFMap from the ESDFBuilderPlugin for collision checking.
 */
class JpsPlannerPlugin : public navsim::plugin::PlannerPluginInterface {
public:
  JpsPlannerPlugin() = default;
  ~JpsPlannerPlugin() override = default;

  // ========== Plugin Interface ==========

  /**
   * @brief Get plugin metadata
   */
  navsim::plugin::PlannerPluginMetadata getMetadata() const override;

  /**
   * @brief Initialize the plugin
   * @param config JSON configuration
   * @return true if initialization succeeded
   */
  bool initialize(const nlohmann::json& config) override;

  /**
   * @brief Reset the plugin
   */
  void reset() override;

  /**
   * @brief Check if the plugin is available
   * @param context Planning context
   * @return Pair of (is_available, reason)
   */
  std::pair<bool, std::string> isAvailable(
      const navsim::planning::PlanningContext& context) const override;

  /**
   * @brief Plan a path
   * @param context Planning context
   * @param deadline Planning deadline
   * @param result Planning result (output)
   * @return true if planning succeeded
   */
  bool plan(
      const navsim::planning::PlanningContext& context,
      std::chrono::milliseconds deadline,
      navsim::plugin::PlanningResult& result) override;

  /**
   * @brief Get statistics
   */
  nlohmann::json getStatistics() const override;

private:
  // ========== Configuration ==========

  /**
   * @brief Load configuration from JSON
   * @param config JSON configuration
   * @return true if loading succeeded
   */
  bool loadConfig(const nlohmann::json& config);

  /**
   * @brief Validate configuration
   * @return true if configuration is valid
   */
  bool validateConfig() const;

  // ========== Data Conversion ==========

  /**
   * @brief Convert PlanningContext to JPS input
   * @param context Planning context
   * @param start Output start state (x, y, yaw)
   * @param goal Output goal state (x, y, yaw)
   * @return true if conversion succeeded
   */
  bool convertContextToJPSInput(const navsim::planning::PlanningContext& context,
                                 Eigen::Vector3d& start,
                                 Eigen::Vector3d& goal) const;

  /**
   * @brief Convert JPS output to PlanningResult
   * @param jps_planner JPS planner instance
   * @param result Output planning result
   * @return true if conversion succeeded
   */
  bool convertJPSOutputToResult(const JPS::JPSPlanner& jps_planner,
                                 navsim::plugin::PlanningResult& result) const;

  /**
   * @brief Extract MINCO optimized trajectory with full dynamics
   * @return Vector of trajectory points with pose, twist, acceleration, etc.
   */
  std::vector<navsim::plugin::TrajectoryPoint> extractMincoTrajectory() const;

  /**
   * @brief Extract preprocessing trajectory (before main optimization)
   * @return Vector of poses representing the preprocessing trajectory
   */
  std::vector<navsim::planning::Pose2d> extractPreprocessingTrajectory() const;

  /**
   * @brief Extract main optimization trajectory (after main optimization)
   * @return Vector of poses representing the main optimization trajectory
   */
  std::vector<navsim::planning::Pose2d> extractMainOptimizationTrajectory() const;

  /**
   * @brief Convert MINCO output to PlanningResult format
   * @param context Planning context (for start/goal states and vehicle params)
   * @param result Output planning result
   * @return true if conversion succeeded
   */
  bool convertMincoOutputToResult(const navsim::planning::PlanningContext& context,
                                   navsim::plugin::PlanningResult& result) const;

  /**
   * @brief Update optimizer config from ego vehicle chassis configuration
   * @param ego Ego vehicle from planning context
   * @param config Optimizer config to update (output)
   */
  void updateOptimizerConfigFromChassis(const navsim::planning::EgoVehicle& ego,
                                         JPS::OptimizerConfig& config) const;

  /**
   * @brief Save trajectory to log file with metadata
   * @param context Planning context (for start/goal states and vehicle params)
   * @param minco_trajectory Trajectory points to save
   * @param result Planning result (for statistics)
   */
  void saveTrajectoryToLog(const navsim::planning::PlanningContext& context,
                           const std::vector<navsim::plugin::TrajectoryPoint>& minco_trajectory,
                           const navsim::plugin::PlanningResult& result) const;

  // ========== Member Variables ==========

  // Core algorithm object
  std::unique_ptr<JPS::JPSPlanner> jps_planner_;

  // Trajectory optimizer
  std::shared_ptr<JPS::MSPlanner> msplanner_;

  // ESDFMap (from PlanningContext)
  std::shared_ptr<navsim::perception::ESDFMap> esdf_map_;

  // Configuration
  JPS::JPSConfig jps_config_;

  // Plugin state
  bool initialized_ = false;

  // Trajectory total time (from optimizer)
  double Traj_total_time_ = 0.0;

  // Statistics
  mutable int total_plans_ = 0;
  mutable int successful_plans_ = 0;
  mutable int failed_plans_ = 0;
  mutable double total_planning_time_ms_ = 0.0;

  // Configuration parameters
  bool verbose_ = false;
};

/**
 * @brief Register JpsPlanner plugin
 */
void registerJpsPlannerPlugin();

} // namespace adapter
} // namespace jps_planner


#pragma once

#include "plugin/framework/planner_plugin_interface.hpp"
#include "plugin/data/planning_result.hpp"
#include "core/planning_context.hpp"
#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <Eigen/Dense>

// T-MPC algorithm headers
#include "mpc_planner/planner.h"
#include "mpc_planner_types/realtime_data.h"
#include "mpc_planner_types/data_types.h"
#include "mpc_planner_solver/state.h"
#include "mpc_planner/data_preparation.h"
#include "mpc_planner_util/parameters.h"
#include "guidance_planner/global_guidance.h"
#include "ros_tools/spline.h"
#include "ros_tools/profiling.h"

namespace tmpc_planner {
namespace adapter {

/**
 * @brief T-MPC Planner Plugin
 *
 * This plugin implements the PlannerPluginInterface and provides
 * Topology-based Model Predictive Control (T-MPC) path planning functionality.
 */
class TMPCPlannerPlugin : public navsim::plugin::PlannerPluginInterface {
public:
  TMPCPlannerPlugin() = default;
  ~TMPCPlannerPlugin() override = default;

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
   * @brief Convert PlanningContext ego state to T-MPC State
   * @param context Planning context
   * @param state Output T-MPC state
   * @return true if conversion succeeded
   */
  bool convertEgoState(const navsim::planning::PlanningContext& context,
                       MPCPlanner::State& state) const;

  /**
   * @brief Convert reference line to T-MPC ReferencePath
   * @param reference_line Input reference line from context
   * @param reference_path Output T-MPC reference path
   * @return true if conversion succeeded
   */
  bool convertReferencePath(const std::vector<navsim::planning::Pose2d>& reference_line,
                            MPCPlanner::ReferencePath& reference_path);

  /**
   * @brief Convert dynamic obstacles to T-MPC DynamicObstacle format
   * @param obstacles Input obstacles from context
   * @param state Current robot state (for dummy obstacle placement)
   * @param mpc_obstacles Output T-MPC obstacles
   * @return true if conversion succeeded
   */
  bool convertDynamicObstacles(const std::vector<navsim::planning::DynamicObstacle>& obstacles,
                               const MPCPlanner::State& state,
                               std::vector<MPCPlanner::DynamicObstacle>& mpc_obstacles) const;

  /**
   * @brief Convert static BEV obstacles to T-MPC format
   * @param bev_obstacles BEV obstacles from context
   * @param state Current robot state
   * @param mpc_obstacles Output T-MPC obstacles (appended to existing)
   */
  void convertStaticBEVObstacles(const navsim::planning::BEVObstacles& bev_obstacles,
                                  const MPCPlanner::State& state,
                                  std::vector<MPCPlanner::DynamicObstacle>& mpc_obstacles) const;

  /**
   * @brief Create robot area (multi-disc collision model)
   * @param ego Ego vehicle state
   * @return Vector of discs representing robot footprint
   */
  std::vector<MPCPlanner::Disc> createRobotArea(
      const navsim::planning::EgoVehicle& ego) const;

  /**
   * @brief Generate constant velocity prediction for obstacle
   * @param position Current position
   * @param velocity Current velocity
   * @param dt Time step
   * @param steps Number of prediction steps
   * @return Prediction structure
   */
  MPCPlanner::Prediction getConstantVelocityPrediction(
      const Eigen::Vector2d& position,
      const Eigen::Vector2d& velocity,
      double dt,
      int steps) const;

  /**
   * @brief Extract radius from obstacle shape
   * @param obstacle Obstacle from context
   * @return Radius (for circle) or circumscribed radius (for rectangle)
   */
  double extractObstacleRadius(const navsim::planning::DynamicObstacle& obstacle) const;

  /**
   * @brief Compute reference path progress (spline parameter)
   * @param state Current robot state
   * @return Spline parameter value
   */
  double computeReferenceProgress(const MPCPlanner::State& state) const;

  /**
   * @brief Update guidance trajectories
   * @param state Current robot state
   * @param data Real-time data
   */
  void updateGuidanceTrajectories(const MPCPlanner::State& state,
                                  MPCPlanner::RealTimeData& data);

  /**
   * @brief Convert T-MPC output to PlanningResult
   * @param output MPC solver output
   * @param result Output planning result
   * @return true if conversion succeeded
   */
  bool convertMPCOutputToResult(const MPCPlanner::PlannerOutput& output,
                                 navsim::plugin::PlanningResult& result) const;

  /**
   * @brief Extract debug visualization paths
   * @param state Current state
   * @param data Real-time data
   * @param result Planning result (to store debug paths)
   */
  void extractDebugPaths(const MPCPlanner::State& state,
                         const MPCPlanner::RealTimeData& data,
                         navsim::plugin::PlanningResult& result);

  // ========== Member Variables ==========

  // Core algorithm objects
  std::unique_ptr<MPCPlanner::Planner> planner_;
  std::shared_ptr<GuidancePlanner::GlobalGuidance> global_guidance_;

  // Reference path spline
  std::shared_ptr<RosTools::Spline2D> reference_spline_;

  // T-MPC state and data
  MPCPlanner::State state_;
  MPCPlanner::RealTimeData data_;

  // Configuration parameters
  std::string config_dir_;
  int horizon_steps_ = 30;
  double integrator_step_ = 0.2;
  int max_obstacles_ = 12;
  int n_discs_ = 1;

  // Plugin state
  bool initialized_ = false;
  bool reference_progress_initialized_ = false;
  int reference_segment_ = -1;
  double reference_parameter_ = 0.0;

  // Statistics
  mutable int total_plans_ = 0;
  mutable int successful_plans_ = 0;
  mutable int failed_plans_ = 0;
  mutable double total_planning_time_ms_ = 0.0;

  // Configuration
  bool verbose_ = false;

  // Debug visualization storage
  std::vector<std::vector<navsim::planning::Pose2d>> debug_paths_;
  std::vector<std::string> debug_path_names_;
  std::vector<std::string> debug_path_colors_;
};

/**
 * @brief Register TMPCPlanner plugin
 */
void registerTMPCPlannerPlugin();

} // namespace adapter
} // namespace tmpc_planner


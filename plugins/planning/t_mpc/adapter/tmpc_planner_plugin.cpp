/**
 * @file tmpc_planner_plugin.cpp
 * @brief Implementation of TMPCPlannerPlugin
 */

#include "tmpc_planner_plugin.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>

// Include complete definitions for unique_ptr members
#include "ros_tools/profiling.h"
#include "ros_tools/data_saver.h"

namespace tmpc_planner {
namespace adapter {

using json = nlohmann::json;

// ============================================================================
// Plugin Metadata
// ============================================================================

navsim::plugin::PlannerPluginMetadata TMPCPlannerPlugin::getMetadata() const {
  navsim::plugin::PlannerPluginMetadata metadata;
  metadata.name = "TMPCPlanner";
  metadata.version = "1.0.0";
  metadata.description = "Topology-based Model Predictive Control (T-MPC) planner";
  metadata.author = "NavSim Team";
  metadata.type = "optimization";
  metadata.required_perception_data = {};
  metadata.can_be_fallback = false;
  return metadata;
}

// ============================================================================
// Plugin Lifecycle
// ============================================================================

bool TMPCPlannerPlugin::initialize(const json& config) {
  if (initialized_) {
    std::cerr << "[TMPCPlannerPlugin] Already initialized!" << std::endl;
    return false;
  }

  // Load configuration
  if (!loadConfig(config)) {
    std::cerr << "[TMPCPlannerPlugin] Failed to load configuration!" << std::endl;
    return false;
  }

  // Validate configuration
  if (!validateConfig()) {
    std::cerr << "[TMPCPlannerPlugin] Invalid configuration!" << std::endl;
    return false;
  }

  // Initialize T-MPC configuration
  std::string config_file = config_dir_ + "/settings.yaml";

  // Check if file exists before loading
  std::ifstream test_file(config_file);
  if (!test_file.good()) {
    std::cerr << "[TMPCPlannerPlugin] Config file does not exist or cannot be opened: "
              << config_file << std::endl;
    return false;
  }
  test_file.close();

  try {
    Configuration::getInstance().initialize(config_file);
  } catch (const std::exception& e) {
    std::cerr << "[TMPCPlannerPlugin] Failed to initialize T-MPC configuration: "
              << e.what() << std::endl;
    return false;
  }

  // Read parameters from T-MPC settings.yaml (override default.json values if present)
  horizon_steps_ = CONFIG["N"].as<int>();
  integrator_step_ = CONFIG["integrator_step"].as<double>();
  max_obstacles_ = CONFIG["max_obstacles"].as<int>();
  n_discs_ = CONFIG["n_discs"].as<int>();

  // Create planner
  planner_ = std::make_unique<MPCPlanner::Planner>();

  // Create global guidance planner
  global_guidance_ = std::make_shared<GuidancePlanner::GlobalGuidance>();

  // Initialize state
  state_.initialize();

  // Initialize robot area
  // Note: Will be updated from chassis config during planning
  data_.robot_area = MPCPlanner::defineRobotArea(0.65, 0.65, n_discs_);

  // Initialize past trajectory
  data_.past_trajectory = MPCPlanner::FixedSizeTrajectory(200);

  initialized_ = true;

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Initialized successfully" << std::endl;
    std::cout << "  - Config dir: " << config_dir_ << std::endl;
    std::cout << "  - Horizon steps: " << horizon_steps_ << std::endl;
    std::cout << "  - Integrator step: " << integrator_step_ << " s" << std::endl;
    std::cout << "  - Max obstacles: " << max_obstacles_ << std::endl;
    std::cout << "  - Number of discs: " << n_discs_ << std::endl;
  }

  return true;
}

void TMPCPlannerPlugin::reset() {
  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Reset" << std::endl;
  }

  // Reset statistics
  total_plans_ = 0;
  successful_plans_ = 0;
  failed_plans_ = 0;
  total_planning_time_ms_ = 0.0;

  // Reset reference path tracking
  reference_progress_initialized_ = false;
  reference_segment_ = -1;
  reference_parameter_ = 0.0;
  reference_spline_.reset();

  // Clear past trajectory
  data_.past_trajectory = MPCPlanner::FixedSizeTrajectory(200);
}

nlohmann::json TMPCPlannerPlugin::getStatistics() const {
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

std::pair<bool, std::string> TMPCPlannerPlugin::isAvailable(
    const navsim::planning::PlanningContext& context) const {
  if (!initialized_) {
    return {false, "Plugin not initialized"};
  }

  // T-MPC requires a goal
  // We will generate a simple reference path from current position to goal
  (void)context;  // Suppress unused warning for now

  return {true, ""};
}

bool TMPCPlannerPlugin::plan(const navsim::planning::PlanningContext& context,
                             std::chrono::milliseconds deadline,
                             navsim::plugin::PlanningResult& result) {
  (void)deadline;  // Unused parameter
  auto start_time = std::chrono::steady_clock::now();
  total_plans_++;

  if (!initialized_) {
    std::cerr << "[TMPCPlannerPlugin] Not initialized!" << std::endl;
    result.success = false;
    result.failure_reason = "Plugin not initialized";
    failed_plans_++;
    return false;
  }

  if (verbose_) {
    std::cout << "\n[TMPCPlannerPlugin] ========== Planning Cycle " << total_plans_ 
              << " ==========" << std::endl;
  }

  // ========== Step 1: Convert Ego State ==========
  if (!convertEgoState(context, state_)) {
    std::cerr << "[TMPCPlannerPlugin] Failed to convert ego state!" << std::endl;
    result.success = false;
    result.failure_reason = "Failed to convert ego state";
    failed_plans_++;
    return false;
  }

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Ego state converted:" << std::endl;
    std::cout << "  - Position: (" << state_.get("x") << ", " << state_.get("y") << ")" << std::endl;
    std::cout << "  - Heading: " << state_.get("psi") << " rad" << std::endl;
    std::cout << "  - Velocity: " << state_.get("v") << " m/s" << std::endl;
  }

  // ========== Step 2: Update Robot Area ==========
  data_.robot_area = createRobotArea(context.ego);

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Robot area created: " << data_.robot_area.size() 
              << " discs" << std::endl;
    for (size_t i = 0; i < data_.robot_area.size(); ++i) {
      std::cout << "  - Disc " << i << ": offset=" << data_.robot_area[i].offset 
                << ", radius=" << data_.robot_area[i].radius << std::endl;
    }
  }

  // ========== Step 3: Generate Reference Path ==========
  // Generate a simple straight-line reference path from current position to goal
  std::vector<navsim::planning::Pose2d> reference_line;

  // Start from current position
  navsim::planning::Pose2d start_pose = context.ego.pose;
  navsim::planning::Pose2d goal_pose = context.task.goal_pose;

  // Generate waypoints along straight line
  int num_waypoints = 50;
  for (int i = 0; i <= num_waypoints; ++i) {
    double t = static_cast<double>(i) / num_waypoints;
    navsim::planning::Pose2d waypoint;
    waypoint.x = start_pose.x + t * (goal_pose.x - start_pose.x);
    waypoint.y = start_pose.y + t * (goal_pose.y - start_pose.y);
    waypoint.yaw = std::atan2(goal_pose.y - start_pose.y, goal_pose.x - start_pose.x);
    reference_line.push_back(waypoint);
  }

  if (!convertReferencePath(reference_line, data_.reference_path)) {
    std::cerr << "[TMPCPlannerPlugin] Failed to convert reference path!" << std::endl;
    result.success = false;
    result.failure_reason = "Failed to convert reference path";
    failed_plans_++;
    return false;
  }

  // Notify planner that reference path has been updated
  planner_->onDataReceived(data_, "reference_path");

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Reference path converted: "
              << data_.reference_path.x.size() << " points" << std::endl;
    if (!data_.reference_path.x.empty()) {
      std::cout << "  - Start: (" << data_.reference_path.x.front() << ", "
                << data_.reference_path.y.front() << ")" << std::endl;
      std::cout << "  - End: (" << data_.reference_path.x.back() << ", "
                << data_.reference_path.y.back() << ")" << std::endl;
      std::cout << "  - Total length: " << data_.reference_path.s.back() << " m" << std::endl;
    }
  }

  // ========== Step 4: Set Goal ==========
  if (!data_.reference_path.x.empty()) {
    const auto goal_index = data_.reference_path.x.size() - 1;
    data_.goal = Eigen::Vector2d(data_.reference_path.x[goal_index], 
                                  data_.reference_path.y[goal_index]);
    data_.goal_received = true;

    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin] Goal set: (" << data_.goal.x() << ", " 
                << data_.goal.y() << ")" << std::endl;
    }
  }

  // ========== Step 5: Convert All Obstacles ==========
  // Note: We need to convert both dynamic and static obstacles BEFORE padding with dummies

  // First, convert dynamic obstacles (without padding)
  std::vector<MPCPlanner::DynamicObstacle> temp_obstacles;
  if (!convertDynamicObstacles(context.dynamic_obstacles, state_, temp_obstacles)) {
    std::cerr << "[TMPCPlannerPlugin] Failed to convert dynamic obstacles!" << std::endl;
    result.success = false;
    result.failure_reason = "Failed to convert dynamic obstacles";
    failed_plans_++;
    return false;
  }

  // Then, add static BEV obstacles
  if (context.bev_obstacles) {
    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin] BEV obstacles found: circles=" << context.bev_obstacles->circles.size()
                << ", rectangles=" << context.bev_obstacles->rectangles.size()
                << ", polygons=" << context.bev_obstacles->polygons.size() << std::endl;
    }
    convertStaticBEVObstacles(*context.bev_obstacles, state_, temp_obstacles);
  } else {
    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin] No BEV obstacles in context" << std::endl;
    }
  }

  // Finally, pad with dummies to reach max_obstacles
  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Before ensureObstacleSize: " << temp_obstacles.size() << " obstacles" << std::endl;
  }
  MPCPlanner::ensureObstacleSize(temp_obstacles, state_);
  data_.dynamic_obstacles = temp_obstacles;

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] All obstacles converted: "
              << data_.dynamic_obstacles.size() << " total" << std::endl;
    int real_count = 0;
    int static_count = 0;
    for (const auto& obs : data_.dynamic_obstacles) {
      if (obs.index >= 0) {
        real_count++;
        if (obs.type == MPCPlanner::ObstacleType::STATIC) static_count++;
      }
    }
    std::cout << "  - Real obstacles: " << real_count << " (static: " << static_count << ", dynamic: " << (real_count - static_count) << ")" << std::endl;
    std::cout << "  - Dummy obstacles: " << (data_.dynamic_obstacles.size() - real_count) << std::endl;
  }

  // ========== Step 6: Compute Reference Progress ==========
  double spline_progress = computeReferenceProgress(state_);
  state_.set("spline", spline_progress);

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Reference progress: " << spline_progress << std::endl;
  }

  // ========== Step 7: Update Guidance Trajectories ==========
  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Updating guidance trajectories..." << std::endl;
  }

  try {
    updateGuidanceTrajectories(state_, data_);
    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin] Guidance trajectories updated successfully" << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "[TMPCPlannerPlugin] Guidance planner exception: " << e.what() << std::endl;
    result.success = false;
    result.failure_reason = "Guidance planner failed";
    return false;
  } catch (...) {
    std::cerr << "[TMPCPlannerPlugin] Guidance planner unknown exception" << std::endl;
    result.success = false;
    result.failure_reason = "Guidance planner failed";
    return false;
  }

  // ========== Step 8: Update Past Trajectory ==========
  data_.past_trajectory.add(state_.getPos());

  // ========== Step 9: Call MPC Solver ==========
  MPCPlanner::PlannerOutput mpc_output;

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Calling MPC solver..." << std::endl;
    std::cout << "  - State: x=" << state_.get("x") << ", y=" << state_.get("y")
              << ", psi=" << state_.get("psi") << ", v=" << state_.get("v") << std::endl;
    std::cout << "  - Reference path points: " << data_.reference_path.x.size() << std::endl;
    std::cout << "  - Dynamic obstacles: " << data_.dynamic_obstacles.size() << std::endl;
    std::cout << "  - Robot area discs: " << data_.robot_area.size() << std::endl;
  }

  try {
    mpc_output = planner_->solveMPC(state_, data_);
    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin] MPC solver returned: "
                << (mpc_output.success ? "SUCCESS" : "FAILED") << std::endl;
    }
  } catch (const std::exception& e) {
    if (verbose_) {
      std::cerr << "[TMPCPlannerPlugin] MPC solver exception: " << e.what() << std::endl;
    }
    mpc_output.success = false;
  } catch (...) {
    if (verbose_) {
      std::cerr << "[TMPCPlannerPlugin] MPC solver unknown exception" << std::endl;
    }
    mpc_output.success = false;
  }

  // ========== Step 10: Convert MPC Trajectory to NavSim Format ==========
  if (mpc_output.success && !mpc_output.trajectory.positions.empty()) {
    result.success = true;
    result.planner_name = "TMPCPlanner";
    result.trajectory.clear();

    size_t traj_size = mpc_output.trajectory.positions.size();
    result.trajectory.reserve(traj_size);

    for (size_t i = 0; i < traj_size; ++i) {
      navsim::plugin::TrajectoryPoint point;

      // Position from trajectory
      point.pose.x = mpc_output.trajectory.positions[i].x();
      point.pose.y = mpc_output.trajectory.positions[i].y();

      // Get other states from solver output (k+1 because trajectory starts from k=1)
      int k = i + 1;
      point.pose.yaw = planner_->getSolution(k, "psi");
      point.twist.vx = planner_->getSolution(k, "v");
      point.twist.vy = 0.0;
      point.twist.omega = planner_->getSolution(k, "w");

      // Acceleration (if available)
      try {
        point.acceleration = planner_->getSolution(k, "a");
      } catch (...) {
        point.acceleration = 0.0;
      }

      // Steering angle (if available)
      try {
        point.steering_angle = planner_->getSolution(k, "delta");
      } catch (...) {
        point.steering_angle = 0.0;
      }

      // Time
      point.time_from_start = i * mpc_output.trajectory.dt;

      // Path length (approximate)
      if (i > 0) {
        double dx = point.pose.x - result.trajectory[i-1].pose.x;
        double dy = point.pose.y - result.trajectory[i-1].pose.y;
        point.path_length = result.trajectory[i-1].path_length + std::sqrt(dx*dx + dy*dy);
      } else {
        point.path_length = 0.0;
      }

      result.trajectory.push_back(point);
    }

    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin] MPC trajectory converted: " << traj_size << " points" << std::endl;
      std::cout << "  - Duration: " << result.trajectory.back().time_from_start << " s" << std::endl;
      std::cout << "  - Length: " << result.trajectory.back().path_length << " m" << std::endl;
    }
  } else {
    result.success = false;
    result.planner_name = "TMPCPlanner";
    result.failure_reason = mpc_output.success ? "Empty trajectory" : "MPC solver failed";
    result.trajectory.clear();

    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin] Planning failed: " << result.failure_reason << std::endl;
    }
  }

  // ========== Step 11: Extract Debug Visualization Paths ==========
  extractDebugPaths(state_, data_, result);

  // Update statistics
  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  double planning_time_ms = duration.count() / 1000.0;
  total_planning_time_ms_ += planning_time_ms;
  result.computation_time_ms = planning_time_ms;

  if (result.success) {
    successful_plans_++;
  } else {
    failed_plans_++;
  }

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Planning completed" << std::endl;
    std::cout << "  - Success: " << (result.success ? "yes" : "no") << std::endl;
    std::cout << "  - Computation time: " << planning_time_ms << " ms" << std::endl;
  }

  return result.success;
}

// ============================================================================
// Configuration
// ============================================================================

bool TMPCPlannerPlugin::loadConfig(const json& config) {
  try {
    // Load T-MPC configuration directory
    config_dir_ = config.value("config_dir", 
        "navsim-local/plugins/planning/t_mpc/algorithm/mpc_planner_jackalsimulator/config");

    // Load MPC parameters
    horizon_steps_ = config.value("horizon_steps", 30);
    integrator_step_ = config.value("integrator_step", 0.2);
    max_obstacles_ = config.value("max_obstacles", 12);
    n_discs_ = config.value("n_discs", 1);

    // Load plugin configuration
    verbose_ = config.value("verbose", true);

    return true;
  } catch (const std::exception& e) {
    std::cerr << "[TMPCPlannerPlugin] Exception while loading config: " << e.what() << std::endl;
    return false;
  }
}

bool TMPCPlannerPlugin::validateConfig() const {
  if (horizon_steps_ <= 0) {
    std::cerr << "[TMPCPlannerPlugin] Invalid horizon_steps: " << horizon_steps_ << std::endl;
    return false;
  }

  if (integrator_step_ <= 0.0) {
    std::cerr << "[TMPCPlannerPlugin] Invalid integrator_step: " << integrator_step_ << std::endl;
    return false;
  }

  if (max_obstacles_ <= 0) {
    std::cerr << "[TMPCPlannerPlugin] Invalid max_obstacles: " << max_obstacles_ << std::endl;
    return false;
  }

  if (n_discs_ <= 0) {
    std::cerr << "[TMPCPlannerPlugin] Invalid n_discs: " << n_discs_ << std::endl;
    return false;
  }

  return true;
}

// ============================================================================
// Data Conversion
// ============================================================================

bool TMPCPlannerPlugin::convertEgoState(const navsim::planning::PlanningContext& context,
                                        MPCPlanner::State& state) const {
  // Set position
  state.set("x", context.ego.pose.x);
  state.set("y", context.ego.pose.y);
  state.set("psi", context.ego.pose.yaw);

  // Set velocity (ensure minimum velocity to avoid numerical issues)
  double vx = context.ego.twist.vx;
  double vy = context.ego.twist.vy;
  double v = std::hypot(vx, vy);
  if (v < 0.01) {
    v = 0.1;  // Minimum velocity 0.1 m/s to avoid singularities
  }
  state.set("v", v);

  // Set spline parameter (will be updated later)
  state.set("spline", 0.0);

  // Note: Control inputs (w, a) should NOT be set here as they are not part of the state vector
  // They will be computed by the MPC solver

  return true;
}

bool TMPCPlannerPlugin::convertReferencePath(
    const std::vector<navsim::planning::Pose2d>& reference_line,
    MPCPlanner::ReferencePath& reference_path) {

  reference_path.clear();

  if (reference_line.empty()) {
    std::cerr << "[TMPCPlannerPlugin] Reference line is empty!" << std::endl;
    return false;
  }

  const size_t points = reference_line.size();
  reference_path.x.reserve(points);
  reference_path.y.reserve(points);
  reference_path.psi.reserve(points);
  reference_path.v.reserve(points);
  reference_path.s.reserve(points);

  double s = 0.0;
  for (size_t i = 0; i < points; ++i) {
    reference_path.x.push_back(reference_line[i].x);
    reference_path.y.push_back(reference_line[i].y);
    reference_path.psi.push_back(reference_line[i].yaw);

    // Use constant velocity for now
    // TODO: Extract velocity from reference line if available
    reference_path.v.push_back(1.0);

    // Compute cumulative path length
    if (i == 0) {
      reference_path.s.push_back(0.0);
    } else {
      double dx = reference_line[i].x - reference_line[i-1].x;
      double dy = reference_line[i].y - reference_line[i-1].y;
      s += std::hypot(dx, dy);
      reference_path.s.push_back(s);
    }
  }

  // Create spline for path tracking
  reference_spline_ = std::make_shared<RosTools::Spline2D>(
      reference_path.x, reference_path.y);

  return true;
}

std::vector<MPCPlanner::Disc> TMPCPlannerPlugin::createRobotArea(
    const navsim::planning::EgoVehicle& ego) const {

  double length = ego.kinematics.body_length;
  double width = ego.kinematics.body_width;

  return MPCPlanner::defineRobotArea(length, width, n_discs_);
}

double TMPCPlannerPlugin::extractObstacleRadius(
    const navsim::planning::DynamicObstacle& obstacle) const {

  if (obstacle.shape_type == "circle") {
    // For circle, radius is half of length (diameter)
    return obstacle.length / 2.0;
  } else {
    // For rectangle, use circumscribed circle radius
    double half_length = obstacle.length / 2.0;
    double half_width = obstacle.width / 2.0;
    return std::hypot(half_length, half_width);
  }
}

MPCPlanner::Prediction TMPCPlannerPlugin::getConstantVelocityPrediction(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& velocity,
    double dt,
    int steps) const {

  MPCPlanner::Prediction prediction;

  // Use deterministic prediction for now
  // TODO: Add probabilistic prediction support
  prediction = MPCPlanner::Prediction(MPCPlanner::PredictionType::DETERMINISTIC);

  // Generate prediction trajectory (constant velocity model)
  for (int i = 0; i < steps; i++) {
    Eigen::Vector2d predicted_pos = position + velocity * dt * i;
    prediction.modes[0].push_back(
        MPCPlanner::PredictionStep(predicted_pos, 0.0, 0.0, 0.0));
  }

  return prediction;
}

bool TMPCPlannerPlugin::convertDynamicObstacles(
    const std::vector<navsim::planning::DynamicObstacle>& obstacles,
    const MPCPlanner::State& state,
    std::vector<MPCPlanner::DynamicObstacle>& mpc_obstacles) const {

  mpc_obstacles.clear();

  // Convert each obstacle
  for (size_t i = 0; i < obstacles.size() && i < (size_t)max_obstacles_; ++i) {
    const auto& obs = obstacles[i];

    // Extract position
    Eigen::Vector2d position(obs.current_pose.x, obs.current_pose.y);

    // Extract velocity
    Eigen::Vector2d velocity(obs.current_twist.vx, obs.current_twist.vy);

    // Extract radius
    double radius = extractObstacleRadius(obs);

    // Create T-MPC obstacle
    MPCPlanner::DynamicObstacle mpc_obs(
        i,  // index
        position,
        obs.current_pose.yaw,
        radius
    );

    // Generate prediction
    mpc_obs.prediction = getConstantVelocityPrediction(
        position, velocity, integrator_step_, horizon_steps_);

    // Determine if static
    if (velocity.norm() < 0.01) {
      mpc_obs.type = MPCPlanner::ObstacleType::STATIC;
    }

    mpc_obstacles.push_back(mpc_obs);
  }

  // Note: Do NOT pad with dummies here - will be done after adding static obstacles

  return true;
}

void TMPCPlannerPlugin::convertStaticBEVObstacles(
    const navsim::planning::BEVObstacles& bev_obstacles,
    const MPCPlanner::State& state,
    std::vector<MPCPlanner::DynamicObstacle>& mpc_obstacles) const {

  int start_index = mpc_obstacles.size();

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Converting static BEV obstacles, start_index=" << start_index
              << ", max_obstacles=" << max_obstacles_ << std::endl;
  }

  // Convert circle obstacles
  for (const auto& circle : bev_obstacles.circles) {
    if (start_index >= max_obstacles_) {
      if (verbose_) {
        std::cout << "[TMPCPlannerPlugin]   Skipping circle: start_index=" << start_index << " >= max_obstacles=" << max_obstacles_ << std::endl;
      }
      break;
    }

    Eigen::Vector2d position(circle.center.x, circle.center.y);
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();

    MPCPlanner::DynamicObstacle mpc_obs(
        start_index,
        position,
        0.0,  // yaw
        circle.radius
    );

    // Static obstacle - zero velocity prediction
    mpc_obs.prediction = getConstantVelocityPrediction(
        position, velocity, integrator_step_, horizon_steps_);
    mpc_obs.type = MPCPlanner::ObstacleType::STATIC;

    mpc_obstacles.push_back(mpc_obs);
    start_index++;

    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin]   Added static circle: pos=("
                << position.x() << ", " << position.y() << "), r=" << circle.radius << std::endl;
    }
  }

  // Convert polygon obstacles (approximate with multiple circles)
  for (const auto& polygon : bev_obstacles.polygons) {
    if (start_index >= max_obstacles_) break;
    if (polygon.vertices.empty()) continue;

    // Compute centroid
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto& vertex : polygon.vertices) {
      centroid.x() += vertex.x;
      centroid.y() += vertex.y;
    }
    centroid /= polygon.vertices.size();

    // For 4-vertex polygons (rectangles), use multi-circle approximation
    if (polygon.vertices.size() == 4) {
      // Compute the two diagonal lengths to find major/minor axes
      std::vector<Eigen::Vector2d> verts;
      for (const auto& v : polygon.vertices) {
        verts.emplace_back(v.x, v.y);
      }

      // Compute edge lengths
      double edge1 = (verts[1] - verts[0]).norm();
      double edge2 = (verts[2] - verts[1]).norm();

      // Major and minor axes
      double major_axis = std::max(edge1, edge2);
      double minor_axis = std::min(edge1, edge2);

      // Major axis direction
      Eigen::Vector2d major_dir;
      if (edge1 >= edge2) {
        major_dir = (verts[1] - verts[0]).normalized();
      } else {
        major_dir = (verts[2] - verts[1]).normalized();
      }

      // Circle radius = half of minor axis
      double circle_radius = minor_axis / 2.0;

      // Number of circles
      int num_circles = std::max(1, static_cast<int>(std::ceil(major_axis / minor_axis)));
      num_circles = std::min(num_circles, std::max(1, max_obstacles_ - start_index));

      // Spacing between circles
      double spacing = (num_circles > 1) ? (major_axis - minor_axis) / (num_circles - 1) : 0.0;

      // Starting position
      Eigen::Vector2d start_pos = centroid - major_dir * (major_axis / 2.0 - circle_radius);

      if (verbose_) {
        std::cout << "[TMPCPlannerPlugin]   Adding static polygon (rectangle): centroid=("
                  << centroid.x() << ", " << centroid.y() << "), major=" << major_axis
                  << ", minor=" << minor_axis << " → " << num_circles << " circles (r="
                  << circle_radius << ")" << std::endl;
      }

      // Create circles along major axis
      for (int i = 0; i < num_circles && start_index < max_obstacles_; ++i) {
        Eigen::Vector2d circle_pos = start_pos + major_dir * (i * spacing);
        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();

        MPCPlanner::DynamicObstacle mpc_obs(
            start_index,
            circle_pos,
            0.0,  // yaw
            circle_radius
        );

        // Static obstacle - zero velocity prediction
        mpc_obs.prediction = getConstantVelocityPrediction(
            circle_pos, velocity, integrator_step_, horizon_steps_);
        mpc_obs.type = MPCPlanner::ObstacleType::STATIC;

        mpc_obstacles.push_back(mpc_obs);
        start_index++;

        if (verbose_) {
          std::cout << "[TMPCPlannerPlugin]     Circle " << i << ": pos=("
                    << circle_pos.x() << ", " << circle_pos.y() << ")" << std::endl;
        }
      }
    } else {
      // For non-rectangular polygons, use single circumscribed circle
      double max_radius = 0.0;
      for (const auto& vertex : polygon.vertices) {
        double dx = vertex.x - centroid.x();
        double dy = vertex.y - centroid.y();
        double dist = std::hypot(dx, dy);
        max_radius = std::max(max_radius, dist);
      }

      Eigen::Vector2d velocity = Eigen::Vector2d::Zero();

      MPCPlanner::DynamicObstacle mpc_obs(
          start_index,
          centroid,
          0.0,  // yaw
          max_radius
      );

      // Static obstacle - zero velocity prediction
      mpc_obs.prediction = getConstantVelocityPrediction(
          centroid, velocity, integrator_step_, horizon_steps_);
      mpc_obs.type = MPCPlanner::ObstacleType::STATIC;

      mpc_obstacles.push_back(mpc_obs);
      start_index++;

      if (verbose_) {
        std::cout << "[TMPCPlannerPlugin]   Added static polygon: centroid=("
                  << centroid.x() << ", " << centroid.y() << "), r=" << max_radius
                  << " (vertices=" << polygon.vertices.size() << ")" << std::endl;
      }
    }
  }

  // Convert rectangle obstacles (approximate with multiple circles along major axis)
  for (const auto& rect : bev_obstacles.rectangles) {
    if (start_index >= max_obstacles_) break;

    // Determine major and minor axes
    double major_axis = std::max(rect.width, rect.height);
    double minor_axis = std::min(rect.width, rect.height);
    bool width_is_major = (rect.width >= rect.height);

    // Circle radius = half of minor axis (so circles cover the width)
    double circle_radius = minor_axis / 2.0;

    // Number of circles needed to cover the major axis
    // Use spacing = minor_axis to ensure overlap
    int num_circles = std::max(1, static_cast<int>(std::ceil(major_axis / minor_axis)));

    // Limit number of circles to avoid using too many obstacle slots
    num_circles = std::min(num_circles, std::max(1, max_obstacles_ - start_index));

    // Spacing between circle centers
    double spacing = (num_circles > 1) ? (major_axis - minor_axis) / (num_circles - 1) : 0.0;

    // Direction along major axis (in world frame)
    double cos_yaw = std::cos(rect.pose.yaw);
    double sin_yaw = std::sin(rect.pose.yaw);
    Eigen::Vector2d major_axis_dir;
    if (width_is_major) {
      major_axis_dir = Eigen::Vector2d(cos_yaw, sin_yaw);  // Along width
    } else {
      major_axis_dir = Eigen::Vector2d(-sin_yaw, cos_yaw);  // Along height (perpendicular)
    }

    // Starting position (one end of the rectangle)
    Eigen::Vector2d center(rect.pose.x, rect.pose.y);
    Eigen::Vector2d start_pos = center - major_axis_dir * (major_axis / 2.0 - circle_radius);

    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin]   Adding static rectangle: center=("
                << center.x() << ", " << center.y() << "), w=" << rect.width
                << ", h=" << rect.height << " → " << num_circles << " circles (r="
                << circle_radius << ")" << std::endl;
    }

    // Create circles along the major axis
    for (int i = 0; i < num_circles && start_index < max_obstacles_; ++i) {
      Eigen::Vector2d circle_pos = start_pos + major_axis_dir * (i * spacing);
      Eigen::Vector2d velocity = Eigen::Vector2d::Zero();

      MPCPlanner::DynamicObstacle mpc_obs(
          start_index,
          circle_pos,
          rect.pose.yaw,
          circle_radius
      );

      // Static obstacle - zero velocity prediction
      mpc_obs.prediction = getConstantVelocityPrediction(
          circle_pos, velocity, integrator_step_, horizon_steps_);
      mpc_obs.type = MPCPlanner::ObstacleType::STATIC;

      mpc_obstacles.push_back(mpc_obs);
      start_index++;

      if (verbose_) {
        std::cout << "[TMPCPlannerPlugin]     Circle " << i << ": pos=("
                  << circle_pos.x() << ", " << circle_pos.y() << ")" << std::endl;
      }
    }
  }
}

double TMPCPlannerPlugin::computeReferenceProgress(const MPCPlanner::State& state) const {
  if (!reference_spline_) {
    return 0.0;
  }

  Eigen::Vector2d robot_pos = state.getPos();

  // Find closest point on spline
  double best_param = 0.0;
  double best_dist = std::numeric_limits<double>::infinity();

  // Sample along spline to find closest point
  double max_param = reference_spline_->parameterLength();
  int samples = 100;
  for (int i = 0; i <= samples; ++i) {
    double param = max_param * i / samples;
    Eigen::Vector2d spline_point = reference_spline_->getPoint(param);
    double dist = (spline_point - robot_pos).norm();

    if (dist < best_dist) {
      best_dist = dist;
      best_param = param;
    }
  }

  return best_param;
}

void TMPCPlannerPlugin::updateGuidanceTrajectories(const MPCPlanner::State& state,
                                                    MPCPlanner::RealTimeData& data) {
  if (!global_guidance_ || !reference_spline_) {
    return;
  }

  double spline_position = state.get("spline");
  double robot_radius = data.robot_area.empty() ? 0.0 : data.robot_area.front().radius;

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin]   Step 1: Setting start state..." << std::endl;
  }
  // Step 1: Set start state (must be done first)
  global_guidance_->SetStart(state.getPos(), state.get("psi"), state.get("v"));

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin]   Step 2: Setting reference velocity..." << std::endl;
  }
  // Step 2: Set reference velocity
  double reference_velocity = std::max(0.5, state.get("v"));
  global_guidance_->SetReferenceVelocity(reference_velocity);

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin]   Step 3: Loading reference path..." << std::endl;
    std::cout << "[TMPCPlannerPlugin]     - Spline position: " << spline_position << std::endl;
    std::cout << "[TMPCPlannerPlugin]     - Road width: " << 4.0 << std::endl;
  }
  // Step 3: Load reference path
  double road_width = 4.0;  // TODO: Get from configuration
  global_guidance_->LoadReferencePath(
      std::max(0.0, spline_position), reference_spline_, road_width);

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin]   Step 4: Converting obstacles..." << std::endl;
  }

  // Step 4: Convert obstacles to guidance planner format
  std::vector<GuidancePlanner::Obstacle> obstacles;
  obstacles.reserve(data.dynamic_obstacles.size());

  for (const auto& obs : data.dynamic_obstacles) {
    if (obs.index < 0) continue;  // Skip dummy obstacles

    std::vector<Eigen::Vector2d> positions;
    positions.reserve(1 + (obs.prediction.modes.empty() ? 0 : obs.prediction.modes.front().size()));
    positions.push_back(obs.position);

    if (!obs.prediction.modes.empty()) {
      for (const auto& step : obs.prediction.modes.front()) {
        positions.push_back(step.position);
      }
    }

    // Validate positions
    if (positions.size() < 2) {
      if (verbose_) {
        std::cerr << "[TMPCPlannerPlugin] Warning: Obstacle " << obs.index
                  << " has only " << positions.size() << " positions, skipping" << std::endl;
      }
      continue;
    }

    // Check for NaN or Inf
    bool valid = true;
    for (const auto& pos : positions) {
      if (!std::isfinite(pos.x()) || !std::isfinite(pos.y())) {
        if (verbose_) {
          std::cerr << "[TMPCPlannerPlugin] Warning: Obstacle " << obs.index
                    << " has invalid position (" << pos.x() << ", " << pos.y() << "), skipping" << std::endl;
        }
        valid = false;
        break;
      }
    }
    if (!valid) continue;

    obstacles.emplace_back(obs.index, positions, obs.radius + robot_radius);
  }

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin]   Step 5: Defining static obstacles..." << std::endl;
  }
  // Step 5: Define static obstacles (road boundaries)
  std::vector<GuidancePlanner::Halfspace> static_obstacles;
  // Add road boundaries if needed
  // static_obstacles.emplace_back(Eigen::Vector2d(0., 1.), 10.);   // y <= 10
  // static_obstacles.emplace_back(Eigen::Vector2d(0., -1.), 10.);  // y >= -10

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin]   Step 6: Loading obstacles (" << obstacles.size() << " dynamic, "
              << static_obstacles.size() << " static)..." << std::endl;
    for (size_t i = 0; i < obstacles.size(); ++i) {
      const auto& obs = obstacles[i];
      std::cout << "[TMPCPlannerPlugin]     Obstacle " << i << ": id=" << obs.id_
                << ", positions=" << obs.positions_.size()
                << ", radius=" << obs.radius_ << std::endl;
      if (!obs.positions_.empty()) {
        std::cout << "       First pos: (" << obs.positions_[0].x() << ", " << obs.positions_[0].y() << ")" << std::endl;
      }
    }
  }
  // Step 6: Load obstacles
  global_guidance_->LoadObstacles(obstacles, static_obstacles);

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin]   Step 7: Calling Update()..." << std::endl;
  }

  // Update guidance planner
  try {
    global_guidance_->Update();
    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin] Guidance planner updated successfully" << std::endl;
    }
  } catch (const std::exception& e) {
    if (verbose_) {
      std::cerr << "[TMPCPlannerPlugin] Guidance planner update failed: "
                << e.what() << std::endl;
    }
    // Continue even if guidance planner fails - MPC can still work without it
  }
}

void TMPCPlannerPlugin::extractDebugPaths(const MPCPlanner::State& state,
                                          const MPCPlanner::RealTimeData& data,
                                          navsim::plugin::PlanningResult& result) {
  // Use static variable to store debug paths (same as JPS planner)
  static std::vector<std::vector<navsim::planning::Pose2d>> global_debug_paths;
  static std::vector<std::string> global_debug_path_types;  // Store path types

  // Store approximation circles for obstacles
  struct ApproximationCircle {
    double x, y, radius;
  };
  static std::vector<ApproximationCircle> global_approximation_circles;

  global_debug_paths.clear();
  global_debug_path_types.clear();
  global_approximation_circles.clear();

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Extracting debug paths for visualization..." << std::endl;
  }

  // Path 1: Reference path
  if (!data.reference_path.x.empty()) {
    std::vector<navsim::planning::Pose2d> ref_path;
    for (size_t i = 0; i < data.reference_path.x.size(); ++i) {
      navsim::planning::Pose2d pose;
      pose.x = data.reference_path.x[i];
      pose.y = data.reference_path.y[i];
      pose.yaw = data.reference_path.psi[i];
      ref_path.push_back(pose);
    }
    global_debug_paths.push_back(ref_path);
    global_debug_path_types.push_back("reference");
    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin] === PATH 1: Reference path size: " << ref_path.size() << std::endl;
    }
  }

  // Path 2-N: Guidance planner trajectories (dashed lines)
  int guidance_count = 0;
  if (global_guidance_ && global_guidance_->NumberOfGuidanceTrajectories() > 0) {
    int selected_topology = -1;
    try {
      auto& selected = global_guidance_->GetUsedTrajectory();
      selected_topology = selected.topology_class;
    } catch (...) {
      selected_topology = -1;
    }

    for (int i = 0; i < global_guidance_->NumberOfGuidanceTrajectories(); ++i) {
      auto& traj = global_guidance_->GetGuidanceTrajectory(i);
      RosTools::Spline2D& trajectory_spline = traj.spline.GetTrajectory();

      int samples = 80;
      std::vector<navsim::planning::Pose2d> guidance_path;
      guidance_path.reserve(samples + 1);
      double end_param = trajectory_spline.parameterLength();

      for (int k = 0; k <= samples; ++k) {
        double t = (samples == 0) ? 0.0 : end_param * static_cast<double>(k) / static_cast<double>(samples);
        Eigen::Vector2d point = trajectory_spline.getPoint(t);
        navsim::planning::Pose2d pose;
        pose.x = point.x();
        pose.y = point.y();
        pose.yaw = 0.0;
        guidance_path.push_back(pose);
      }

      global_debug_paths.push_back(guidance_path);
      global_debug_path_types.push_back("guidance");
      guidance_count++;
      if (verbose_) {
        std::cout << "[TMPCPlannerPlugin] === PATH " << (1 + guidance_count)
                  << ": Guidance trajectory " << i
                  << (traj.topology_class == selected_topology ? " (SELECTED)" : "")
                  << " size: " << guidance_path.size() << std::endl;
      }
    }
  }

  // Path N+1-M: MPC candidate trajectories (solid lines)
  int candidate_count = 0;
  auto candidates = planner_->getTMPCandidates();
  if (!candidates.empty()) {
    for (const auto& candidate : candidates) {
      if (!candidate.success || candidate.traj.positions.empty()) {
        continue;
      }

      std::vector<navsim::planning::Pose2d> candidate_path;
      candidate_path.reserve(candidate.traj.positions.size());
      for (const auto& pos : candidate.traj.positions) {
        navsim::planning::Pose2d pose;
        pose.x = pos.x();
        pose.y = pos.y();
        pose.yaw = 0.0;
        candidate_path.push_back(pose);
      }

      global_debug_paths.push_back(candidate_path);
      global_debug_path_types.push_back("mpc_candidate");
      candidate_count++;
      if (verbose_) {
        std::cout << "[TMPCPlannerPlugin] === PATH " << (1 + guidance_count + candidate_count)
                  << ": MPC candidate " << candidate.guidance_id
                  << (candidate.is_best ? " (BEST)" : "")
                  << (candidate.is_original ? " (ORIGINAL)" : "")
                  << " size: " << candidate_path.size() << std::endl;
      }
    }
  }

  // Path M+1-K: Obstacle predictions
  int pred_count = 0;
  for (size_t obs_idx = 0; obs_idx < data.dynamic_obstacles.size(); ++obs_idx) {
    const auto& obs = data.dynamic_obstacles[obs_idx];
    if (obs.index < 0) continue;  // Skip dummy obstacles

    // Store approximation circle for this obstacle
    ApproximationCircle circle;
    circle.x = obs.position.x();
    circle.y = obs.position.y();
    circle.radius = obs.radius;
    global_approximation_circles.push_back(circle);

    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin]   Approximation circle " << global_approximation_circles.size() - 1
                << ": pos=(" << circle.x << ", " << circle.y << "), radius=" << circle.radius << std::endl;
    }

    if (!obs.prediction.modes.empty() && !obs.prediction.modes[0].empty()) {
      std::vector<navsim::planning::Pose2d> pred_path;

      // Add current position
      navsim::planning::Pose2d current_pose;
      current_pose.x = obs.position.x();
      current_pose.y = obs.position.y();
      current_pose.yaw = obs.angle;
      pred_path.push_back(current_pose);

      // Add predicted positions
      for (const auto& step : obs.prediction.modes[0]) {
        navsim::planning::Pose2d pose;
        pose.x = step.position.x();
        pose.y = step.position.y();
        pose.yaw = step.angle;
        pred_path.push_back(pose);
      }

      global_debug_paths.push_back(pred_path);
      global_debug_path_types.push_back("obstacle_prediction");
      pred_count++;
      if (verbose_) {
        std::cout << "[TMPCPlannerPlugin] === PATH " << (1 + guidance_count + candidate_count + pred_count)
                  << ": Obstacle " << obs.index << " prediction size: " << pred_path.size() << std::endl;
      }
    }
  }

  // Path K+1: Past trajectory
  if (!data.past_trajectory.positions.empty()) {
    std::vector<navsim::planning::Pose2d> past_path;
    for (const auto& pos : data.past_trajectory.positions) {
      navsim::planning::Pose2d pose;
      pose.x = pos.x();
      pose.y = pos.y();
      pose.yaw = 0.0;
      past_path.push_back(pose);
    }
    global_debug_paths.push_back(past_path);
    global_debug_path_types.push_back("past_trajectory");
    if (verbose_) {
      std::cout << "[TMPCPlannerPlugin] === PATH " << (1 + guidance_count + candidate_count + pred_count + 1)
                << ": Past trajectory size: " << past_path.size() << std::endl;
    }
  }

  // Store debug paths in result metadata
  result.metadata["has_debug_paths"] = 1.0;
  result.metadata["debug_paths_ptr"] = static_cast<double>(
      reinterpret_cast<uintptr_t>(&global_debug_paths));
  result.metadata["debug_path_types_ptr"] = static_cast<double>(
      reinterpret_cast<uintptr_t>(&global_debug_path_types));
  result.metadata["approximation_circles_ptr"] = static_cast<double>(
      reinterpret_cast<uintptr_t>(&global_approximation_circles));
  result.metadata["approximation_circles_count"] = static_cast<double>(global_approximation_circles.size());

  if (verbose_) {
    std::cout << "[TMPCPlannerPlugin] Total debug paths: " << global_debug_paths.size() << std::endl;
    std::cout << "[TMPCPlannerPlugin] Total approximation circles: " << global_approximation_circles.size() << std::endl;
  }
}

} // namespace adapter
} // namespace tmpc_planner


/**
 * @file jps_planner.hpp
 * @brief JPS Planner - Core algorithm layer
 * 
 * Migrated from ROS-based implementation to NavSim plugin system.
 * This is the pure algorithm layer, independent of plugin system.
 */

#ifndef JPS_PLANNER_HPP
#define JPS_PLANNER_HPP

#include "esdf_map.hpp"
#include "graph_search.hpp"
#include "jps_data_structures.hpp"
#include <Eigen/Eigen>
#include <memory>
#include <vector>

namespace JPS {

/**
 * @brief JPSPlanner class - Core algorithm implementation
 * 
 * This class implements the JPS planning algorithm including:
 * - Path planning using GraphSearch
 * - Path optimization (corner removal)
 * - Trajectory generation
 * - Time planning with trapezoidal velocity profile
 * 
 * This is a pure algorithm class that does not depend on:
 * - Plugin system
 * - PlanningContext
 * - ROS
 */
class JPSPlanner {
 public:
  /**
   * @brief Constructor
   * @param map ESDFMap pointer
   */
  JPSPlanner(std::shared_ptr<navsim::perception::ESDFMap> map);

  /**
   * @brief Plan from start to goal
   * @param start Start state (x, y, yaw)
   * @param goal Goal state (x, y, yaw)
   * @return true if planning succeeded
   */
  bool plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal);

  /**
   * @brief Set configuration
   * @param config JPSConfig structure
   */
  void setConfig(const JPSConfig& config);

  /**
   * @brief Get configuration
   * @return const reference to JPSConfig
   */
  const JPSConfig& getConfig() const { return config_; }

  /**
   * @brief Get raw path (from JPS search)
   * @return const reference to raw path
   */
  const std::vector<Eigen::Vector2d>& getRawPath() const { return raw_path_; }

  /**
   * @brief Get optimized path (after corner removal)
   * @return const reference to optimized path
   */
  const std::vector<Eigen::Vector2d>& getOptimizedPath() const { return path_; }

  /**
   * @brief Get flat trajectory data
   * @return const reference to FlatTrajData
   */
  const FlatTrajData& getFlatTraj() const { return flat_traj_; }

  /**
   * @brief Get planning status
   * @return status code (-1 = failed, 0 = success)
   */
  int getStatus() const { return status_; }

  // Public member for trajectory data (kept for compatibility)
  FlatTrajData flat_traj_;

 private:
  // ========== Core Functions ==========

  /**
   * @brief Get small resolution path
   */
  void get_small_resolution_path_();

  /**
   * @brief Remove corner points from path
   * @param path Input path
   * @return Optimized path
   */
  std::vector<Eigen::Vector2d> removeCornerPts(const std::vector<Eigen::Vector2d>& path);

  /**
   * @brief Check line collision
   * @param start Start point
   * @param end End point
   * @return true if collision detected
   */
  bool checkLineCollision(const Eigen::Vector2d& start, const Eigen::Vector2d& end);

  /**
   * @brief Bresenham line algorithm
   * @param start Start grid index
   * @param end End grid index
   * @return Vector of grid indices along the line
   */
  std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i& start,
                                                        const Eigen::Vector2i& end);

  /**
   * @brief Get kinodynamic node with start path
   * @param start_path Start path
   * @param if_forward Forward flag
   * @param current_state_VAJ Current state VAJ
   * @param current_state_OAJ Current state OAJ
   */
  void getKinoNodeWithStartPath(const std::vector<Eigen::Vector3d>& start_path,
                                 const bool if_forward,
                                 const Eigen::Vector3d& current_state_VAJ,
                                 const Eigen::Vector3d& current_state_OAJ);

  /**
   * @brief Get sample trajectory
   */
  void getSampleTraj();

  /**
   * @brief Get trajectories with time
   */
  void getTrajsWithTime();

  /**
   * @brief Normalize angle
   * @param ref_angle Reference angle
   * @param angle Angle to normalize
   */
  void normalizeAngle(const double& ref_angle, double& angle);

  /**
   * @brief Evaluate duration using trapezoidal velocity profile
   * @param length Path length
   * @param startV Start velocity
   * @param endV End velocity
   * @param maxV Max velocity
   * @param maxA Max acceleration
   * @return Duration
   */
  double evaluateDuration(const double& length, const double& startV, const double& endV,
                          const double& maxV, const double& maxA);

  /**
   * @brief Evaluate length at current time
   * @param curt Current time
   * @param locallength Local length
   * @param localtime Local time
   * @param startV Start velocity
   * @param endV End velocity
   * @param maxV Max velocity
   * @param maxA Max acceleration
   * @return Length
   */
  double evaluateLength(const double& curt, const double& locallength, const double& localtime,
                        const double& startV, const double& endV, const double& maxV,
                        const double& maxA);

  /**
   * @brief Evaluate velocity at current time
   * @param curt Current time
   * @param locallength Local length
   * @param localtime Local time
   * @param startV Start velocity
   * @param endV End velocity
   * @param maxV Max velocity
   * @param maxA Max acceleration
   * @return Velocity
   */
  double evaluateVel(const double& curt, const double& locallength, const double& localtime,
                     const double& startV, const double& endV, const double& maxV,
                     const double& maxA);

  /**
   * @brief Evaluate time of position
   * @param pos Position
   * @param locallength Local length
   * @param startV Start velocity
   * @param endV End velocity
   * @param maxV Max velocity
   * @param maxA Max acceleration
   * @return Time
   */
  double evaluteTimeOfPos(const double& pos, const double& locallength, const double& startV,
                          const double& endV, const double& maxV, const double& maxA);

  /**
   * @brief JPS collision check
   * @param pos Position
   * @return true if collision-free
   */
  bool JPS_check_if_collision(const Eigen::Vector2d& pos);

  // ========== Member Variables ==========

  // Configuration
  JPSConfig config_;

  // Data
  Eigen::Vector3d start_state_;        // x y yaw
  Eigen::Vector3d current_state_VAJ_;  // velocity, acceleration, jerk
  Eigen::Vector3d current_state_OAJ_;  // omega, alpha, jerk
  Eigen::Vector3d end_state_;          // x y yaw

  bool if_first_point_cut_;

  // Core objects
  std::shared_ptr<navsim::perception::ESDFMap> map_util_;
  std::shared_ptr<GraphSearch> graph_search_;

  // Status
  int status_;

  // Path data
  std::vector<Eigen::Vector2d> raw_path_;
  std::vector<Eigen::Vector2d> path_;
  std::vector<Eigen::Vector2d> Unoccupied_path_;
  std::vector<Eigen::VectorXd> Unoccupied_sample_trajs_;      // x y theta dtheta ds
  std::vector<Eigen::VectorXd> cut_Unoccupied_sample_trajs_;  // x y theta dtheta ds
  std::vector<Eigen::Vector2i> small_resolution_path_;
};

}  // namespace JPS

#endif  // JPS_PLANNER_HPP


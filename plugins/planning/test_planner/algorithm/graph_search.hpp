/**
 * @file graph_search.hpp
 * @brief Backend of graph search, implementation of A* and JPS
 * 
 * Migrated from ROS-based implementation to NavSim plugin system.
 * Core algorithm remains unchanged.
 */

#ifndef JPS_GRAPH_SEARCH_HPP
#define JPS_GRAPH_SEARCH_HPP

#include "jps_data_structures.hpp"
#include "esdf_map.hpp"
#include <memory>
#include <vector>
#include <cmath>

namespace JPS {

/**
 * @brief GraphSearch class
 *
 * Implement A* and Jump Point Search
 */
class GraphSearch {
 public:
  /**
   * @brief Constructor with ESDFMap
   *
   * @param Map ESDFMap pointer
   * @param safe_dis Safe distance for collision checking
   */
  GraphSearch(std::shared_ptr<navsim::perception::ESDFMap> Map, const double& safe_dis);

  /**
   * @brief Start 2D planning thread
   *
   * @param xStart start x coordinate
   * @param yStart start y coordinate
   * @param xGoal goal x coordinate
   * @param yGoal goal y coordinate
   * @param useJps if true, enable JPS search; else the planner is implementing A*
   * @param maxExpand maximum number of expansion allowed, optional, default is -1, means no limitation
   */
  bool plan(int xStart, int yStart, int xGoal, int yGoal, bool useJps, int maxExpand = -1);

  /**
   * @brief Start 3D planning thread (currently not implemented)
   *
   * @param xStart start x coordinate
   * @param yStart start y coordinate
   * @param zStart start z coordinate
   * @param xGoal goal x coordinate
   * @param yGoal goal y coordinate
   * @param zGoal goal z coordinate
   * @param useJps if true, enable JPS search; else the planner is implementing A*
   * @param maxExpand maximum number of expansion allowed, optional, default is -1, means no limitation
   */
  bool plan(int xStart, int yStart, int zStart, int xGoal, int yGoal, int zGoal, bool useJps, int maxExpand = -1);

  /// Get the optimal path
  std::vector<StatePtr> getPath() const;

  /// Get the states in open set
  std::vector<StatePtr> getOpenSet() const;

  /// Get the states in close set
  std::vector<StatePtr> getCloseSet() const;

  /// Get the states in hash map
  std::vector<StatePtr> getAllSet() const;

  /// Set Safe Distance
  void SetSafeDis(const double& safe_dis);
  
  /// Get Safe Distance
  double GetSafeDis();

 private:
  /// Main planning loop
  bool plan(StatePtr& currNode_ptr, int max_expand, int start_id, int goal_id);
  
  /// Get successor function for A*
  void getSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
  
  /// Get successor function for JPS
  void getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs);
  
  /// Recover the optimal path
  std::vector<StatePtr> recoverPath(StatePtr node, int id);

  /// Get subscript
  int coordToId(int x, int y) const;

  /// Check if (x, y) is free
  bool isFree(int x, int y) const;

  /// Check if (x, y) is unoccupied
  bool isUnoccupied(int x, int y) const;

  /// Check if (x, y) is occupied
  bool isOccupied(int x, int y) const;

  /// Calculate heuristic
  double getHeur(int x, int y) const;

  /// Determine if (x, y) has forced neighbor with direction (dx, dy)
  bool hasForced(int x, int y, int dx, int dy);

  /// 2D jump, return true iff finding the goal or a jump point
  bool jump(int x, int y, int dx, int dy, int& new_x, int& new_y);

  /// Initialize 2D jps arrays
  void init2DJps();

  // ESDFMap pointer (replaces SDFmap)
  std::shared_ptr<navsim::perception::ESDFMap> map_;
  
  int xDim_, yDim_, zDim_;
  double eps_;
  bool verbose_;

  double safe_dis_;

  const char val_free_ = 0;
  int xGoal_, yGoal_, zGoal_;
  bool use_2d_;
  bool use_jps_ = false;

  priorityQueue pq_;
  std::vector<StatePtr> hm_;
  std::vector<bool> seen_;

  std::vector<StatePtr> path_;

  std::vector<std::vector<int>> ns_;
  std::shared_ptr<JPS2DNeib> jn2d_;
  std::shared_ptr<JPS3DNeib> jn3d_;
};

}  // namespace JPS

#endif  // JPS_GRAPH_SEARCH_HPP


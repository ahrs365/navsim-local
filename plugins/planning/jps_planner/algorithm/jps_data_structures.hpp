/**
 * @file jps_data_structures.hpp
 * @brief Data structures for JPS planner
 * 
 * This file contains all data structures used by the JPS planner,
 * migrated from the original ROS-based implementation.
 */

#ifndef JPS_DATA_STRUCTURES_HPP
#define JPS_DATA_STRUCTURES_HPP

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/heap/d_ary_heap.hpp>
#include <memory>
#include <limits>
#include <vector>
#include <iostream>

namespace JPS {

// ============================================================================
// Forward Declarations
// ============================================================================

struct State;
using StatePtr = std::shared_ptr<State>;

// ============================================================================
// Heap Element Comparison
// ============================================================================

/**
 * @brief Comparison functor for priority queue
 */
template <class T>
struct compare_state {
  bool operator()(T a1, T a2) const {
    double f1 = a1->g + a1->h;
    double f2 = a2->g + a2->h;
    if ((f1 >= f2 - 0.000001) && (f1 <= f2 + 0.000001))
      return a1->g < a2->g;  // if equal compare gvals
    return f1 > f2;
  }
};

// ============================================================================
// Priority Queue Definition
// ============================================================================

using priorityQueue = boost::heap::d_ary_heap<StatePtr, 
                                               boost::heap::mutable_<true>,
                                               boost::heap::arity<2>, 
                                               boost::heap::compare<compare_state<StatePtr>>>;

// ============================================================================
// State - Node of the graph in graph search
// ============================================================================

struct State {
  /// ID
  int id;
  /// Coord
  int x, y, z = 0;
  /// direction
  int dx, dy, dz;
  /// id of predecessor
  int parentId = -1;

  /// pointer to heap location
  priorityQueue::handle_type heapkey;

  /// g cost
  double g = std::numeric_limits<double>::infinity();
  /// heuristic cost
  double h;
  /// if has been opened
  bool opened = false;
  /// if has been closed
  bool closed = false;

  /// 2D constructor
  State(int id, int x, int y, int dx, int dy)
      : id(id), x(x), y(y), dx(dx), dy(dy) {}

  /// 3D constructor
  State(int id, int x, int y, int z, int dx, int dy, int dz)
      : id(id), x(x), y(y), z(z), dx(dx), dy(dy), dz(dz) {}
};

// ============================================================================
// JPS2DNeib - Search and prune neighbors for JPS 2D
// ============================================================================

struct JPS2DNeib {
  // for each (dx,dy) these contain:
  //    ns: neighbors that are always added
  //    f1: forced neighbors to check
  //    f2: neighbors to add if f1 is forced
  int ns[9][2][8];
  int f1[9][2][2];
  int f2[9][2][2];
  
  // nsz contains the number of neighbors for the four different types of moves:
  // no move (norm 0):        8 neighbors always added
  //                          0 forced neighbors to check (never happens)
  //                          0 neighbors to add if forced (never happens)
  // straight (norm 1):       1 neighbor always added
  //                          2 forced neighbors to check
  //                          2 neighbors to add if forced
  // diagonal (norm sqrt(2)): 3 neighbors always added
  //                          2 forced neighbors to check
  //                          2 neighbors to add if forced
  static constexpr int nsz[3][2] = {{8, 0}, {1, 2}, {3, 2}};

  void print();
  JPS2DNeib();

 private:
  void Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty);
  void FNeib(int dx, int dy, int norm1, int dev, int& fx, int& fy, int& nx, int& ny);
};

// ============================================================================
// JPS3DNeib - Search and prune neighbors for JPS 3D
// ============================================================================

struct JPS3DNeib {
  // for each (dx,dy,dz) these contain:
  //    ns: neighbors that are always added
  //    f1: forced neighbors to check
  //    f2: neighbors to add if f1 is forced
  int ns[27][3][26];
  int f1[27][3][12];
  int f2[27][3][12];
  
  // nsz contains the number of neighbors for the four different types of moves:
  // no move (norm 0):        26 neighbors always added
  //                          0 forced neighbors to check (never happens)
  //                          0 neighbors to add if forced (never happens)
  // straight (norm 1):       1 neighbor always added
  //                          8 forced neighbors to check
  //                          8 neighbors to add if forced
  // diagonal (norm sqrt(2)): 3 neighbors always added
  //                          8 forced neighbors to check
  //                          12 neighbors to add if forced
  // diagonal (norm sqrt(3)): 7 neighbors always added
  //                          6 forced neighbors to check
  //                          12 neighbors to add if forced
  static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
  
  JPS3DNeib();

 private:
  void Neib(int dx, int dy, int dz, int norm1, int dev, int& tx, int& ty, int& tz);
  void FNeib(int dx, int dy, int dz, int norm1, int dev,
             int& fx, int& fy, int& fz,
             int& nx, int& ny, int& nz);
};

// ============================================================================
// PathNode - Path node for kinodynamic planning (currently unused in JPS)
// ============================================================================

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector2i index;
  int yaw_idx;
  /* --- the state is x y theta(orientation) */
  Eigen::Vector3d state;
  double g_score, f_score;
  double penalty_score;
  /* control input should be steer and arc */
  Eigen::Vector2d input;
  PathNode* parent;
  // Three states: not expanded, in close set, in open set
  char node_state;
  
  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
};
typedef PathNode* PathNodePtr;

// ============================================================================
// FlatTrajData - Flat trajectory data
// ============================================================================

struct FlatTrajData {
  // All initial values after uniform sampling: yaw, s, t
  std::vector<Eigen::Vector3d> UnOccupied_traj_pts;
  double UnOccupied_initT;
  std::vector<Eigen::Vector3d> UnOccupied_positions;

  Eigen::MatrixXd start_state;   // pva
  Eigen::MatrixXd final_state;   // end flat state (2, 3)

  Eigen::Vector3d start_state_XYTheta;
  Eigen::Vector3d final_state_XYTheta;
  bool if_cut;

  void printFlatTrajData() {
    std::cout << "UnOccupied_traj_pts:" << std::endl;
    for (const auto& pt : UnOccupied_traj_pts) {
      std::cout << pt.transpose() << std::endl;
    }
    std::cout << "UnOccupied_initT: " << UnOccupied_initT << std::endl;

    std::cout << "start_state:" << std::endl;
    std::cout << start_state << std::endl;
    std::cout << "final_state:" << std::endl;
    std::cout << final_state << std::endl;
    std::cout << "start_state_XYTheta: " << start_state_XYTheta.transpose() << std::endl;
    std::cout << "final_state_XYTheta: " << final_state_XYTheta.transpose() << std::endl;
    std::cout << "if_cut: " << if_cut << std::endl;
  }
};

// ============================================================================
// OptimizerConfig - Configuration for trajectory optimizer
// ============================================================================

struct OptimizerConfig {
  // Kinematic constraints
  double max_vel = 5.0;
  double min_vel = -5.0;
  double max_acc = 5.0;
  double max_omega = 1.0;
  double max_domega = 50.0;
  double max_centripetal_acc = 10000.0;
  bool if_directly_constrain_v_omega = false;

  // Optimizer parameters
  double mean_time_lowBound = 0.5;
  double mean_time_uppBound = 2.0;
  double smoothEps = 1e-3;  // for smoothL1
  double safeDis = 0.3;

  // Final collision check parameters
  double finalMinSafeDis = 0.2;
  int finalSafeDisCheckNum = 10;
  int safeReplanMaxTime = 3;

  // Penalty weights
  double time_weight = 1.0;
  double acc_weight = 1.0;
  double domega_weight = 1.0;
  double collision_weight = 1.0;
  double moment_weight = 1.0;
  double mean_time_weight = 1.0;
  double cen_acc_weight = 1.0;

  // Path penalty weights (for pre-processing)
  double path_time_weight = 1.0;
  double path_bigpath_sdf_weight = 1.0;
  double path_moment_weight = 1.0;
  double path_mean_time_weight = 1.0;
  double path_acc_weight = 1.0;
  double path_domega_weight = 1.0;

  // Energy weights
  std::vector<double> energyWeights = {1.0, 1.0};

  // Augmented Lagrangian parameters
  std::vector<double> EqualLambda = {0.0, 0.0};
  std::vector<double> EqualRho = {1.0, 1.0};
  std::vector<double> EqualRhoMax = {100.0, 100.0};
  std::vector<double> EqualGamma = {0.1, 0.1};
  std::vector<double> EqualTolerance = {0.01, 0.01};

  // Cut trajectory Augmented Lagrangian parameters
  std::vector<double> CutEqualLambda = {0.0, 0.0};
  std::vector<double> CutEqualRho = {1.0, 1.0};
  std::vector<double> CutEqualRhoMax = {100.0, 100.0};
  std::vector<double> CutEqualGamma = {0.1, 0.1};
  std::vector<double> CutEqualTolerance = {0.01, 0.01};

  // LBFGS parameters for path pre-processing
  int path_lbfgs_mem_size = 16;
  int path_lbfgs_past = 3;
  double path_lbfgs_g_epsilon = 1e-3;
  double path_lbfgs_min_step = 1e-32;
  double path_lbfgs_delta = 1e-4;
  int path_lbfgs_max_iterations = 200;
  double path_lbfgs_shot_path_past = 1.0;
  double path_lbfgs_shot_path_horizon = 2.0;

  // LBFGS parameters for main optimization
  int lbfgs_mem_size = 16;
  int lbfgs_past = 3;
  double lbfgs_g_epsilon = 1e-3;
  double lbfgs_min_step = 1e-32;
  double lbfgs_delta = 1e-4;
  int lbfgs_max_iterations = 200;

  // Sampling parameters
  int sparseResolution = 10;
  double timeResolution = 0.1;
  int mintrajNum = 10;

  // Trajectory prediction resolution
  double trajPredictResolution = 0.1;

  // Visualization
  bool if_visual_optimization = false;

  // Horizon limitation
  bool hrz_limited = false;
  double hrz_laser_range_dgr = 180.0;

  // Checkpoint for collision detection (robot footprint)
  std::vector<Eigen::Vector2d> checkpoint = {
    Eigen::Vector2d(0.0, 0.0)
  };

  // ICR parameters (Instantaneous Center of Rotation)
  Eigen::Vector3d ICR = Eigen::Vector3d(0.0, 0.0, 1.0);  // yl, yr, xv
  bool if_standard_diff = true;
};

// ============================================================================
// JPSConfig - Configuration for JPS planner
// ============================================================================

struct JPSConfig {
  // Safety distance
  double safe_dis = 0.3;
  double max_jps_dis = 10.0;

  // Weights
  double distance_weight = 1.0;
  double yaw_weight = 1.0;

  // Trajectory parameters
  double traj_cut_length = 5.0;

  // Kinematic constraints
  double max_vel = 1.0;
  double max_acc = 1.0;
  double max_omega = 1.0;
  double max_domega = 1.0;

  // Sampling parameters
  double sample_time = 0.1;
  int min_traj_num = 10;

  // JPS parameters
  double jps_truncation_time = 5.0;

  // Optimizer configuration
  OptimizerConfig optimizer;
};

}  // namespace JPS

#endif  // JPS_DATA_STRUCTURES_HPP


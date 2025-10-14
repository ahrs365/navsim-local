#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <Eigen/Dense>

namespace navsim {
namespace planning {

// ========== 基础数据类型 ==========

struct Point2d {
  double x = 0.0;
  double y = 0.0;

  Point2d() = default;
  Point2d(double x_, double y_) : x(x_), y(y_) {}

  // 支持大括号初始化
  Point2d(const std::initializer_list<double>& init) {
    auto it = init.begin();
    if (it != init.end()) x = *it++;
    if (it != init.end()) y = *it++;
  }
};

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;  // 朝向角 (rad)

  Pose2d() = default;
  Pose2d(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}

  // 支持大括号初始化
  Pose2d(const std::initializer_list<double>& init) {
    auto it = init.begin();
    if (it != init.end()) x = *it++;
    if (it != init.end()) y = *it++;
    if (it != init.end()) yaw = *it++;
  }
};

struct Twist2d {
  double vx = 0.0;    // 纵向速度 (m/s)
  double vy = 0.0;    // 横向速度 (m/s)
  double omega = 0.0; // 角速度 (rad/s)

  Twist2d() = default;
  Twist2d(double vx_, double vy_, double omega_) : vx(vx_), vy(vy_), omega(omega_) {}

  // 支持大括号初始化
  Twist2d(const std::initializer_list<double>& init) {
    auto it = init.begin();
    if (it != init.end()) vx = *it++;
    if (it != init.end()) vy = *it++;
    if (it != init.end()) omega = *it++;
  }
};

// ========== 地图表示 ==========

/**
 * @brief 栅格占据地图
 * 适用于：A*、Hybrid A*、RRT* 等基于搜索的规划器
 */
struct OccupancyGrid {
  struct Config {
    Point2d origin;      // 地图原点 (m)
    double resolution;   // 栅格分辨率 (m/cell)
    int width;           // 宽度 (cells)
    int height;          // 高度 (cells)
  } config;

  std::vector<uint8_t> data;  // 占据概率 [0,100], 255=未知

  // 工具函数
  bool isOccupied(int x, int y, uint8_t threshold = 50) const;
  Point2d cellToWorld(int x, int y) const;
  std::pair<int, int> worldToCell(const Point2d& point) const;
};

/**
 * @brief BEV (Bird's Eye View) 障碍物表示
 * 适用于：优化轨迹规划、深度学习规划器
 */
struct BEVObstacles {
  struct Circle {
    Point2d center;
    double radius;
    double confidence = 1.0;  // 检测置信度
  };

  struct Rectangle {
    Pose2d pose;        // 中心位置和朝向
    double width;       // 宽度 (m)
    double height;      // 长度 (m)
    double confidence = 1.0;
  };

  struct Polygon {
    std::vector<Point2d> vertices;
    double confidence = 1.0;
  };

  std::vector<Circle> circles;
  std::vector<Rectangle> rectangles;
  std::vector<Polygon> polygons;
};

/**
 * @brief 车道线表示
 * 适用于：车道保持、换道决策
 */
struct LaneLines {
  struct Lane {
    std::vector<Point2d> centerline;   // 车道中心线
    std::vector<Point2d> left_bound;   // 左边界
    std::vector<Point2d> right_bound;  // 右边界
    double width = 3.5;                // 车道宽度 (m)
    int lane_id = -1;                  // 车道ID
  };

  std::vector<Lane> lanes;
  int ego_lane_id = -1;  // 自车所在车道ID
};

// ========== 动态障碍物 ==========

/**
 * @brief 动态障碍物预测轨迹
 * 适用于：交互感知规划、博弈论规划
 */
struct DynamicObstacle {
  int id;
  std::string type;  // "vehicle", "pedestrian", "cyclist"
  std::string shape_type;  // "circle" or "rectangle" - 🔧 新增：形状类型

  // 当前状态
  Pose2d current_pose;
  Twist2d current_twist;

  // 预测轨迹 (多种可能性)
  struct Trajectory {
    std::vector<Pose2d> poses;
    std::vector<double> timestamps;  // 对应时间戳
    double probability = 1.0;        // 轨迹概率
  };
  std::vector<Trajectory> predicted_trajectories;

  // 包围盒
  double length = 4.5;
  double width = 2.0;
  double height = 1.8;
};

// ========== 车辆状态 ==========

/**
 * @brief 自车状态和约束
 */
struct EgoVehicle {
  // 当前状态
  Pose2d pose;
  Twist2d twist;
  double timestamp = 0.0;

  // 车辆参数
  struct Kinematics {
    double wheelbase = 2.8;       // 轴距 (m)
    double front_overhang = 1.0;  // 前悬 (m)
    double rear_overhang = 1.0;   // 后悬 (m)
    double width = 2.0;           // 车宽 (m)
    double height = 1.8;          // 车高 (m)
  } kinematics;

  // 动力学约束
  struct Limits {
    double max_velocity = 15.0;      // 最大速度 (m/s)
    double max_acceleration = 3.0;   // 最大加速度 (m/s²)
    double max_deceleration = 8.0;   // 最大减速度 (m/s²)
    double max_steer_angle = 0.6;    // 最大转向角 (rad)
    double max_steer_rate = 1.0;     // 最大转向角速度 (rad/s)
    double max_jerk = 3.0;           // 最大急动度 (m/s³)
    double max_curvature = 0.2;      // 最大曲率 (1/m)
  } limits;
};

// ========== 任务目标 ==========

/**
 * @brief 规划任务定义
 */
struct PlanningTask {
  // 目标点
  Pose2d goal_pose;

  // 容差
  struct Tolerance {
    double position = 0.5;  // 位置容差 (m)
    double yaw = 0.2;       // 朝向容差 (rad)
  } tolerance;

  // 任务类型
  enum class Type {
    GOTO_GOAL,      // 点到点导航
    LANE_FOLLOWING, // 车道保持
    LANE_CHANGE,    // 换道
    PARKING,        // 泊车
    EMERGENCY_STOP  // 紧急停车
  } type = Type::GOTO_GOAL;

  // 任务参数
  std::unordered_map<std::string, double> parameters;
};

// ========== 规划上下文 ==========

/**
 * @brief 规划算法的统一输入接口
 * 支持多种规划器，每个规划器只使用自己需要的数据
 */
struct PlanningContext {
  // 必需数据 (所有规划器都需要)
  EgoVehicle ego;
  PlanningTask task;
  double planning_horizon = 5.0;  // 规划时域 (s)
  double timestamp = 0.0;         // 当前时间戳

  // 可选数据 (根据规划器需求选择性填充)
  std::unique_ptr<OccupancyGrid> occupancy_grid;        // 栅格地图
  std::unique_ptr<BEVObstacles> bev_obstacles;          // BEV障碍物
  std::unique_ptr<LaneLines> lane_lines;                // 车道线
  std::vector<DynamicObstacle> dynamic_obstacles;       // 动态障碍物

  // 规划器特定数据
  std::unordered_map<std::string, std::shared_ptr<void>> custom_data;

  // 工具函数
  template<typename T>
  void setCustomData(const std::string& key, std::shared_ptr<T> data) {
    custom_data[key] = std::static_pointer_cast<void>(data);
  }

  template<typename T>
  std::shared_ptr<T> getCustomData(const std::string& key) const {
    auto it = custom_data.find(key);
    if (it != custom_data.end()) {
      return std::static_pointer_cast<T>(it->second);
    }
    return nullptr;
  }

  bool hasCustomData(const std::string& key) const {
    return custom_data.find(key) != custom_data.end();
  }

  void clearCustomData() {
    custom_data.clear();
  }
};

} // namespace planning
} // namespace navsim
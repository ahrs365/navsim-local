#pragma once

#include "planning_context.hpp"
#include "websocket_visualizer.hpp"
#include "plan_update.pb.h"
#include "ego_cmd.pb.h"
#include <memory>
#include <chrono>
#include <string>
#include <unordered_map>

namespace navsim {

// 前向声明
class Bridge;

namespace planning {

/**
 * @brief 轨迹点结构
 */
struct TrajectoryPoint {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;      // 期望速度 (m/s)
  double a = 0.0;      // 期望加速度 (m/s²)
  double kappa = 0.0;  // 曲率 (1/m)
  double t = 0.0;      // 时间戳 (s)
  double s = 0.0;      // 弧长 (m)
};

/**
 * @brief 规划结果
 */
struct PlanningResult {
  // 轨迹
  std::vector<TrajectoryPoint> trajectory;

  // 控制指令 (可选)
  struct ControlCommand {
    double acceleration = 0.0;  // 加速度指令 (m/s²)
    double steering = 0.0;      // 转向角指令 (rad)
    double velocity = 0.0;      // 速度指令 (m/s) [可选]
  } control_cmd;

  // 规划状态
  enum class Status {
    SUCCESS,          // 成功
    FAILED,           // 失败
    NO_SOLUTION,      // 无解
    TIMEOUT,          // 超时
    INVALID_INPUT     // 输入无效
  } status = Status::SUCCESS;

  // 代价信息
  struct Cost {
    double total = 0.0;
    double smoothness = 0.0;
    double obstacle = 0.0;
    double goal = 0.0;
    double comfort = 0.0;
    double efficiency = 0.0;
  } cost;

  // 诊断信息
  struct Diagnostics {
    int iterations = 0;
    bool converged = false;
    double computation_time_ms = 0.0;
    std::string failure_reason;
  } diagnostics;
};

/**
 * @brief 规划器接口基类
 * 所有规划算法都需要实现这个接口
 */
class PlannerInterface {
public:
  virtual ~PlannerInterface() = default;

  /**
   * @brief 规划主函数
   * @param context 规划上下文 (输入)
   * @param deadline 规划截止时间
   * @param result 规划结果 (输出)
   * @return 规划是否成功
   */
  virtual bool plan(const PlanningContext& context,
                   std::chrono::milliseconds deadline,
                   PlanningResult& result) = 0;

  /**
   * @brief 获取规划器名称
   */
  virtual std::string getName() const = 0;

  /**
   * @brief 获取规划器类型
   */
  virtual std::string getType() const = 0;

  /**
   * @brief 检查规划器是否可用
   * @param context 规划上下文
   * @return 是否可用及原因
   */
  virtual std::pair<bool, std::string> isAvailable(const PlanningContext& context) const = 0;

  /**
   * @brief 重置规划器状态
   */
  virtual void reset() {}

  /**
   * @brief 获取参数
   */
  virtual std::unordered_map<std::string, double> getParameters() const { return {}; }

  /**
   * @brief 设置参数
   */
  virtual bool setParameter(const std::string& name, double value) { return false; }
};

/**
 * @brief 简单直线规划器 (当前实现的升级版)
 */
class StraightLinePlanner : public PlannerInterface {
public:
  struct Config {
    double time_step = 0.1;           // 时间步长 (s)
    double default_velocity = 2.0;    // 默认速度 (m/s)
    double max_acceleration = 2.0;    // 最大加速度 (m/s²)
    double arrival_tolerance = 0.5;   // 到达容差 (m)
  };

  StraightLinePlanner();
  explicit StraightLinePlanner(const Config& config);

  bool plan(const PlanningContext& context,
            std::chrono::milliseconds deadline,
            PlanningResult& result) override;

  std::string getName() const override { return "StraightLinePlanner"; }
  std::string getType() const override { return "geometric"; }

  std::pair<bool, std::string> isAvailable(const PlanningContext& context) const override;

  std::unordered_map<std::string, double> getParameters() const override;
  bool setParameter(const std::string& name, double value) override;

private:
  Config config_;
};

/**
 * @brief A*路径规划器
 */
class AStarPlanner : public PlannerInterface {
public:
  struct Config {
    double time_step = 0.1;
    double heuristic_weight = 1.0;    // 启发式权重
    double step_size = 0.5;           // 搜索步长 (m)
    int max_iterations = 10000;       // 最大迭代次数
    double goal_tolerance = 0.5;      // 目标容差 (m)
  };

  AStarPlanner();
  explicit AStarPlanner(const Config& config);

  bool plan(const PlanningContext& context,
            std::chrono::milliseconds deadline,
            PlanningResult& result) override;

  std::string getName() const override { return "AStarPlanner"; }
  std::string getType() const override { return "search"; }

  std::pair<bool, std::string> isAvailable(const PlanningContext& context) const override;

  std::unordered_map<std::string, double> getParameters() const override;
  bool setParameter(const std::string& name, double value) override;

private:
  Config config_;

  // A*搜索内部结构
  struct Node {
    Point2d position;
    double g_cost = 0.0;  // 从起点到当前点的代价
    double h_cost = 0.0;  // 从当前点到终点的启发式代价
    double f_cost = 0.0;  // 总代价
    std::shared_ptr<Node> parent = nullptr;
  };

  std::vector<Point2d> searchPath(const PlanningContext& context);
  double heuristic(const Point2d& a, const Point2d& b) const;
  bool isCollisionFree(const Point2d& point, const PlanningContext& context) const;
};

/**
 * @brief 优化轨迹规划器 (基于优化的方法)
 */
class OptimizationPlanner : public PlannerInterface {
public:
  struct Config {
    double time_step = 0.1;
    int max_iterations = 100;         // 优化最大迭代次数
    double convergence_tolerance = 1e-3; // 收敛容差
    double smoothness_weight = 1.0;   // 平滑性权重
    double obstacle_weight = 10.0;    // 避障权重
    double goal_weight = 5.0;         // 目标权重
  };

  OptimizationPlanner();
  explicit OptimizationPlanner(const Config& config);

  bool plan(const PlanningContext& context,
            std::chrono::milliseconds deadline,
            PlanningResult& result) override;

  std::string getName() const override { return "OptimizationPlanner"; }
  std::string getType() const override { return "optimization"; }

  std::pair<bool, std::string> isAvailable(const PlanningContext& context) const override;

  std::unordered_map<std::string, double> getParameters() const override;
  bool setParameter(const std::string& name, double value) override;

private:
  Config config_;

  // 优化相关函数
  double evaluateCost(const std::vector<TrajectoryPoint>& trajectory,
                      const PlanningContext& context) const;
  std::vector<TrajectoryPoint> optimizeTrajectory(const std::vector<TrajectoryPoint>& initial,
                                                  const PlanningContext& context,
                                                  std::chrono::milliseconds deadline) const;
};

/**
 * @brief 规划器管理器
 * 管理多个规划器，支持策略切换和降级
 */
class PlannerManager {
public:
  struct Config {
    std::string primary_planner = "OptimizationPlanner";     // 主规划器
    std::string fallback_planner = "StraightLinePlanner";    // 降级规划器
    bool enable_fallback = true;                             // 是否启用降级
    double fallback_timeout_ratio = 0.7;                     // 降级超时比例
    bool enable_visualization = false;                       // 是否启用可视化
    visualization::WebSocketVisualizer::Config viz_config;   // 可视化配置
  };

  PlannerManager();
  explicit PlannerManager(const Config& config);

  /**
   * @brief 注册规划器
   */
  void registerPlanner(std::unique_ptr<PlannerInterface> planner);

  /**
   * @brief 设置当前使用的规划器
   */
  bool setActivePlanner(const std::string& name);

  /**
   * @brief 执行规划
   * 会根据配置自动处理降级逻辑
   */
  bool plan(const PlanningContext& context,
            std::chrono::milliseconds deadline,
            PlanningResult& result);

  /**
   * @brief 获取可用的规划器列表
   */
  std::vector<std::string> getAvailablePlanners() const;

  /**
   * @brief 获取规划器状态
   */
  std::unordered_map<std::string, bool> getPlannerStatus(const PlanningContext& context) const;

  /**
   * @brief 获取当前配置
   */
  const Config& getConfig() const { return config_; }

  /**
   * @brief 更新配置
   */
  void updateConfig(const Config& config) { config_ = config; }

  /**
   * @brief 设置Bridge引用（用于可视化）
   */
  void setBridge(Bridge* bridge);

private:
  Config config_;
  std::unordered_map<std::string, std::unique_ptr<PlannerInterface>> planners_;
  std::string active_planner_;
  std::unique_ptr<visualization::WebSocketVisualizer> visualizer_;

  // 统计信息
  struct Statistics {
    int total_plans = 0;
    int successful_plans = 0;
    int fallback_plans = 0;
    double avg_computation_time = 0.0;
  } stats_;
};

/**
 * @brief 协议转换器
 * 将 PlanningResult 转换为 protobuf 消息
 */
class ProtocolConverter {
public:
  /**
   * @brief 转换规划结果为 PlanUpdate
   */
  static bool convertToPlanUpdate(const PlanningResult& result,
                                 uint64_t tick_id,
                                 double timestamp,
                                 proto::PlanUpdate& plan_update);

  /**
   * @brief 转换控制指令为 EgoCmd
   */
  static bool convertToEgoCmd(const PlanningResult& result,
                             uint64_t tick_id,
                             double timestamp,
                             proto::EgoCmd& ego_cmd);
};

} // namespace planning
} // namespace navsim
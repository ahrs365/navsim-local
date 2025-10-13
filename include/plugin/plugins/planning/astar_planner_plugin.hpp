#pragma once

#include "plugin/framework/planner_plugin_interface.hpp"
#include "core/planning_context.hpp"
#include <nlohmann/json.hpp>
#include <memory>
#include <vector>
#include <queue>
#include <unordered_set>

namespace navsim {
namespace plugins {
namespace planning {

/**
 * @brief A* 路径规划器插件
 * 
 * 基于 A* 搜索算法的路径规划器，在栅格地图上搜索最优路径。
 * 
 * 功能：
 * - A* 搜索算法（8-连通）
 * - 启发式函数（欧几里得距离）
 * - 碰撞检测（基于栅格地图）
 * - 路径平滑（可选）
 * 
 * 要求：
 * - 需要 context.occupancy_grid（栅格地图）
 * 
 * 适用场景：
 * - 静态环境中的路径规划
 * - 需要避障的场景
 * - 中等复杂度的环境
 */
class AStarPlannerPlugin : public plugin::PlannerPluginInterface {
public:
  /**
   * @brief 配置参数
   */
  struct Config {
    double time_step = 0.1;            // 时间步长 (s)
    double heuristic_weight = 1.0;     // 启发式权重
    double step_size = 0.5;            // 搜索步长 (m)
    int max_iterations = 10000;        // 最大迭代次数
    double goal_tolerance = 0.5;       // 目标容差 (m)
    double default_velocity = 1.0;     // 默认速度 (m/s)
    
    /**
     * @brief 从 JSON 加载配置
     */
    static Config fromJson(const nlohmann::json& json);
  };
  
  /**
   * @brief 构造函数
   */
  AStarPlannerPlugin();
  
  /**
   * @brief 带配置的构造函数
   */
  explicit AStarPlannerPlugin(const Config& config);
  
  // ========== 必须实现的接口 ==========
  
  /**
   * @brief 获取插件元数据
   */
  plugin::PlannerPluginMetadata getMetadata() const override;
  
  /**
   * @brief 初始化插件
   */
  bool initialize(const nlohmann::json& config) override;
  
  /**
   * @brief 规划轨迹
   * 
   * @param context 规划上下文
   * @param deadline 截止时间
   * @param result 规划结果（输出）
   * @return 规划是否成功
   */
  bool plan(const navsim::planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           plugin::PlanningResult& result) override;
  
  /**
   * @brief 检查规划器是否可用
   */
  std::pair<bool, std::string> isAvailable(const navsim::planning::PlanningContext& context) const override;
  
  // ========== 可选实现的接口 ==========
  
  /**
   * @brief 重置规划器状态
   */
  void reset() override;
  
  /**
   * @brief 获取统计信息
   */
  nlohmann::json getStatistics() const override;

private:
  /**
   * @brief A* 搜索节点
   */
  struct Node {
    navsim::planning::Point2d position;
    double g_cost = 0.0;  // 从起点到当前点的代价
    double h_cost = 0.0;  // 从当前点到终点的启发式代价
    double f_cost = 0.0;  // 总代价
    std::shared_ptr<Node> parent = nullptr;
  };

  /**
   * @brief 执行 A* 搜索
   *
   * @param context 规划上下文
   * @return 路径点列表（世界坐标）
   */
  std::vector<navsim::planning::Point2d> searchPath(const navsim::planning::PlanningContext& context);

  /**
   * @brief 启发式函数（欧几里得距离）
   *
   * @param a 点 A
   * @param b 点 B
   * @return 距离
   */
  double heuristic(const navsim::planning::Point2d& a, const navsim::planning::Point2d& b) const;

  /**
   * @brief 检查点是否无碰撞
   *
   * @param point 点
   * @param context 规划上下文
   * @return 是否无碰撞
   */
  bool isCollisionFree(const navsim::planning::Point2d& point,
                      const navsim::planning::PlanningContext& context) const;

  /**
   * @brief 将路径转换为轨迹
   *
   * @param path 路径点列表
   * @param context 规划上下文
   * @return 轨迹点列表
   */
  std::vector<plugin::TrajectoryPoint> pathToTrajectory(
      const std::vector<navsim::planning::Point2d>& path,
      const navsim::planning::PlanningContext& context) const;
  
  // 配置参数
  Config config_;
  
  // 统计信息
  struct Statistics {
    size_t total_plans = 0;
    size_t successful_plans = 0;
    size_t failed_no_grid = 0;
    size_t failed_no_path = 0;
    double average_time_ms = 0.0;
    double average_path_length = 0.0;
  };
  Statistics stats_;
};

} // namespace planning
} // namespace plugins
} // namespace navsim


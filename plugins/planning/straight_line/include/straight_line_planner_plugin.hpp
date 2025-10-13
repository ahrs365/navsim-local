#pragma once

#include "plugin/framework/planner_plugin_interface.hpp"
#include "core/planning_context.hpp"
#include <nlohmann/json.hpp>

namespace navsim {
namespace plugins {
namespace planning {

/**
 * @brief 直线规划器插件
 * 
 * 简单的几何规划器，生成从当前位置到目标点的直线轨迹。
 * 这是一个降级规划器，当主规划器失败时使用。
 * 
 * 功能：
 * - 生成直线轨迹
 * - 不考虑障碍物（仅用于降级）
 * - 简单的速度规划（匀速或梯形速度曲线）
 * - 快速计算（< 1ms）
 * 
 * 适用场景：
 * - 主规划器失败时的降级方案
 * - 开阔环境中的快速规划
 * - 调试和测试
 */
class StraightLinePlannerPlugin : public plugin::PlannerPluginInterface {
public:
  /**
   * @brief 配置参数
   */
  struct Config {
    double default_velocity = 1.0;     // 默认速度 (m/s)
    double time_step = 0.1;            // 时间步长 (s)
    double planning_horizon = 5.0;     // 规划时域 (s)
    bool use_trapezoidal_profile = true;  // 是否使用梯形速度曲线
    double max_acceleration = 1.0;     // 最大加速度 (m/s^2)
    
    /**
     * @brief 从 JSON 加载配置
     */
    static Config fromJson(const nlohmann::json& json);
  };
  
  /**
   * @brief 构造函数
   */
  StraightLinePlannerPlugin();
  
  /**
   * @brief 带配置的构造函数
   */
  explicit StraightLinePlannerPlugin(const Config& config);
  
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
   * @brief 生成直线轨迹点
   *
   * @param start 起点
   * @param goal 终点
   * @param num_points 轨迹点数量
   * @return 轨迹点列表
   */
  std::vector<plugin::TrajectoryPoint> generateStraightLine(
      const navsim::planning::Pose2d& start,
      const navsim::planning::Pose2d& goal,
      int num_points) const;
  
  /**
   * @brief 计算速度曲线
   * 
   * @param trajectory 轨迹（输入/输出）
   * @param total_distance 总距离
   */
  void computeVelocityProfile(std::vector<plugin::TrajectoryPoint>& trajectory,
                             double total_distance) const;
  
  /**
   * @brief 计算梯形速度曲线
   * 
   * @param trajectory 轨迹（输入/输出）
   * @param total_distance 总距离
   */
  void computeTrapezoidalProfile(std::vector<plugin::TrajectoryPoint>& trajectory,
                                 double total_distance) const;
  
  // 配置参数
  Config config_;
  
  // 统计信息
  struct Statistics {
    size_t total_plans = 0;
    size_t successful_plans = 0;
    double average_time_ms = 0.0;
  };
  Statistics stats_;
};

} // namespace planning
} // namespace plugins
} // namespace navsim


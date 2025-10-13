#pragma once

#include "planner_plugin_interface.hpp"
#include "plugin_registry.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <memory>
#include <chrono>

namespace navsim {
namespace plugin {

/**
 * @brief 规划器插件管理器
 * 
 * 负责加载、初始化和执行规划器插件。
 * 支持主规划器和降级规划器机制。
 * 
 * 工作流程：
 * 1. loadPlanners() - 从配置加载规划器
 * 2. initialize() - 初始化所有规划器
 * 3. plan() - 执行规划（主规划器失败时使用降级规划器）
 * 4. reset() - 重置所有规划器
 */
class PlannerPluginManager {
public:
  /**
   * @brief 构造函数
   */
  PlannerPluginManager() = default;
  
  /**
   * @brief 析构函数
   */
  ~PlannerPluginManager() = default;
  
  /**
   * @brief 从配置加载规划器
   * 
   * @param primary_planner_name 主规划器名称
   * @param fallback_planner_name 降级规划器名称
   * @param enable_fallback 是否启用降级机制
   * @param planner_configs 规划器配置（JSON 格式）
   * @return 加载是否成功
   * 
   * 配置示例：
   * {
   *   "AStarPlannerPlugin": {
   *     "time_step": 0.1,
   *     "heuristic_weight": 1.0
   *   },
   *   "StraightLinePlannerPlugin": {
   *     "time_step": 0.1,
   *     "default_velocity": 2.0
   *   }
   * }
   */
  bool loadPlanners(
      const std::string& primary_planner_name,
      const std::string& fallback_planner_name,
      bool enable_fallback,
      const nlohmann::json& planner_configs);
  
  /**
   * @brief 初始化所有规划器
   * 
   * @return 初始化是否成功
   */
  bool initialize();
  
  /**
   * @brief 执行规划
   * 
   * 首先尝试使用主规划器，如果失败且启用了降级机制，
   * 则使用降级规划器。
   * 
   * @param context 规划上下文
   * @param deadline 截止时间
   * @param result 规划结果（输出）
   * @return 规划是否成功
   */
  bool plan(
      const planning::PlanningContext& context,
      std::chrono::milliseconds deadline,
      PlanningResult& result);
  
  /**
   * @brief 重置所有规划器
   */
  void reset();
  
  /**
   * @brief 获取主规划器名称
   */
  std::string getPrimaryPlannerName() const {
    return primary_planner_name_;
  }
  
  /**
   * @brief 获取降级规划器名称
   */
  std::string getFallbackPlannerName() const {
    return fallback_planner_name_;
  }
  
  /**
   * @brief 检查是否启用降级机制
   */
  bool isFallbackEnabled() const {
    return enable_fallback_;
  }
  
  /**
   * @brief 获取统计信息
   */
  nlohmann::json getStatistics() const;

private:
  /**
   * @brief 尝试使用指定规划器进行规划
   * 
   * @param planner 规划器
   * @param planner_name 规划器名称
   * @param context 规划上下文
   * @param deadline 截止时间
   * @param result 规划结果（输出）
   * @return 规划是否成功
   */
  bool tryPlan(
      PlannerPluginPtr planner,
      const std::string& planner_name,
      const planning::PlanningContext& context,
      std::chrono::milliseconds deadline,
      PlanningResult& result);
  
  // 主规划器
  PlannerPluginPtr primary_planner_;
  std::string primary_planner_name_;
  
  // 降级规划器
  PlannerPluginPtr fallback_planner_;
  std::string fallback_planner_name_;
  
  // 是否启用降级机制
  bool enable_fallback_ = false;
  
  // 是否已初始化
  bool initialized_ = false;
  
  // 统计信息
  struct Statistics {
    size_t total_calls = 0;
    size_t primary_success = 0;
    size_t primary_failure = 0;
    size_t fallback_success = 0;
    size_t fallback_failure = 0;
    double total_time_ms = 0.0;
  } stats_;
};

} // namespace plugin
} // namespace navsim


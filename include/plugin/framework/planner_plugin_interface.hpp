#pragma once

#include "plugin/data/planning_result.hpp"
#include "plugin/framework/plugin_metadata.hpp"
#include "core/planning_context.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace navsim {
namespace plugin {

/**
 * @brief 规划器插件接口
 * 
 * 所有规划器插件必须实现此接口。
 * 
 * 设计原则：
 * - 规划器从 PlanningContext 读取感知数据，生成轨迹
 * - 规划器应该检查必需数据是否存在（通过 isAvailable()）
 * - 规划器应该在截止时间前完成计算
 * - 规划器应该是无状态的（或状态可重置）
 * 
 * 生命周期：
 * 1. 构造 - 创建规划器实例
 * 2. initialize() - 初始化规划器，读取配置
 * 3. isAvailable() - 检查是否可用（每次规划前调用）
 * 4. plan() - 生成轨迹（每个 tick 调用一次）
 * 5. reset() - 重置规划器状态（可选）
 * 6. 析构 - 销毁规划器实例
 */
class PlannerPluginInterface {
public:
  /**
   * @brief 虚析构函数
   */
  virtual ~PlannerPluginInterface() = default;
  
  // ========== 必须实现的方法 ==========
  
  /**
   * @brief 获取插件元数据
   * 
   * @return 插件元数据
   */
  virtual PlannerPluginMetadata getMetadata() const = 0;
  
  /**
   * @brief 初始化规划器
   * 
   * 在规划器加载后调用一次，用于读取配置参数、分配资源等。
   * 
   * @param config 规划器配置（JSON 格式）
   * @return 初始化是否成功
   * 
   * 配置示例：
   * {
   *   "time_step": 0.1,
   *   "heuristic_weight": 1.0,
   *   "max_iterations": 10000
   * }
   */
  virtual bool initialize(const nlohmann::json& config) = 0;
  
  /**
   * @brief 生成轨迹
   * 
   * 每个 tick 调用一次，从 PlanningContext 读取感知数据，
   * 生成从当前位置到目标的轨迹。
   * 
   * @param context 规划上下文（包含感知数据）
   * @param deadline 截止时间（规划器应该在此时间前完成）
   * @param result 规划结果（输出）
   * @return 规划是否成功
   * 
   * 实现要求：
   * - 应该在 deadline 前完成计算
   * - 如果超时，应该返回部分结果或失败
   * - 应该填充 result.planner_name
   * - 应该填充 result.computation_time_ms
   * 
   * 实现示例：
   * ```cpp
   * bool MyPlanner::plan(const planning::PlanningContext& context,
   *                     std::chrono::milliseconds deadline,
   *                     PlanningResult& result) {
   *   auto start_time = std::chrono::steady_clock::now();
   *   
   *   // 1. 检查必需数据
   *   if (!context.occupancy_grid) {
   *     result.success = false;
   *     result.failure_reason = "Missing occupancy grid";
   *     return false;
   *   }
   *   
   *   // 2. 执行规划
   *   auto trajectory = planTrajectory(context);
   *   
   *   // 3. 填充结果
   *   result.trajectory = trajectory;
   *   result.success = true;
   *   result.planner_name = "MyPlanner";
   *   
   *   auto end_time = std::chrono::steady_clock::now();
   *   result.computation_time_ms = 
   *       std::chrono::duration<double, std::milli>(end_time - start_time).count();
   *   
   *   return true;
   * }
   * ```
   */
  virtual bool plan(
      const planning::PlanningContext& context,
      std::chrono::milliseconds deadline,
      PlanningResult& result) = 0;
  
  /**
   * @brief 检查规划器是否可用
   * 
   * 在调用 plan() 前调用，检查规划器是否可用。
   * 规划器应该检查必需的感知数据是否存在。
   * 
   * @param context 规划上下文
   * @return {是否可用, 不可用原因}
   * 
   * 实现示例：
   * ```cpp
   * std::pair<bool, std::string> MyPlanner::isAvailable(
   *     const planning::PlanningContext& context) const {
   *   if (!context.occupancy_grid) {
   *     return {false, "Missing occupancy grid"};
   *   }
   *   return {true, ""};
   * }
   * ```
   */
  virtual std::pair<bool, std::string> isAvailable(
      const planning::PlanningContext& context) const = 0;
  
  // ========== 可选实现的方法 ==========
  
  /**
   * @brief 重置规划器状态
   * 
   * 在场景切换或需要重新开始时调用。
   * 规划器应该清除所有内部状态，回到初始化后的状态。
   */
  virtual void reset() {}
  
  /**
   * @brief 获取规划器统计信息
   * 
   * 返回规划器的运行统计信息，用于调试和性能分析。
   * 
   * @return 统计信息（JSON 格式）
   * 
   * 示例：
   * {
   *   "total_calls": 1000,
   *   "success_rate": 0.95,
   *   "average_time_ms": 15.5,
   *   "average_iterations": 500
   * }
   */
  virtual nlohmann::json getStatistics() const {
    return nlohmann::json::object();
  }
  
  /**
   * @brief 获取规划器名称（便捷方法）
   */
  std::string getName() const {
    return getMetadata().name;
  }
  
  /**
   * @brief 获取规划器版本（便捷方法）
   */
  std::string getVersion() const {
    return getMetadata().version;
  }
  
  /**
   * @brief 获取规划器类型（便捷方法）
   */
  std::string getType() const {
    return getMetadata().type;
  }
  
  /**
   * @brief 检查是否可以作为降级规划器（便捷方法）
   */
  bool canBeFallback() const {
    return getMetadata().can_be_fallback;
  }
};

/**
 * @brief 规划器插件智能指针类型
 */
using PlannerPluginPtr = std::shared_ptr<PlannerPluginInterface>;

/**
 * @brief 规划器插件工厂函数类型
 * 
 * 用于创建规划器实例的工厂函数。
 */
using PlannerPluginFactory = std::function<PlannerPluginPtr()>;

} // namespace plugin
} // namespace navsim


#pragma once

#include "plugin/data/perception_input.hpp"
#include "plugin/framework/plugin_metadata.hpp"
#include "core/planning_context.hpp"
#include <nlohmann/json.hpp>
#include <memory>
#include <string>

namespace navsim {
namespace plugin {

/**
 * @brief 感知插件接口
 * 
 * 所有感知插件必须实现此接口。
 * 
 * 设计原则：
 * - 感知插件从标准化的 PerceptionInput 构建特定的地图表示
 * - 插件专注于单一职责（例如：构建栅格地图、构建 ESDF、构建点云地图）
 * - 插件输出到 PlanningContext，供规划器使用
 * - 插件应该是无状态的（或状态可重置）
 * 
 * 生命周期：
 * 1. 构造 - 创建插件实例
 * 2. initialize() - 初始化插件，读取配置
 * 3. process() - 处理感知数据（每个 tick 调用一次）
 * 4. reset() - 重置插件状态（可选）
 * 5. 析构 - 销毁插件实例
 */
class PerceptionPluginInterface {
public:
  /**
   * @brief 虚析构函数
   */
  virtual ~PerceptionPluginInterface() = default;
  
  // ========== 必须实现的方法 ==========
  
  /**
   * @brief 获取插件元数据
   * 
   * @return 插件元数据
   */
  virtual PerceptionPluginMetadata getMetadata() const = 0;
  
  /**
   * @brief 初始化插件
   * 
   * 在插件加载后调用一次，用于读取配置参数、分配资源等。
   * 
   * @param config 插件配置（JSON 格式）
   * @return 初始化是否成功
   * 
   * 配置示例：
   * {
   *   "resolution": 0.1,
   *   "map_width": 100.0,
   *   "map_height": 100.0,
   *   "inflation_radius": 0.3
   * }
   */
  virtual bool initialize(const nlohmann::json& config) = 0;
  
  /**
   * @brief 处理感知数据
   * 
   * 每个 tick 调用一次，从 PerceptionInput 构建地图表示，
   * 并将结果输出到 PlanningContext。
   * 
   * @param input 标准化的感知输入数据
   * @param context 规划上下文（输出）
   * @return 处理是否成功
   * 
   * 实现示例：
   * ```cpp
   * bool MyPlugin::process(const PerceptionInput& input,
   *                       planning::PlanningContext& context) {
   *   // 1. 从 input 读取数据
   *   const auto& bev_obstacles = input.bev_obstacles;
   *   
   *   // 2. 构建地图
   *   auto my_map = buildMap(bev_obstacles);
   *   
   *   // 3. 输出到 context
   *   context.setCustomData("my_map", my_map);
   *   
   *   return true;
   * }
   * ```
   */
  virtual bool process(
      const PerceptionInput& input,
      planning::PlanningContext& context) = 0;
  
  // ========== 可选实现的方法 ==========
  
  /**
   * @brief 重置插件状态
   * 
   * 在场景切换或需要重新开始时调用。
   * 插件应该清除所有内部状态，回到初始化后的状态。
   */
  virtual void reset() {}
  
  /**
   * @brief 获取插件统计信息
   * 
   * 返回插件的运行统计信息，用于调试和性能分析。
   * 
   * @return 统计信息（JSON 格式）
   * 
   * 示例：
   * {
   *   "total_calls": 1000,
   *   "average_time_ms": 2.5,
   *   "last_map_size": 1000000
   * }
   */
  virtual nlohmann::json getStatistics() const {
    return nlohmann::json::object();
  }
  
  /**
   * @brief 检查插件是否可用
   * 
   * 检查插件是否处于可用状态（例如：是否已初始化）。
   * 
   * @return 是否可用
   */
  virtual bool isAvailable() const {
    return true;
  }
  
  /**
   * @brief 获取插件名称（便捷方法）
   */
  std::string getName() const {
    return getMetadata().name;
  }
  
  /**
   * @brief 获取插件版本（便捷方法）
   */
  std::string getVersion() const {
    return getMetadata().version;
  }
};

/**
 * @brief 感知插件智能指针类型
 */
using PerceptionPluginPtr = std::shared_ptr<PerceptionPluginInterface>;

/**
 * @brief 感知插件工厂函数类型
 * 
 * 用于创建插件实例的工厂函数。
 */
using PerceptionPluginFactory = std::function<PerceptionPluginPtr()>;

} // namespace plugin
} // namespace navsim


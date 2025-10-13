#pragma once

#include "plugin/framework/perception_plugin_interface.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <memory>

namespace navsim {
namespace plugin {

/**
 * @brief 感知插件配置
 */
struct PerceptionPluginConfig {
  std::string name;              // 插件名称
  bool enabled = true;           // 是否启用
  int priority = 0;              // 优先级（数字越小越先执行）
  nlohmann::json params;         // 插件参数
  
  PerceptionPluginConfig() = default;
  
  PerceptionPluginConfig(const std::string& name_, bool enabled_ = true, 
                        int priority_ = 0, const nlohmann::json& params_ = {})
      : name(name_), enabled(enabled_), priority(priority_), params(params_) {}
};

/**
 * @brief 感知插件管理器
 * 
 * 负责加载、初始化和执行感知插件。
 * 
 * 工作流程：
 * 1. loadPlugins() - 从配置加载插件
 * 2. initialize() - 初始化所有插件
 * 3. process() - 执行所有插件（每个 tick 调用一次）
 * 4. reset() - 重置所有插件
 */
class PerceptionPluginManager {
public:
  /**
   * @brief 构造函数
   */
  PerceptionPluginManager() = default;
  
  /**
   * @brief 析构函数
   */
  ~PerceptionPluginManager() = default;
  
  /**
   * @brief 从配置加载插件
   * 
   * @param plugin_configs 插件配置列表
   * @return 加载是否成功
   * 
   * 配置示例：
   * [
   *   {
   *     "name": "GridMapBuilderPlugin",
   *     "enabled": true,
   *     "priority": 1,
   *     "params": {
   *       "resolution": 0.1,
   *       "map_width": 100.0
   *     }
   *   }
   * ]
   */
  bool loadPlugins(const std::vector<PerceptionPluginConfig>& plugin_configs);
  
  /**
   * @brief 初始化所有插件
   * 
   * @return 初始化是否成功
   */
  bool initialize();
  
  /**
   * @brief 执行所有插件
   * 
   * 按优先级顺序执行所有已启用的插件。
   * 
   * @param input 感知输入数据
   * @param context 规划上下文（输出）
   * @return 执行是否成功
   */
  bool process(const PerceptionInput& input, planning::PlanningContext& context);
  
  /**
   * @brief 重置所有插件
   */
  void reset();
  
  /**
   * @brief 获取已加载的插件数量
   */
  size_t getPluginCount() const {
    return plugins_.size();
  }
  
  /**
   * @brief 获取已启用的插件数量
   */
  size_t getEnabledPluginCount() const;
  
  /**
   * @brief 获取插件列表
   */
  const std::vector<PerceptionPluginPtr>& getPlugins() const {
    return plugins_;
  }
  
  /**
   * @brief 获取插件配置列表
   */
  const std::vector<PerceptionPluginConfig>& getPluginConfigs() const {
    return plugin_configs_;
  }
  
  /**
   * @brief 获取统计信息
   */
  nlohmann::json getStatistics() const;

private:
  /**
   * @brief 按优先级排序插件
   */
  void sortPluginsByPriority();
  
  // 插件列表
  std::vector<PerceptionPluginPtr> plugins_;
  
  // 插件配置列表
  std::vector<PerceptionPluginConfig> plugin_configs_;
  
  // 是否已初始化
  bool initialized_ = false;
};

} // namespace plugin
} // namespace navsim


#pragma once

#include "plugin/framework/perception_plugin_manager.hpp"
#include "plugin/framework/planner_plugin_manager.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace navsim {
namespace plugin {

/**
 * @brief 配置加载器
 * 
 * 从 JSON 文件加载插件系统配置。
 * 
 * 配置文件格式：
 * {
 *   "perception": {
 *     "preprocessing": {
 *       "prediction_horizon": 5.0,
 *       "prediction_time_step": 0.1
 *     },
 *     "plugins": [
 *       {
 *         "name": "GridMapBuilderPlugin",
 *         "enabled": true,
 *         "priority": 1,
 *         "params": {
 *           "resolution": 0.1,
 *           "map_width": 100.0
 *         }
 *       }
 *     ]
 *   },
 *   "planning": {
 *     "primary_planner": "AStarPlannerPlugin",
 *     "fallback_planner": "StraightLinePlannerPlugin",
 *     "enable_fallback": true,
 *     "planners": {
 *       "AStarPlannerPlugin": {
 *         "time_step": 0.1,
 *         "heuristic_weight": 1.0
 *       },
 *       "StraightLinePlannerPlugin": {
 *         "time_step": 0.1,
 *         "default_velocity": 2.0
 *       }
 *     }
 *   }
 * }
 */
class ConfigLoader {
public:
  /**
   * @brief 构造函数
   */
  ConfigLoader() = default;
  
  /**
   * @brief 从文件加载配置
   * 
   * @param config_file 配置文件路径
   * @return 加载是否成功
   */
  bool loadFromFile(const std::string& config_file);
  
  /**
   * @brief 从 JSON 字符串加载配置
   * 
   * @param json_str JSON 字符串
   * @return 加载是否成功
   */
  bool loadFromString(const std::string& json_str);
  
  /**
   * @brief 从 JSON 对象加载配置
   * 
   * @param json JSON 对象
   * @return 加载是否成功
   */
  bool loadFromJson(const nlohmann::json& json);
  
  /**
   * @brief 获取感知插件配置列表
   */
  const std::vector<PerceptionPluginConfig>& getPerceptionPluginConfigs() const {
    return perception_plugin_configs_;
  }
  
  /**
   * @brief 获取主规划器名称
   */
  const std::string& getPrimaryPlannerName() const {
    return primary_planner_name_;
  }
  
  /**
   * @brief 获取降级规划器名称
   */
  const std::string& getFallbackPlannerName() const {
    return fallback_planner_name_;
  }
  
  /**
   * @brief 检查是否启用降级机制
   */
  bool isFallbackEnabled() const {
    return enable_fallback_;
  }
  
  /**
   * @brief 获取规划器配置
   */
  const nlohmann::json& getPlannerConfigs() const {
    return planner_configs_;
  }
  
  /**
   * @brief 获取前置处理配置
   */
  const nlohmann::json& getPreprocessingConfig() const {
    return preprocessing_config_;
  }
  
  /**
   * @brief 获取完整配置
   */
  const nlohmann::json& getConfig() const {
    return config_;
  }
  
  /**
   * @brief 保存配置到文件
   * 
   * @param config_file 配置文件路径
   * @return 保存是否成功
   */
  bool saveToFile(const std::string& config_file) const;

private:
  /**
   * @brief 解析感知配置
   */
  bool parsePerceptionConfig(const nlohmann::json& perception_config);
  
  /**
   * @brief 解析规划配置
   */
  bool parsePlanningConfig(const nlohmann::json& planning_config);
  
  // 配置数据
  nlohmann::json config_;
  
  // 感知插件配置
  std::vector<PerceptionPluginConfig> perception_plugin_configs_;
  nlohmann::json preprocessing_config_;
  
  // 规划器配置
  std::string primary_planner_name_;
  std::string fallback_planner_name_;
  bool enable_fallback_ = false;
  nlohmann::json planner_configs_;
};

/**
 * @brief 创建默认配置
 * 
 * @return 默认配置 JSON 对象
 */
nlohmann::json createDefaultConfig();

/**
 * @brief 验证配置
 * 
 * @param config 配置 JSON 对象
 * @return {是否有效, 错误信息}
 */
std::pair<bool, std::string> validateConfig(const nlohmann::json& config);

} // namespace plugin
} // namespace navsim


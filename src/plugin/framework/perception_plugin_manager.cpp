#include "plugin/framework/perception_plugin_manager.hpp"
#include <algorithm>
#include <iostream>

namespace navsim {
namespace plugin {

bool PerceptionPluginManager::loadPlugins(
    const std::vector<PerceptionPluginConfig>& plugin_configs) {
  // 清空现有插件
  plugins_.clear();
  plugin_configs_.clear();
  
  // 获取注册表
  auto& registry = PerceptionPluginRegistry::getInstance();
  
  // 加载每个插件
  for (const auto& config : plugin_configs) {
    if (!config.enabled) {
      std::cout << "[PerceptionPluginManager] Plugin '" << config.name 
                << "' is disabled, skipping" << std::endl;
      continue;
    }
    
    // 创建插件实例
    auto plugin = registry.createPlugin(config.name);
    if (!plugin) {
      std::cerr << "[PerceptionPluginManager] Failed to create plugin: " 
                << config.name << std::endl;
      return false;
    }
    
    // 添加到列表
    plugins_.push_back(plugin);
    plugin_configs_.push_back(config);
    
    std::cout << "[PerceptionPluginManager] Loaded plugin: " << config.name 
              << " (priority: " << config.priority << ")" << std::endl;
  }
  
  // 按优先级排序
  sortPluginsByPriority();
  
  std::cout << "[PerceptionPluginManager] Loaded " << plugins_.size() 
            << " plugins" << std::endl;
  
  return true;
}

bool PerceptionPluginManager::initialize() {
  if (initialized_) {
    std::cerr << "[PerceptionPluginManager] Already initialized!" << std::endl;
    return false;
  }
  
  // 初始化每个插件
  for (size_t i = 0; i < plugins_.size(); ++i) {
    const auto& plugin = plugins_[i];
    const auto& config = plugin_configs_[i];
    
    std::cout << "[PerceptionPluginManager] Initializing plugin: " 
              << config.name << std::endl;
    
    if (!plugin->initialize(config.params)) {
      std::cerr << "[PerceptionPluginManager] Failed to initialize plugin: " 
                << config.name << std::endl;
      return false;
    }
    
    std::cout << "[PerceptionPluginManager] Plugin '" << config.name 
              << "' initialized successfully" << std::endl;
  }
  
  initialized_ = true;
  std::cout << "[PerceptionPluginManager] All plugins initialized" << std::endl;
  
  return true;
}

bool PerceptionPluginManager::process(
    const PerceptionInput& input,
    planning::PlanningContext& context) {
  if (!initialized_) {
    std::cerr << "[PerceptionPluginManager] Not initialized!" << std::endl;
    return false;
  }
  
  // 执行每个插件
  for (size_t i = 0; i < plugins_.size(); ++i) {
    const auto& plugin = plugins_[i];
    const auto& config = plugin_configs_[i];
    
    // 检查插件是否可用
    if (!plugin->isAvailable()) {
      std::cerr << "[PerceptionPluginManager] Plugin '" << config.name 
                << "' is not available, skipping" << std::endl;
      continue;
    }
    
    // 执行插件
    if (!plugin->process(input, context)) {
      std::cerr << "[PerceptionPluginManager] Plugin '" << config.name 
                << "' failed to process" << std::endl;
      // 继续执行其他插件，不返回失败
      continue;
    }
  }
  
  return true;
}

void PerceptionPluginManager::reset() {
  for (const auto& plugin : plugins_) {
    plugin->reset();
  }
  std::cout << "[PerceptionPluginManager] All plugins reset" << std::endl;
}

size_t PerceptionPluginManager::getEnabledPluginCount() const {
  return plugins_.size();
}

nlohmann::json PerceptionPluginManager::getStatistics() const {
  nlohmann::json stats;
  stats["total_plugins"] = plugins_.size();
  stats["initialized"] = initialized_;
  
  nlohmann::json plugin_stats = nlohmann::json::array();
  for (size_t i = 0; i < plugins_.size(); ++i) {
    const auto& plugin = plugins_[i];
    const auto& config = plugin_configs_[i];
    
    nlohmann::json plugin_stat;
    plugin_stat["name"] = config.name;
    plugin_stat["priority"] = config.priority;
    plugin_stat["available"] = plugin->isAvailable();
    plugin_stat["stats"] = plugin->getStatistics();
    
    plugin_stats.push_back(plugin_stat);
  }
  stats["plugins"] = plugin_stats;
  
  return stats;
}

void PerceptionPluginManager::sortPluginsByPriority() {
  // 创建索引数组
  std::vector<size_t> indices(plugins_.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    indices[i] = i;
  }
  
  // 按优先级排序索引
  std::sort(indices.begin(), indices.end(),
            [this](size_t a, size_t b) {
              return plugin_configs_[a].priority < plugin_configs_[b].priority;
            });
  
  // 重新排列插件和配置
  std::vector<PerceptionPluginPtr> sorted_plugins;
  std::vector<PerceptionPluginConfig> sorted_configs;
  
  for (size_t idx : indices) {
    sorted_plugins.push_back(plugins_[idx]);
    sorted_configs.push_back(plugin_configs_[idx]);
  }
  
  plugins_ = sorted_plugins;
  plugin_configs_ = sorted_configs;
}

} // namespace plugin
} // namespace navsim


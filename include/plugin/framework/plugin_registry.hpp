#pragma once

#include "plugin/framework/perception_plugin_interface.hpp"
#include "plugin/framework/planner_plugin_interface.hpp"
#include <map>
#include <memory>
#include <string>
#include <functional>
#include <iostream>

namespace navsim {
namespace plugin {

/**
 * @brief 感知插件注册表
 * 
 * 单例模式，管理所有感知插件的注册和创建。
 * 
 * 使用方式：
 * 1. 插件通过 REGISTER_PERCEPTION_PLUGIN 宏自动注册
 * 2. 插件管理器通过 createPlugin() 创建插件实例
 */
class PerceptionPluginRegistry {
public:
  /**
   * @brief 获取单例实例
   */
  static PerceptionPluginRegistry& getInstance() {
    static PerceptionPluginRegistry instance;
    return instance;
  }
  
  /**
   * @brief 注册插件
   * 
   * @param name 插件名称
   * @param factory 插件工厂函数
   * @return 注册是否成功
   */
  bool registerPlugin(const std::string& name, PerceptionPluginFactory factory) {
    if (factories_.find(name) != factories_.end()) {
      std::cerr << "[PerceptionPluginRegistry] Plugin '" << name 
                << "' already registered!" << std::endl;
      return false;
    }
    
    factories_[name] = factory;
    std::cout << "[PerceptionPluginRegistry] Registered plugin: " << name << std::endl;
    return true;
  }
  
  /**
   * @brief 创建插件实例
   * 
   * @param name 插件名称
   * @return 插件实例，如果插件未注册则返回 nullptr
   */
  PerceptionPluginPtr createPlugin(const std::string& name) const {
    auto it = factories_.find(name);
    if (it == factories_.end()) {
      std::cerr << "[PerceptionPluginRegistry] Plugin '" << name 
                << "' not found!" << std::endl;
      return nullptr;
    }
    
    return it->second();
  }
  
  /**
   * @brief 检查插件是否已注册
   * 
   * @param name 插件名称
   * @return 是否已注册
   */
  bool hasPlugin(const std::string& name) const {
    return factories_.find(name) != factories_.end();
  }
  
  /**
   * @brief 获取所有已注册的插件名称
   * 
   * @return 插件名称列表
   */
  std::vector<std::string> getPluginNames() const {
    std::vector<std::string> names;
    for (const auto& pair : factories_) {
      names.push_back(pair.first);
    }
    return names;
  }
  
  /**
   * @brief 获取已注册插件数量
   */
  size_t getPluginCount() const {
    return factories_.size();
  }

private:
  // 私有构造函数（单例模式）
  PerceptionPluginRegistry() = default;
  
  // 禁止拷贝和赋值
  PerceptionPluginRegistry(const PerceptionPluginRegistry&) = delete;
  PerceptionPluginRegistry& operator=(const PerceptionPluginRegistry&) = delete;
  
  // 插件工厂函数映射表
  std::map<std::string, PerceptionPluginFactory> factories_;
};

/**
 * @brief 规划器插件注册表
 * 
 * 单例模式，管理所有规划器插件的注册和创建。
 */
class PlannerPluginRegistry {
public:
  /**
   * @brief 获取单例实例
   */
  static PlannerPluginRegistry& getInstance() {
    static PlannerPluginRegistry instance;
    return instance;
  }
  
  /**
   * @brief 注册插件
   * 
   * @param name 插件名称
   * @param factory 插件工厂函数
   * @return 注册是否成功
   */
  bool registerPlugin(const std::string& name, PlannerPluginFactory factory) {
    if (factories_.find(name) != factories_.end()) {
      std::cerr << "[PlannerPluginRegistry] Plugin '" << name 
                << "' already registered!" << std::endl;
      return false;
    }
    
    factories_[name] = factory;
    std::cout << "[PlannerPluginRegistry] Registered plugin: " << name << std::endl;
    return true;
  }
  
  /**
   * @brief 创建插件实例
   * 
   * @param name 插件名称
   * @return 插件实例，如果插件未注册则返回 nullptr
   */
  PlannerPluginPtr createPlugin(const std::string& name) const {
    auto it = factories_.find(name);
    if (it == factories_.end()) {
      std::cerr << "[PlannerPluginRegistry] Plugin '" << name 
                << "' not found!" << std::endl;
      return nullptr;
    }
    
    return it->second();
  }
  
  /**
   * @brief 检查插件是否已注册
   * 
   * @param name 插件名称
   * @return 是否已注册
   */
  bool hasPlugin(const std::string& name) const {
    return factories_.find(name) != factories_.end();
  }
  
  /**
   * @brief 获取所有已注册的插件名称
   * 
   * @return 插件名称列表
   */
  std::vector<std::string> getPluginNames() const {
    std::vector<std::string> names;
    for (const auto& pair : factories_) {
      names.push_back(pair.first);
    }
    return names;
  }
  
  /**
   * @brief 获取已注册插件数量
   */
  size_t getPluginCount() const {
    return factories_.size();
  }

private:
  // 私有构造函数（单例模式）
  PlannerPluginRegistry() = default;
  
  // 禁止拷贝和赋值
  PlannerPluginRegistry(const PlannerPluginRegistry&) = delete;
  PlannerPluginRegistry& operator=(const PlannerPluginRegistry&) = delete;
  
  // 插件工厂函数映射表
  std::map<std::string, PlannerPluginFactory> factories_;
};

// ========== 插件注册辅助类 ==========

/**
 * @brief 感知插件注册辅助类
 * 
 * 用于在全局作用域自动注册插件。
 * 通过 REGISTER_PERCEPTION_PLUGIN 宏使用。
 */
template<typename PluginType>
class PerceptionPluginRegistrar {
public:
  explicit PerceptionPluginRegistrar(const std::string& name) {
    PerceptionPluginRegistry::getInstance().registerPlugin(
        name,
        []() -> PerceptionPluginPtr {
          return std::make_shared<PluginType>();
        });
  }
};

/**
 * @brief 规划器插件注册辅助类
 * 
 * 用于在全局作用域自动注册插件。
 * 通过 REGISTER_PLANNER_PLUGIN 宏使用。
 */
template<typename PluginType>
class PlannerPluginRegistrar {
public:
  explicit PlannerPluginRegistrar(const std::string& name) {
    PlannerPluginRegistry::getInstance().registerPlugin(
        name,
        []() -> PlannerPluginPtr {
          return std::make_shared<PluginType>();
        });
  }
};

} // namespace plugin
} // namespace navsim

// ========== 插件注册宏 ==========

/**
 * @brief 注册感知插件宏
 * 
 * 在插件类定义后使用此宏自动注册插件。
 * 
 * 使用示例：
 * ```cpp
 * class MyPerceptionPlugin : public PerceptionPluginInterface {
 *   // ... 实现接口 ...
 * };
 * 
 * REGISTER_PERCEPTION_PLUGIN(MyPerceptionPlugin)
 * ```
 */
#define REGISTER_PERCEPTION_PLUGIN(PluginClass) \
  namespace { \
    static ::navsim::plugin::PerceptionPluginRegistrar<PluginClass> \
        registrar_##PluginClass(#PluginClass); \
  }

/**
 * @brief 注册规划器插件宏
 * 
 * 在插件类定义后使用此宏自动注册插件。
 * 
 * 使用示例：
 * ```cpp
 * class MyPlannerPlugin : public PlannerPluginInterface {
 *   // ... 实现接口 ...
 * };
 * 
 * REGISTER_PLANNER_PLUGIN(MyPlannerPlugin)
 * ```
 */
#define REGISTER_PLANNER_PLUGIN(PluginClass) \
  namespace { \
    static ::navsim::plugin::PlannerPluginRegistrar<PluginClass> \
        registrar_##PluginClass(#PluginClass); \
  }


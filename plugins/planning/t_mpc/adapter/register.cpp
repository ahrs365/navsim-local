/**
 * @file register.cpp
 * @brief Plugin registration for TMPCPlanner
 */

#include "tmpc_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <memory>
#include <iostream>

namespace tmpc_planner {
namespace adapter {

// 插件自注册函数
void registerTMPCPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    std::cout << "[DEBUG] Registering TMPCPlanner plugin..." << std::endl;
    navsim::plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "TMPCPlanner",
        []() -> std::shared_ptr<navsim::plugin::PlannerPluginInterface> {
          return std::make_shared<TMPCPlannerPlugin>();
        });
    registered = true;
    std::cout << "[DEBUG] TMPCPlanner plugin registered successfully" << std::endl;
  }
}

} // namespace adapter
} // namespace tmpc_planner

// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void registerTMPCPlannerPlugin() {
    tmpc_planner::adapter::registerTMPCPlannerPlugin();
  }
}

// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
  struct TMPCPlannerPluginInitializer {
    TMPCPlannerPluginInitializer() {
      tmpc_planner::adapter::registerTMPCPlannerPlugin();
    }
  };
  static TMPCPlannerPluginInitializer g_tmpc_planner_initializer;
}


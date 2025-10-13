#include "grid_map_builder_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <iostream>

namespace navsim {
namespace plugins {
namespace perception {

// 插件自注册函数
void registerGridMapBuilderPlugin() {
  static bool registered = false;
  if (!registered) {
    std::cout << "[DEBUG] Registering GridMapBuilder plugin..." << std::endl;
    plugin::PerceptionPluginRegistry::getInstance().registerPlugin(
        "GridMapBuilder",
        []() -> std::shared_ptr<plugin::PerceptionPluginInterface> {
          return std::make_shared<GridMapBuilderPlugin>();
        });
    registered = true;
    std::cout << "[DEBUG] GridMapBuilder plugin registered successfully" << std::endl;
  }
}

} // namespace perception
} // namespace plugins
} // namespace navsim

// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void registerGridMapBuilderPlugin() {
    navsim::plugins::perception::registerGridMapBuilderPlugin();
  }
}

// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
struct GridMapBuilderPluginInitializer {
  GridMapBuilderPluginInitializer() {
    navsim::plugins::perception::registerGridMapBuilderPlugin();
  }
};
static GridMapBuilderPluginInitializer g_grid_map_builder_initializer;
}


#include "straight_line_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <memory>

namespace straight_line {
namespace adapter {

// 插件自注册函数
void registerStraightLinePlugin() {
  static bool registered = false;
  if (!registered) {
    std::cout << "[DEBUG] Registering StraightLine plugin..." << std::endl;
    navsim::plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "StraightLine",
        []() -> std::shared_ptr<navsim::plugin::PlannerPluginInterface> {
          return std::make_shared<StraightLinePlugin>();
        });
    registered = true;
    std::cout << "[DEBUG] StraightLine plugin registered successfully" << std::endl;
  }
}

} // namespace adapter
} // namespace straight_line

// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void registerStraightLinePlugin() {
    straight_line::adapter::registerStraightLinePlugin();
  }
}

// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
  struct StraightLinePluginInitializer {
    StraightLinePluginInitializer() {
      straight_line::adapter::registerStraightLinePlugin();
    }
  };
  static StraightLinePluginInitializer g_straight_line_initializer;
}


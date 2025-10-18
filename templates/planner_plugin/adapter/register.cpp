#include "{{PLUGIN_NAME_SNAKE}}_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <memory>

namespace {{NAMESPACE}} {
namespace adapter {

// 插件自注册函数
void register{{PLUGIN_NAME}}Plugin() {
  static bool registered = false;
  if (!registered) {
    std::cout << "[DEBUG] Registering {{PLUGIN_NAME}} plugin..." << std::endl;
    navsim::plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "{{PLUGIN_NAME}}",
        []() -> std::shared_ptr<navsim::plugin::PlannerPluginInterface> {
          return std::make_shared<{{PLUGIN_NAME}}Plugin>();
        });
    registered = true;
    std::cout << "[DEBUG] {{PLUGIN_NAME}} plugin registered successfully" << std::endl;
  }
}

} // namespace adapter
} // namespace {{NAMESPACE}}

// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void register{{PLUGIN_NAME}}Plugin() {
    {{NAMESPACE}}::adapter::register{{PLUGIN_NAME}}Plugin();
  }
}

// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
  struct {{PLUGIN_NAME}}PluginInitializer {
    {{PLUGIN_NAME}}PluginInitializer() {
      {{NAMESPACE}}::adapter::register{{PLUGIN_NAME}}Plugin();
    }
  };
  static {{PLUGIN_NAME}}PluginInitializer g_{{PLUGIN_NAME_SNAKE}}_initializer;
}


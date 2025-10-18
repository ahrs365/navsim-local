#include "test_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <memory>

namespace test_planner {
namespace adapter {

// 插件自注册函数
void registerTestPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    std::cout << "[DEBUG] Registering TestPlanner plugin..." << std::endl;
    navsim::plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "TestPlanner",
        []() -> std::shared_ptr<navsim::plugin::PlannerPluginInterface> {
          return std::make_shared<TestPlannerPlugin>();
        });
    registered = true;
    std::cout << "[DEBUG] TestPlanner plugin registered successfully" << std::endl;
  }
}

} // namespace adapter
} // namespace test_planner

// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void registerTestPlannerPlugin() {
    test_planner::adapter::registerTestPlannerPlugin();
  }
}

// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
  struct TestPlannerPluginInitializer {
    TestPlannerPluginInitializer() {
      test_planner::adapter::registerTestPlannerPlugin();
    }
  };
  static TestPlannerPluginInitializer g_test_planner_initializer;
}


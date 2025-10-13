#include "plugin/plugin_registry.hpp"
#include "../../plugins/perception/grid_map_builder_plugin.hpp"
#include "../../plugins/planning/straight_line_planner_plugin.hpp"
#include "../../plugins/planning/astar_planner_plugin.hpp"

namespace navsim {
namespace plugin {

/**
 * @brief 初始化所有插件
 * 
 * 这个函数强制链接器包含所有插件的注册代码。
 * 必须在使用插件系统之前调用。
 */
void initializeAllPlugins() {
  // 手动注册所有插件

  // 感知插件
  PerceptionPluginRegistry::getInstance().registerPlugin(
      "GridMapBuilder",
      []() -> std::shared_ptr<PerceptionPluginInterface> {
        return std::make_shared<plugins::perception::GridMapBuilderPlugin>();
      });

  // 规划器插件
  PlannerPluginRegistry::getInstance().registerPlugin(
      "StraightLinePlanner",
      []() -> std::shared_ptr<PlannerPluginInterface> {
        return std::make_shared<plugins::planning::StraightLinePlannerPlugin>();
      });

  PlannerPluginRegistry::getInstance().registerPlugin(
      "AStarPlanner",
      []() -> std::shared_ptr<PlannerPluginInterface> {
        return std::make_shared<plugins::planning::AStarPlannerPlugin>();
      });
}

} // namespace plugin
} // namespace navsim


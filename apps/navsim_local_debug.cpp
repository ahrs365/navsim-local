/**
 * @file navsim_local_debug.cpp
 * @brief 本地调试工具 - 独立运行规划器，无需 navsim-online
 * 
 * 功能：
 * - 从 JSON 文件加载场景
 * - 加载指定的规划器和感知插件
 * - 运行规划并输出结果
 * - 可选：可视化规划结果
 * 
 * 使用示例：
 * ```bash
 * # 基本用法
 * ./navsim_local_debug --scenario scenarios/simple_corridor.json --planner JpsPlanner
 * 
 * # 指定感知插件
 * ./navsim_local_debug --scenario scenarios/complex.json \
 *                      --planner AStarPlanner \
 *                      --perception GridMapBuilder,ESDFBuilder
 * 
 * # 使用完整路径
 * ./navsim_local_debug --scenario scenarios/test.json \
 *                      --planner /home/user/MyPlanner/build/libmy_planner.so
 * 
 * # 启用可视化
 * ./navsim_local_debug --scenario scenarios/test.json \
 *                      --planner JpsPlanner \
 *                      --visualize
 * ```
 */

#include "core/scenario_loader.hpp"
#include "plugin/framework/dynamic_plugin_loader.hpp"
#include "plugin/framework/plugin_init.hpp"
#include "plugin/framework/planner_plugin_manager.hpp"
#include "plugin/framework/perception_plugin_manager.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include "plugin/data/planning_result.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>

using namespace navsim;

// ========== 命令行参数 ==========

struct CommandLineArgs {
  std::string scenario_file;
  std::string planner_name;
  std::vector<std::string> perception_plugins;
  bool visualize = false;
  bool verbose = false;
  std::string output_file;  // 可选：保存规划结果到文件
  
  void print() const {
    std::cout << "=== Command Line Arguments ===" << std::endl;
    std::cout << "Scenario file: " << scenario_file << std::endl;
    std::cout << "Planner: " << planner_name << std::endl;
    std::cout << "Perception plugins: ";
    for (size_t i = 0; i < perception_plugins.size(); ++i) {
      std::cout << perception_plugins[i];
      if (i < perception_plugins.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;
    std::cout << "Visualize: " << (visualize ? "yes" : "no") << std::endl;
    std::cout << "Verbose: " << (verbose ? "yes" : "no") << std::endl;
    if (!output_file.empty()) {
      std::cout << "Output file: " << output_file << std::endl;
    }
    std::cout << "===============================" << std::endl;
  }
};

void printUsage(const char* program_name) {
  std::cout << "Usage: " << program_name << " [options]\n\n"
            << "Options:\n"
            << "  --scenario <file>       JSON scenario file (required)\n"
            << "  --planner <name>        Planner plugin name or path (required)\n"
            << "  --perception <plugins>  Comma-separated perception plugin names (optional)\n"
            << "  --visualize             Enable visualization (optional)\n"
            << "  --verbose               Enable verbose logging (optional)\n"
            << "  --output <file>         Save planning result to JSON file (optional)\n"
            << "  --help                  Show this help message\n\n"
            << "Examples:\n"
            << "  " << program_name << " --scenario scenarios/simple.json --planner JpsPlanner\n"
            << "  " << program_name << " --scenario scenarios/test.json --planner AStarPlanner --perception GridMapBuilder,ESDFBuilder\n"
            << "  " << program_name << " --scenario scenarios/complex.json --planner /path/to/libmy_planner.so --visualize\n"
            << std::endl;
}

bool parseCommandLine(int argc, char** argv, CommandLineArgs& args) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return false;
    } else if (arg == "--scenario" && i + 1 < argc) {
      args.scenario_file = argv[++i];
    } else if (arg == "--planner" && i + 1 < argc) {
      args.planner_name = argv[++i];
    } else if (arg == "--perception" && i + 1 < argc) {
      std::string plugins_str = argv[++i];
      // 分割逗号分隔的插件列表
      size_t start = 0;
      size_t end = plugins_str.find(',');
      while (end != std::string::npos) {
        args.perception_plugins.push_back(plugins_str.substr(start, end - start));
        start = end + 1;
        end = plugins_str.find(',', start);
      }
      args.perception_plugins.push_back(plugins_str.substr(start));
    } else if (arg == "--visualize") {
      args.visualize = true;
    } else if (arg == "--verbose") {
      args.verbose = true;
    } else if (arg == "--output" && i + 1 < argc) {
      args.output_file = argv[++i];
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      printUsage(argv[0]);
      return false;
    }
  }
  
  // 检查必需参数
  if (args.scenario_file.empty()) {
    std::cerr << "Error: --scenario is required" << std::endl;
    printUsage(argv[0]);
    return false;
  }
  
  if (args.planner_name.empty()) {
    std::cerr << "Error: --planner is required" << std::endl;
    printUsage(argv[0]);
    return false;
  }
  
  return true;
}

// ========== 主函数 ==========

int main(int argc, char** argv) {
  std::cout << "=== NavSim Local Debug Tool ===" << std::endl;
  std::cout << "Version: 1.0.0" << std::endl;
  std::cout << "===============================" << std::endl << std::endl;
  
  // 解析命令行参数
  CommandLineArgs args;
  if (!parseCommandLine(argc, argv, args)) {
    return 1;
  }
  
  if (args.verbose) {
    args.print();
    std::cout << std::endl;
  }
  
  // 1. 初始化插件系统
  std::cout << "[1/5] Initializing plugin system..." << std::endl;
  navsim::plugin::initializeAllPlugins();
  
  // 2. 加载插件
  std::cout << "[2/5] Loading plugins..." << std::endl;
  plugin::DynamicPluginLoader plugin_loader;
  
  // 加载感知插件
  for (const auto& plugin_name : args.perception_plugins) {
    std::cout << "  Loading perception plugin: " << plugin_name << std::endl;
    if (!plugin_loader.loadPlugin(plugin_name, "")) {
      std::cerr << "  Failed to load perception plugin: " << plugin_name << std::endl;
      return 1;
    }
  }
  
  // 加载规划器插件
  std::cout << "  Loading planner plugin: " << args.planner_name << std::endl;
  if (!plugin_loader.loadPlugin(args.planner_name, "")) {
    std::cerr << "  Failed to load planner plugin: " << args.planner_name << std::endl;
    return 1;
  }

  std::cout << "  Successfully loaded " << plugin_loader.getLoadedPlugins().size()
            << " plugins" << std::endl;

  // 调试：检查注册表中的插件
  std::cout << "  Checking plugin registry..." << std::endl;
  auto& registry = plugin::PlannerPluginRegistry::getInstance();
  std::cout << "  Registry has " << registry.getPluginCount() << " plugins" << std::endl;
  auto plugin_names = registry.getPluginNames();
  for (const auto& name : plugin_names) {
    std::cout << "    - " << name << std::endl;
  }
  
  // 3. 加载场景
  std::cout << "[3/5] Loading scenario from: " << args.scenario_file << std::endl;
  planning::PlanningContext context;
  if (!planning::ScenarioLoader::loadFromFile(args.scenario_file, context)) {
    std::cerr << "  Failed to load scenario" << std::endl;
    return 1;
  }
  
  std::cout << "  Scenario loaded successfully" << std::endl;
  std::cout << "  Ego pose: (" << context.ego.pose.x << ", " 
            << context.ego.pose.y << ", " << context.ego.pose.yaw << ")" << std::endl;
  std::cout << "  Goal pose: (" << context.task.goal_pose.x << ", " 
            << context.task.goal_pose.y << ", " << context.task.goal_pose.yaw << ")" << std::endl;
  
  // 4. 运行规划
  std::cout << "[4/5] Running planner..." << std::endl;

  // 创建规划器管理器
  plugin::PlannerPluginManager planner_manager;

  // 创建默认配置
  nlohmann::json planner_configs;
  planner_configs[args.planner_name] = {
    {"safe_dis", 0.3},
    {"max_jps_dis", 10.0},
    {"distance_weight", 1.0},
    {"yaw_weight", 1.0},
    {"traj_cut_length", 5.0},
    {"max_vel", 2.0},
    {"max_acc", 2.0},
    {"max_omega", 1.0},
    {"max_domega", 1.0},
    {"sample_time", 0.1},
    {"min_traj_num", 10},
    {"jps_truncation_time", 5.0},
    {"verbose", args.verbose}
  };

  if (!planner_manager.loadPlanners(args.planner_name, args.planner_name, false, planner_configs)) {
    std::cerr << "  Failed to load planner: " << args.planner_name << std::endl;
    return 1;
  }

  // 初始化规划器
  if (!planner_manager.initialize()) {
    std::cerr << "  Failed to initialize planner" << std::endl;
    return 1;
  }

  // 运行规划
  auto start_time = std::chrono::steady_clock::now();

  plugin::PlanningResult result;
  auto deadline = std::chrono::milliseconds(5000);  // 5秒超时
  bool success = planner_manager.plan(context, deadline, result);

  auto end_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  // 5. 输出结果
  std::cout << "[5/5] Planning result:" << std::endl;
  std::cout << "  Success: " << (success ? "yes" : "no") << std::endl;
  if (!success) {
    std::cout << "  Failure reason: " << result.failure_reason << std::endl;
  } else {
    std::cout << "  Planner: " << result.planner_name << std::endl;
    std::cout << "  Trajectory points: " << result.trajectory.size() << std::endl;
    std::cout << "  Computation time: " << duration << " ms" << std::endl;
    std::cout << "  Total cost: " << result.total_cost << std::endl;
  }
  
  // 保存结果到文件（如果指定）
  if (!args.output_file.empty() && success) {
    std::cout << "\nSaving result to: " << args.output_file << std::endl;
    // TODO: 实现结果保存
  }
  
  // 可视化（如果启用）
  if (args.visualize && success) {
    std::cout << "\nLaunching visualization..." << std::endl;
    // TODO: 实现可视化
    #ifdef ENABLE_VISUALIZATION
    std::cout << "Visualization is enabled in build" << std::endl;
    #else
    std::cout << "Warning: Visualization is not enabled in this build" << std::endl;
    std::cout << "Rebuild with -DENABLE_VISUALIZATION=ON to enable visualization" << std::endl;
    #endif
  }
  
  std::cout << "\n=== Done ===" << std::endl;
  return success ? 0 : 1;
}


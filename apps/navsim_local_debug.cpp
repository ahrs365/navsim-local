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
#include "plugin/data/perception_input.hpp"
#include "viz/visualizer_interface.hpp"
#include "viz/imgui_visualizer.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <thread>
#include <fstream>

using namespace navsim;

// ========== 命令行参数 ==========

struct CommandLineArgs {
  std::string scenario_file;
  std::string planner_name;
  std::vector<std::string> perception_plugins;
  bool visualize = true;  // 默认启用可视化
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
            << "  --visualize             Enable visualization (default: enabled)\n"
            << "  --no-visualize          Disable visualization\n"
            << "  --verbose               Enable verbose logging (optional)\n"
            << "  --output <file>         Save planning result to JSON file (optional)\n"
            << "  --help                  Show this help message\n\n"
            << "Examples:\n"
            << "  " << program_name << " --scenario scenarios/simple.json --planner JpsPlanner\n"
            << "  " << program_name << " --scenario scenarios/test.json --planner AStarPlanner --perception GridMapBuilder,ESDFBuilder\n"
            << "  " << program_name << " --scenario scenarios/complex.json --planner /path/to/libmy_planner.so --no-visualize\n"
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
    } else if (arg == "--no-visualize") {
      args.visualize = false;
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

// ========== 全局变量 ==========
std::unique_ptr<viz::IVisualizer> g_visualizer = nullptr;

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

  // 2.5. 初始化可视化器（如果启用）
  if (args.visualize) {
    std::cout << "[2.5/5] Initializing visualizer..." << std::endl;
    g_visualizer = viz::createVisualizer(true);
    if (g_visualizer && g_visualizer->initialize()) {
      std::cout << "  Visualizer initialized successfully" << std::endl;

      // 设置系统信息
      viz::IVisualizer::SystemInfo system_info;
      system_info.general["Tool"] = "NavSim Local Debug";
      system_info.general["Version"] = "1.0.0";
      system_info.general["Scenario"] = args.scenario_file;
      system_info.general["Planner"] = args.planner_name;
      system_info.general["Visualizer"] = "ImGui (SDL2/OpenGL2)";
      if (!args.perception_plugins.empty()) {
        for (size_t i = 0; i < args.perception_plugins.size(); ++i) {
          system_info.perception_plugins.push_back(args.perception_plugins[i]);
        }
      }
      system_info.planner_plugins.push_back("Primary: " + args.planner_name);
      g_visualizer->setSystemInfo(system_info);

      // 设置连接状态
      viz::IVisualizer::ConnectionStatus connection_status;
      connection_status.connected = true;
      connection_status.label = "Local Debug";
      connection_status.message = "Running local debug session";
      g_visualizer->updateConnectionStatus(connection_status);
    } else {
      std::cerr << "  Failed to initialize visualizer" << std::endl;
      g_visualizer.reset();
    }
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

  // 可视化场景数据
  if (g_visualizer) {
    g_visualizer->beginFrame();
    g_visualizer->showDebugInfo("Status", "Scenario Loaded");
    g_visualizer->drawEgo(context.ego);
    g_visualizer->drawGoal(context.task.goal_pose);

    // 可视化BEV障碍物（如果有）
    if (context.bev_obstacles) {
      g_visualizer->drawBEVObstacles(*context.bev_obstacles);
    }

    // 可视化动态障碍物
    g_visualizer->drawDynamicObstacles(context.dynamic_obstacles);
    g_visualizer->endFrame();
  }

  // 3.5. 运行感知插件（如果有）
  if (!args.perception_plugins.empty()) {
    std::cout << "[3.5/5] Running perception plugins..." << std::endl;

    // 创建感知插件管理器
    plugin::PerceptionPluginManager perception_manager;

    // 创建感知插件配置
    std::vector<plugin::PerceptionPluginConfig> perception_configs;
    int priority = 0;
    for (const auto& plugin_name : args.perception_plugins) {
      plugin::PerceptionPluginConfig config;
      config.name = plugin_name;
      config.enabled = true;
      config.priority = priority++;
      config.params = {
        {"resolution", 0.1},
        {"map_width", 100.0},
        {"map_height", 100.0},
        {"max_distance", 10.0},
        {"include_dynamic", true}
      };
      perception_configs.push_back(config);
    }

    if (!perception_manager.loadPlugins(perception_configs)) {
      std::cerr << "  Failed to load perception plugins" << std::endl;
      return 1;
    }

    if (!perception_manager.initialize()) {
      std::cerr << "  Failed to initialize perception plugins" << std::endl;
      return 1;
    }

    // 运行感知处理
    plugin::PerceptionInput perception_input;

    // 填充 PerceptionInput
    perception_input.ego = context.ego;
    perception_input.task = context.task;
    perception_input.timestamp = context.timestamp;

    // 从 context 中提取 BEV 障碍物（如果有）
    if (context.bev_obstacles) {
      perception_input.bev_obstacles = *context.bev_obstacles;
    }

    // 从 context 中提取动态障碍物
    perception_input.dynamic_obstacles = context.dynamic_obstacles;

    if (!perception_manager.process(perception_input, context)) {
      std::cerr << "  Perception processing failed" << std::endl;
      return 1;
    }

    std::cout << "  Perception processing completed" << std::endl;
    if (context.occupancy_grid) {
      std::cout << "    - Occupancy grid: " << context.occupancy_grid->config.width
                << "x" << context.occupancy_grid->config.height << " cells" << std::endl;
    }
    if (context.esdf_map) {
      std::cout << "    - ESDF map: " << context.esdf_map->config.width
                << "x" << context.esdf_map->config.height << " cells" << std::endl;
    }

    // 可视化感知处理结果
    if (g_visualizer) {
      g_visualizer->beginFrame();
      g_visualizer->showDebugInfo("Status", "Perception Completed");
      g_visualizer->updatePlanningContext(context);

      // 可视化栅格地图
      if (context.occupancy_grid) {
        g_visualizer->drawOccupancyGrid(*context.occupancy_grid);
      }

      g_visualizer->endFrame();
    }
  }
  
  // 4. 运行规划
  std::cout << "[4/5] Running planner..." << std::endl;

  // 创建规划器管理器
  plugin::PlannerPluginManager planner_manager;

  // 从 default.json 加载配置
  nlohmann::json planner_configs;
  // 尝试多个可能的配置文件路径
  std::vector<std::string> config_paths = {
    "config/default.json",
    "../config/default.json",
    "../../config/default.json"
  };

  std::string config_file;
  std::ifstream config_stream;
  for (const auto& path : config_paths) {
    config_stream.open(path);
    if (config_stream.is_open()) {
      config_file = path;
      break;
    }
  }
  if (config_stream.is_open()) {
    try {
      nlohmann::json full_config;
      config_stream >> full_config;
      if (full_config.contains("planning") && full_config["planning"].contains("planners")) {
        planner_configs = full_config["planning"]["planners"];
        std::cout << "  Loaded planner configurations from " << config_file << std::endl;
        if (args.verbose && planner_configs.contains(args.planner_name)) {
          std::cout << "  Configuration for " << args.planner_name << ":" << std::endl;
          std::cout << planner_configs[args.planner_name].dump(2) << std::endl;
        }
      } else {
        std::cerr << "  Warning: Config file does not contain planning.planners section" << std::endl;
      }
    } catch (const std::exception& e) {
      std::cerr << "  Warning: Failed to load config from " << config_file << ": " << e.what() << std::endl;
    }
  } else {
    std::cerr << "  Warning: Could not open config file: " << config_file << std::endl;
  }

  // 如果没有从文件加载到配置，使用默认配置
  if (!planner_configs.contains(args.planner_name)) {
    std::cout << "  Using default configuration for " << args.planner_name << std::endl;
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
  } else {
    // 覆盖 verbose 设置
    planner_configs[args.planner_name]["verbose"] = args.verbose;
  }

  if (!planner_manager.loadPlanners(args.planner_name, args.planner_name, false, planner_configs)) {
    std::cerr << "  Failed to load planner: " << args.planner_name << std::endl;
    return 1;
  }

  // 初始化规划器
  if (!planner_manager.initialize()) {
    std::cerr << "  Failed to initialize planner" << std::endl;
    return 1;
  }

  // 🔧 设置仿真控制回调（Start/Reset 按钮）- 在所有变量声明之后
  // 使用共享指针来标记是否需要重新规划
  auto need_replan = std::make_shared<bool>(false);

  if (g_visualizer) {
    auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(g_visualizer.get());
    if (imgui_viz) {
      // 保存初始场景文件路径
      std::string initial_scenario = args.scenario_file;

      imgui_viz->setSimulationControlCallbacks(
        // Start 回调：触发重新规划
        [need_replan]() {
          std::cout << "\n[Start] Triggering replanning..." << std::endl;
          *need_replan = true;
        },
        nullptr,  // pause_callback (not used in debug mode)
        // Reset 回调：重新加载初始场景
        [initial_scenario, &context, &planner_manager, need_replan]() {
          std::cout << "\n[Reset] Reloading initial scenario: " << initial_scenario << std::endl;

          // 清空所有可视化数据
          auto* viz = dynamic_cast<viz::ImGuiVisualizer*>(g_visualizer.get());
          if (viz) {
            viz->clearAllVisualizationData();
          }

          // 🔧 清空 context 中的旧场景数据（重要！避免旧数据残留）
          context = planning::PlanningContext();  // 重置为空的 context

          // 重新加载初始场景
          if (planning::ScenarioLoader::loadFromFile(initial_scenario, context)) {
            std::cout << "[Reset] Successfully reloaded scenario" << std::endl;
            std::cout << "  Ego pose: (" << context.ego.pose.x << ", "
                      << context.ego.pose.y << ", " << context.ego.pose.yaw << ")" << std::endl;
            std::cout << "  Goal pose: (" << context.task.goal_pose.x << ", "
                      << context.task.goal_pose.y << ", " << context.task.goal_pose.yaw << ")" << std::endl;

            // 更新可视化
            g_visualizer->drawEgo(context.ego);
            g_visualizer->drawGoal(context.task.goal_pose);
            if (context.bev_obstacles) {
              g_visualizer->drawBEVObstacles(*context.bev_obstacles);
            }
            g_visualizer->drawDynamicObstacles(context.dynamic_obstacles);

            // 重置规划器
            planner_manager.reset();

            // 标记需要重新规划
            *need_replan = true;

            std::cout << "[Reset] Scenario reloaded, click Start to plan" << std::endl;
          } else {
            std::cerr << "[Reset] Failed to reload scenario" << std::endl;
          }
          std::cout << "[Reset] Done\n" << std::endl;
        }
      );
    }
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

  // 可视化规划结果
  if (g_visualizer) {
    g_visualizer->beginFrame();

    // 更新状态信息
    if (success) {
      g_visualizer->showDebugInfo("Status", "Planning Success");
      g_visualizer->showDebugInfo("Planner", result.planner_name);

      std::ostringstream trajectory_info;
      trajectory_info << result.trajectory.size() << " points";
      g_visualizer->showDebugInfo("Trajectory", trajectory_info.str());

      std::ostringstream time_info;
      time_info << std::fixed << std::setprecision(2) << duration << " ms";
      g_visualizer->showDebugInfo("Computation Time", time_info.str());

      std::ostringstream cost_info;
      cost_info << std::fixed << std::setprecision(3) << result.total_cost;
      g_visualizer->showDebugInfo("Total Cost", cost_info.str());

      // 可视化轨迹
      g_visualizer->drawTrajectory(result.trajectory, result.planner_name);

      // 可视化调试路径（支持 JPS 和 TMPC）
      if (result.metadata.count("has_debug_paths") > 0 &&
          result.metadata.count("debug_paths_ptr") > 0) {
        // Get debug paths from the pointer stored in metadata
        auto* debug_paths_ptr = reinterpret_cast<std::vector<std::vector<planning::Pose2d>>*>(
            static_cast<uintptr_t>(result.metadata.at("debug_paths_ptr")));

        if (debug_paths_ptr && !debug_paths_ptr->empty()) {
          std::cout << "[Debug] Drawing " << debug_paths_ptr->size() << " debug paths for "
                    << result.planner_name << " visualization" << std::endl;

          std::vector<std::string> path_names;
          std::vector<std::string> colors;

          if (result.planner_name == "JPSPlanner") {
            path_names = {
              "Raw JPS Path",
              "Optimized Path",
              "Sample Trajectory",
              "MINCO Final",
              "MINCO Stage1 (Preprocessing)",
              "MINCO Stage2 (Main Opt)"
            };
            colors = {
              "red",      // Raw JPS Path - 红色
              "green",    // Optimized Path - 绿色
              "blue",     // Sample Trajectory - 蓝色
              "yellow",   // MINCO Final - 黄色（高对比度）
              "magenta",  // MINCO Stage1 - 洋红色（高对比度）
              "cyan"      // MINCO Stage2 - 青色（高对比度）
            };
          } else if (result.planner_name == "TMPCPlanner") {
            // TMPC paths - use path type information from metadata
            std::vector<std::string>* path_types_ptr = nullptr;
            if (result.metadata.count("debug_path_types_ptr") > 0) {
              path_types_ptr = reinterpret_cast<std::vector<std::string>*>(
                  static_cast<uintptr_t>(result.metadata.at("debug_path_types_ptr")));
            }

            if (path_types_ptr && path_types_ptr->size() == debug_paths_ptr->size()) {
              // Use path type information
              int guidance_idx = 0;
              int mpc_idx = 0;
              int obs_idx = 0;

              for (size_t i = 0; i < path_types_ptr->size(); ++i) {
                const std::string& type = (*path_types_ptr)[i];

                if (type == "reference") {
                  path_names.push_back("Reference Path");
                  colors.push_back("blue");
                } else if (type == "guidance") {
                  path_names.push_back("Guidance Path " + std::to_string(guidance_idx++));
                  colors.push_back("dashed_cyan");
                } else if (type == "mpc_candidate") {
                  path_names.push_back("MPC Candidate " + std::to_string(mpc_idx++));
                  colors.push_back("orange");
                } else if (type == "obstacle_prediction") {
                  path_names.push_back("Obstacle " + std::to_string(obs_idx++) + " Prediction");
                  colors.push_back("red");
                } else if (type == "past_trajectory") {
                  path_names.push_back("Past Trajectory");
                  colors.push_back("green");
                } else {
                  path_names.push_back("Unknown Path");
                  colors.push_back("white");
                }
              }
            } else {
              // Fallback: use simple naming
              for (size_t i = 0; i < debug_paths_ptr->size(); ++i) {
                path_names.push_back("TMPC Path " + std::to_string(i));
                colors.push_back("white");
              }
            }
          }

          g_visualizer->drawDebugPaths(*debug_paths_ptr, path_names, colors);
        } else {
          std::cout << "[Debug] No debug paths available for " << result.planner_name << " visualization" << std::endl;
        }
      }

      // 可视化近似圆（TMPC 规划器）
      if (result.planner_name == "TMPCPlanner" &&
          result.metadata.count("approximation_circles_ptr") > 0 &&
          result.metadata.count("approximation_circles_count") > 0) {

        // 定义与插件中相同的结构
        struct ApproximationCircle {
          double x, y, radius;
        };

        auto* circles_ptr = reinterpret_cast<std::vector<ApproximationCircle>*>(
            static_cast<uintptr_t>(result.metadata.at("approximation_circles_ptr")));

        if (circles_ptr && !circles_ptr->empty()) {
          std::vector<std::tuple<double, double, double>> circles;
          for (const auto& c : *circles_ptr) {
            circles.emplace_back(c.x, c.y, c.radius);
          }

          // Use dynamic_cast to call ImGuiVisualizer-specific method
          auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(g_visualizer.get());
          if (imgui_viz) {
            imgui_viz->drawApproximationCircles(circles);
            std::cout << "[Debug] Drawing " << circles.size() << " approximation circles" << std::endl;
          }
        }
      }

      g_visualizer->updatePlanningResult(result);
    } else {
      g_visualizer->showDebugInfo("Status", "Planning Failed");
      g_visualizer->showDebugInfo("Failure Reason", result.failure_reason);
      g_visualizer->updatePlanningResult(result);
    }

    g_visualizer->endFrame();
  }
  
  // 保存结果到文件（如果指定）
  if (!args.output_file.empty() && success) {
    std::cout << "\nSaving result to: " << args.output_file << std::endl;
    // TODO: 实现结果保存
  }
  
  // 可视化（如果启用）
  if (args.visualize && g_visualizer) {
    std::cout << "\nVisualization is running..." << std::endl;
    std::cout << "Press Ctrl+C to exit, or close the visualization window" << std::endl;

    // 持续渲染可视化
    while (!g_visualizer->shouldClose()) {
      g_visualizer->beginFrame();

      // 🔧 检查场景加载请求（在渲染之前处理，确保新场景立即显示）
      auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(g_visualizer.get());
      if (imgui_viz) {
        std::string scenario_path;
        if (imgui_viz->hasScenarioLoadRequest(scenario_path)) {
          std::cout << "\n[Load Scenario] Request received: " << scenario_path << std::endl;

          // 清空所有可视化数据
          imgui_viz->clearAllVisualizationData();

          // 🔧 清空 context 中的旧场景数据（重要！避免旧数据残留）
          context = planning::PlanningContext();  // 重置为空的 context

          // 加载新场景
          if (planning::ScenarioLoader::loadFromFile(scenario_path, context)) {
            std::cout << "[Load Scenario] Successfully loaded: " << scenario_path << std::endl;
            std::cout << "  Ego pose: (" << context.ego.pose.x << ", "
                      << context.ego.pose.y << ", " << context.ego.pose.yaw << ")" << std::endl;
            std::cout << "  Goal pose: (" << context.task.goal_pose.x << ", "
                      << context.task.goal_pose.y << ", " << context.task.goal_pose.yaw << ")" << std::endl;

            // 重置规划器
            planner_manager.reset();

            // 清空之前的规划结果
            success = false;
            result = plugin::PlanningResult();

            // 标记需要重新规划（等待 Start 按钮）
            *need_replan = true;

            std::cout << "[Load Scenario] Scenario loaded, click Start to plan" << std::endl;
          } else {
            std::cerr << "[Load Scenario] Failed to load: " << scenario_path << std::endl;
          }
          std::cout << "[Load Scenario] Done\n" << std::endl;
        }
      }

      // 🔧 检查是否需要重新规划（Start 按钮或 Reset 按钮触发）
      if (*need_replan) {
        *need_replan = false;  // 重置标志

        std::cout << "\n[Planning] Running planner..." << std::endl;
        auto start_time = std::chrono::steady_clock::now();
        plugin::PlanningResult new_result;
        auto deadline = std::chrono::milliseconds(5000);
        bool plan_success = planner_manager.plan(context, deadline, new_result);
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        if (plan_success) {
          std::cout << "[Planning] Success!" << std::endl;
          std::cout << "  Trajectory points: " << new_result.trajectory.size() << std::endl;
          std::cout << "  Computation time: " << duration << " ms" << std::endl;
          success = true;
          result = new_result;
        } else {
          std::cout << "[Planning] Failed: " << new_result.failure_reason << std::endl;
          success = false;
        }
        std::cout << "[Planning] Done\n" << std::endl;
      }

      // 重新渲染所有数据
      g_visualizer->drawEgo(context.ego);
      g_visualizer->drawGoal(context.task.goal_pose);

      if (context.bev_obstacles) {
        g_visualizer->drawBEVObstacles(*context.bev_obstacles);
      }

      g_visualizer->drawDynamicObstacles(context.dynamic_obstacles);

      if (context.occupancy_grid) {
        g_visualizer->drawOccupancyGrid(*context.occupancy_grid);
      }

      if (success) {
        g_visualizer->drawTrajectory(result.trajectory, result.planner_name);

        // 可视化调试路径
        if (result.metadata.count("has_debug_paths") > 0 &&
            result.metadata.count("debug_paths_ptr") > 0) {
          auto* debug_paths_ptr = reinterpret_cast<std::vector<std::vector<planning::Pose2d>>*>(
              static_cast<uintptr_t>(result.metadata.at("debug_paths_ptr")));

          if (debug_paths_ptr && !debug_paths_ptr->empty()) {
            std::vector<std::string> path_names;
            std::vector<std::string> colors;

            if (result.planner_name == "TMPCPlanner") {
              std::vector<std::string>* path_types_ptr = nullptr;
              if (result.metadata.count("debug_path_types_ptr") > 0) {
                path_types_ptr = reinterpret_cast<std::vector<std::string>*>(
                    static_cast<uintptr_t>(result.metadata.at("debug_path_types_ptr")));
              }

              if (path_types_ptr && path_types_ptr->size() == debug_paths_ptr->size()) {
                int guidance_idx = 0;
                int mpc_idx = 0;
                int obs_idx = 0;

                for (const auto& type : *path_types_ptr) {
                  if (type == "guidance") {
                    path_names.push_back("TMPC Guidance " + std::to_string(guidance_idx++));
                    colors.push_back("dashed_cyan");
                  } else if (type == "mpc_candidate") {
                    path_names.push_back("TMPC MPC Candidate " + std::to_string(mpc_idx++));
                    colors.push_back("orange");
                  } else if (type == "reference") {
                    path_names.push_back("TMPC Reference Path");
                    colors.push_back("yellow");
                  } else if (type == "obstacle_prediction") {
                    path_names.push_back("TMPC Obstacle " + std::to_string(obs_idx++));
                    colors.push_back("red");
                  } else if (type == "past_trajectory") {
                    path_names.push_back("TMPC Past Trajectory");
                    colors.push_back("gray");
                  }
                }
              }
            }

            g_visualizer->drawDebugPaths(*debug_paths_ptr, path_names, colors);
          }
        }

        // 可视化近似圆
        if (result.planner_name == "TMPCPlanner" &&
            result.metadata.count("approximation_circles_ptr") > 0) {
          struct ApproximationCircle {
            double x, y, radius;
          };

          auto* circles_ptr = reinterpret_cast<std::vector<ApproximationCircle>*>(
              static_cast<uintptr_t>(result.metadata.at("approximation_circles_ptr")));

          if (circles_ptr && !circles_ptr->empty()) {
            std::vector<std::tuple<double, double, double>> circles;
            for (const auto& c : *circles_ptr) {
              circles.emplace_back(c.x, c.y, c.radius);
            }

            auto* imgui_viz_inner = dynamic_cast<viz::ImGuiVisualizer*>(g_visualizer.get());
            if (imgui_viz_inner) {
              imgui_viz_inner->drawApproximationCircles(circles);
            }
          }
        }

        g_visualizer->updatePlanningResult(result);
      }

      g_visualizer->endFrame();

      // 检查是否有新的目标点被设置
      planning::Pose2d new_goal;
      if (g_visualizer->hasNewGoal(new_goal)) {
        std::cout << "\n[Replanning] New goal detected: (" << new_goal.x << ", " << new_goal.y << ")" << std::endl;

        // 更新规划上下文中的目标点
        context.task.goal_pose = new_goal;

        // 更新可视化中显示的目标点
        g_visualizer->drawGoal(context.task.goal_pose);

        // 执行重规划
        std::cout << "[Replanning] Starting replanning..." << std::endl;
        auto start_time = std::chrono::steady_clock::now();

        plugin::PlanningResult new_result;
        auto deadline = std::chrono::milliseconds(5000);  // 5秒超时
        bool replan_success = planner_manager.plan(context, deadline, new_result);

        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        // 输出重规划结果
        std::cout << "[Replanning] Result:" << std::endl;
        std::cout << "  Success: " << (replan_success ? "yes" : "no") << std::endl;
        if (!replan_success) {
          std::cout << "  Failure reason: " << new_result.failure_reason << std::endl;
        } else {
          std::cout << "  Planner: " << new_result.planner_name << std::endl;
          std::cout << "  Trajectory points: " << new_result.trajectory.size() << std::endl;
          std::cout << "  Computation time: " << duration << " ms" << std::endl;
          std::cout << "  Total cost: " << new_result.total_cost << std::endl;

          // 更新成功状态和结果
          success = replan_success;
          result = new_result;

          // 重新绘制轨迹（包括调试路径）
          g_visualizer->drawTrajectory(result.trajectory, result.planner_name);

          // 可视化调试路径（支持 JPS 和 TMPC）
          if (result.metadata.count("has_debug_paths") > 0 &&
              result.metadata.count("debug_paths_ptr") > 0) {
            // Get debug paths from the pointer stored in metadata
            auto* debug_paths_ptr = reinterpret_cast<std::vector<std::vector<planning::Pose2d>>*>(
                static_cast<uintptr_t>(result.metadata.at("debug_paths_ptr")));

            if (debug_paths_ptr && !debug_paths_ptr->empty()) {
              std::cout << "[Replanning] Drawing " << debug_paths_ptr->size() << " debug paths for "
                        << result.planner_name << " visualization" << std::endl;

              std::vector<std::string> path_names;
              std::vector<std::string> colors;

              if (result.planner_name == "JPSPlanner") {
                path_names = {
                  "Raw JPS Path",
                  "Optimized Path",
                  "Sample Trajectory",
                  "MINCO Final",
                  "MINCO Stage1 (Preprocessing)",
                  "MINCO Stage2 (Main Opt)"
                };
                colors = {
                  "red",      // Raw JPS Path - 红色
                  "green",    // Optimized Path - 绿色
                  "blue",     // Sample Trajectory - 蓝色
                  "yellow",   // MINCO Final - 黄色（高对比度）
                  "magenta",  // MINCO Stage1 - 洋红色（高对比度）
                  "cyan"      // MINCO Stage2 - 青色（高对比度）
                };
              } else if (result.planner_name == "TMPCPlanner") {
                // TMPC paths - use path type information from metadata (same as above)
                std::vector<std::string>* path_types_ptr = nullptr;
                if (result.metadata.count("debug_path_types_ptr") > 0) {
                  path_types_ptr = reinterpret_cast<std::vector<std::string>*>(
                      static_cast<uintptr_t>(result.metadata.at("debug_path_types_ptr")));
                }

                if (path_types_ptr && path_types_ptr->size() == debug_paths_ptr->size()) {
                  int guidance_idx = 0;
                  int mpc_idx = 0;
                  int obs_idx = 0;

                  for (size_t i = 0; i < path_types_ptr->size(); ++i) {
                    const std::string& type = (*path_types_ptr)[i];

                    if (type == "reference") {
                      path_names.push_back("Reference Path");
                      colors.push_back("blue");
                    } else if (type == "guidance") {
                      path_names.push_back("Guidance Path " + std::to_string(guidance_idx++));
                      colors.push_back("dashed_cyan");
                    } else if (type == "mpc_candidate") {
                      path_names.push_back("MPC Candidate " + std::to_string(mpc_idx++));
                      colors.push_back("orange");
                    } else if (type == "obstacle_prediction") {
                      path_names.push_back("Obstacle " + std::to_string(obs_idx++) + " Prediction");
                      colors.push_back("red");
                    } else if (type == "past_trajectory") {
                      path_names.push_back("Past Trajectory");
                      colors.push_back("green");
                    } else {
                      path_names.push_back("Unknown Path");
                      colors.push_back("white");
                    }
                  }
                } else {
                  for (size_t i = 0; i < debug_paths_ptr->size(); ++i) {
                    path_names.push_back("TMPC Path " + std::to_string(i));
                    colors.push_back("white");
                  }
                }
              }

              g_visualizer->drawDebugPaths(*debug_paths_ptr, path_names, colors);
            }
          }

          // 更新可视化
          g_visualizer->updatePlanningResult(result);
        }
        std::cout << "[Replanning] Done\n" << std::endl;
      }

      // 小延迟以控制帧率
      std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    // 清理可视化资源
    g_visualizer->shutdown();
  } else if (args.visualize && !g_visualizer) {
    std::cout << "\nWarning: Visualization was requested but visualizer failed to initialize" << std::endl;
    std::cout << "Check if visualization is enabled in the build" << std::endl;
  }
  
  std::cout << "\n=== Done ===" << std::endl;
  return success ? 0 : 1;
}


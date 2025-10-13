#include "core/algorithm_manager.hpp"
#include "plugin/framework/perception_plugin_manager.hpp"
#include "plugin/framework/planner_plugin_manager.hpp"
#include "plugin/data/perception_input.hpp"
#include "plugin/data/planning_result.hpp"
#include "plugin/framework/plugin_init.hpp"
#include "plugin/framework/plugin_loader.hpp"
#include "plugin/framework/dynamic_plugin_loader.hpp"
#include "plugin/preprocessing/preprocessing.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>

namespace navsim {

AlgorithmManager::AlgorithmManager() : config_(Config{}) {}

AlgorithmManager::AlgorithmManager(const Config& config)
    : config_(config) {}

AlgorithmManager::~AlgorithmManager() = default;

bool AlgorithmManager::initialize() {
  try {
    std::cout << "[AlgorithmManager] Initializing with plugin system..." << std::endl;
    setupPluginSystem();

    std::cout << "[AlgorithmManager] Initialized successfully" << std::endl;
    std::cout << "  Primary planner: " << config_.primary_planner << std::endl;
    std::cout << "  Fallback planner: " << config_.fallback_planner << std::endl;
    std::cout << "  Max computation time: " << config_.max_computation_time_ms << " ms" << std::endl;

    return true;
  } catch (const std::exception& e) {
    std::cerr << "[AlgorithmManager] Initialization failed: " << e.what() << std::endl;
    return false;
  }
}

bool AlgorithmManager::process(const proto::WorldTick& world_tick,
                              std::chrono::milliseconds deadline,
                              proto::PlanUpdate& plan_update,
                              proto::EgoCmd& ego_cmd) {
  stats_.total_processed++;

  auto total_start = std::chrono::steady_clock::now();

  // Step 1: 前置处理（生成标准化的 PerceptionInput）
  auto preprocessing_start = std::chrono::steady_clock::now();

  // 创建前置处理管线并处理
  perception::PreprocessingPipeline preprocessing_pipeline;
  plugin::PerceptionInput perception_input = preprocessing_pipeline.process(world_tick);

  auto preprocessing_end = std::chrono::steady_clock::now();
  double preprocessing_time = std::chrono::duration<double, std::milli>(
      preprocessing_end - preprocessing_start).count();

  // Step 2: 感知插件处理
  auto perception_start = std::chrono::steady_clock::now();

  planning::PlanningContext context;
  // 复制基础数据到 context
  context.ego = perception_input.ego;
  context.task = perception_input.task;
  context.dynamic_obstacles = perception_input.dynamic_obstacles;

  bool perception_success = perception_plugin_manager_->process(perception_input, context);

  auto perception_end = std::chrono::steady_clock::now();
  double perception_time = std::chrono::duration<double, std::milli>(
      perception_end - perception_start).count();

  if (!perception_success) {
    stats_.perception_failures++;
    if (config_.verbose_logging) {
      std::cerr << "[AlgorithmManager] Perception plugin processing failed" << std::endl;
    }
    return false;
  }

  // Step 3: 规划器插件处理
  auto planning_start = std::chrono::steady_clock::now();

  auto remaining_time = deadline - std::chrono::duration_cast<std::chrono::milliseconds>(
      planning_start - total_start);

  plugin::PlanningResult planning_result;
  bool planning_success = planner_plugin_manager_->plan(context, remaining_time, planning_result);

  auto planning_end = std::chrono::steady_clock::now();
  double planning_time = std::chrono::duration<double, std::milli>(
      planning_end - planning_start).count();

  if (!planning_success) {
    stats_.planning_failures++;
    if (config_.verbose_logging) {
      std::cerr << "[AlgorithmManager] Planning failed" << std::endl;
    }
    return false;
  }

  // Step 4: 转换为 proto 格式
  plan_update.set_tick_id(world_tick.tick_id());
  plan_update.set_stamp(world_tick.stamp());

  for (const auto& point : planning_result.trajectory) {
    auto* traj_point = plan_update.add_trajectory();
    traj_point->set_x(point.pose.x);
    traj_point->set_y(point.pose.y);
    traj_point->set_yaw(point.pose.yaw);
    traj_point->set_t(point.time_from_start);
  }

  // 设置控制指令（简单版本：使用第一个轨迹点的加速度）
  if (!planning_result.trajectory.empty()) {
    ego_cmd.set_acceleration(planning_result.trajectory[0].acceleration);
    ego_cmd.set_steering(0.0);  // 简化：假设转向角为0
  }

  auto total_end = std::chrono::steady_clock::now();
  double total_time = std::chrono::duration<double, std::milli>(
      total_end - total_start).count();

  updateStatistics(total_time, perception_time, planning_time, true);

  if (config_.verbose_logging) {
    std::cout << "[AlgorithmManager] Processing successful (plugin system):" << std::endl;
    std::cout << "  Total time: " << total_time << " ms" << std::endl;
    std::cout << "  Preprocessing time: " << preprocessing_time << " ms" << std::endl;
    std::cout << "  Perception time: " << perception_time << " ms" << std::endl;
    std::cout << "  Planning time: " << planning_time << " ms" << std::endl;
    std::cout << "  Planner used: " << planning_result.planner_name << std::endl;
    std::cout << "  Trajectory points: " << planning_result.trajectory.size() << std::endl;
  }

  stats_.successful_processed++;
  return true;
}

void AlgorithmManager::updateConfig(const Config& config) {
  config_ = config;
  std::cout << "[AlgorithmManager] Reinitializing with new config..." << std::endl;
  initialize();
}

void AlgorithmManager::setBridge(Bridge* bridge) {
  bridge_ = bridge;
}

void AlgorithmManager::updateStatistics(double total_time, double perception_time,
                                       double planning_time, bool success) {
  // 使用移动平均更新统计信息
  double alpha = 0.1;  // 平滑因子

  stats_.avg_computation_time_ms =
    stats_.avg_computation_time_ms * (1.0 - alpha) + total_time * alpha;

  stats_.avg_perception_time_ms =
    stats_.avg_perception_time_ms * (1.0 - alpha) + perception_time * alpha;

  stats_.avg_planning_time_ms =
    stats_.avg_planning_time_ms * (1.0 - alpha) + planning_time * alpha;
}

void AlgorithmManager::setupPluginSystem() {
  // 0. 初始化所有插件
  plugin::initializeAllPlugins();

  // 0.1 动态加载插件（从配置文件）
  std::cout << "[AlgorithmManager] Loading plugins dynamically from config..." << std::endl;
  plugin::DynamicPluginLoader plugin_loader;

  // 添加插件搜索路径
  plugin_loader.addSearchPath("./build/plugins");
  plugin_loader.addSearchPath("./plugins");

  // 从配置文件加载插件
  std::string config_file = config_.config_file.empty() ? "config/default.json" : config_.config_file;
  int loaded_count = plugin_loader.loadPluginsFromConfig(config_file);
  std::cout << "[AlgorithmManager] Dynamically loaded " << loaded_count << " plugins" << std::endl;

  // 0.2 如果动态加载失败，回退到静态链接的插件
  if (loaded_count == 0) {
    std::cout << "[AlgorithmManager] No plugins loaded dynamically, using built-in plugins..." << std::endl;
    #ifdef BUILD_PLUGINS
    plugins::loadAllBuiltinPlugins();
    #endif
  }

  // 1. 创建感知插件管理器
  perception_plugin_manager_ = std::make_unique<plugin::PerceptionPluginManager>();

  // 创建插件配置
  std::vector<plugin::PerceptionPluginConfig> perception_configs;

  // GridMapBuilder 插件
  plugin::PerceptionPluginConfig grid_config;
  grid_config.name = "GridMapBuilder";
  grid_config.enabled = true;
  grid_config.priority = 100;
  grid_config.params = {
    {"resolution", 0.1},
    {"map_width", 100.0},
    {"map_height", 100.0},
    {"obstacle_cost", 100},
    {"inflation_radius", 0.5}
  };
  perception_configs.push_back(grid_config);

  // 加载插件
  perception_plugin_manager_->loadPlugins(perception_configs);
  perception_plugin_manager_->initialize();

  std::cout << "[AlgorithmManager] Perception plugin manager initialized with "
            << perception_configs.size() << " plugins" << std::endl;

  // 2. 创建规划器插件管理器
  planner_plugin_manager_ = std::make_unique<plugin::PlannerPluginManager>();

  // 创建规划器配置
  nlohmann::json planner_configs = {
    {"StraightLinePlanner", {
      {"default_velocity", 1.5},
      {"time_step", 0.1},
      {"planning_horizon", 5.0},
      {"use_trapezoidal_profile", true},
      {"max_acceleration", 1.0}
    }},
    {"AStarPlanner", {
      {"time_step", 0.1},
      {"heuristic_weight", 1.2},
      {"step_size", 0.5},
      {"max_iterations", 10000},
      {"goal_tolerance", 0.5},
      {"default_velocity", 1.5}
    }}
  };

  // 加载规划器（使用配置中的规划器名称）
  planner_plugin_manager_->loadPlanners(
      config_.primary_planner,   // 主规划器（从配置读取）
      config_.fallback_planner,  // 降级规划器（从配置读取）
      true,                      // 启用降级
      planner_configs);
  planner_plugin_manager_->initialize();

  std::cout << "[AlgorithmManager] Planner plugin manager initialized" << std::endl;
  std::cout << "  Primary planner: " << planner_plugin_manager_->getPrimaryPlannerName() << std::endl;
  std::cout << "  Fallback planner: " << planner_plugin_manager_->getFallbackPlannerName() << std::endl;
}


} // namespace navsim
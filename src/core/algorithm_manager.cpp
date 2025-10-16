#include "core/algorithm_manager.hpp"
#include "plugin/framework/perception_plugin_manager.hpp"
#include "core/bridge.hpp"
#include "plugin/framework/planner_plugin_manager.hpp"
#include "plugin/data/perception_input.hpp"
#include "plugin/data/planning_result.hpp"
#include "plugin/framework/plugin_init.hpp"
#include "plugin/framework/plugin_loader.hpp"
#include "plugin/framework/dynamic_plugin_loader.hpp"
#include "plugin/framework/config_loader.hpp"
#include "plugin/preprocessing/preprocessing.hpp"
#include "viz/visualizer_interface.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

namespace navsim {

AlgorithmManager::AlgorithmManager() : config_(Config{}) {}

AlgorithmManager::AlgorithmManager(const Config& config)
    : config_(config) {}

AlgorithmManager::~AlgorithmManager() = default;

bool AlgorithmManager::initialize() {
  try {
    std::cout << "[AlgorithmManager] Initializing with plugin system..." << std::endl;
    setupPluginSystem();

    // 初始化可视化器
    if (config_.enable_visualization) {
      std::cout << "[AlgorithmManager] Initializing visualizer..." << std::endl;
      visualizer_ = viz::createVisualizer(true);
      if (visualizer_ && visualizer_->initialize()) {
        std::cout << "[AlgorithmManager] Visualizer initialized successfully" << std::endl;
      } else {
        std::cerr << "[AlgorithmManager] Failed to initialize visualizer" << std::endl;
        visualizer_.reset();
      }
    } else {
      visualizer_ = viz::createVisualizer(false);  // NullVisualizer
    }

    std::cout << "[AlgorithmManager] Initialized successfully" << std::endl;
    std::cout << "  Primary planner: " << config_.primary_planner << std::endl;
    std::cout << "  Fallback planner: " << config_.fallback_planner << std::endl;
    std::cout << "  Max computation time: " << config_.max_computation_time_ms << " ms" << std::endl;
    std::cout << "  Visualization: " << (config_.enable_visualization ? "ENABLED" : "DISABLED") << std::endl;

    if (visualizer_) {
      system_info_cache_.general.clear();
      system_info_cache_.perception_plugins.clear();
      system_info_cache_.planner_plugins.clear();

      system_info_cache_.general["Config File"] = active_config_file_.empty()
        ? "config/default.json"
        : active_config_file_;
      system_info_cache_.general["Visualizer"] = "ImGui (SDL2/OpenGL2)";
      system_info_cache_.general["Primary Planner"] = planner_plugin_manager_
        ? planner_plugin_manager_->getPrimaryPlannerName()
        : config_.primary_planner;
      system_info_cache_.general["Fallback Planner"] = planner_plugin_manager_
        ? planner_plugin_manager_->getFallbackPlannerName()
        : config_.fallback_planner;
      system_info_cache_.general["Fallback Enabled"] = planner_plugin_manager_ && planner_plugin_manager_->isFallbackEnabled()
        ? "Yes"
        : (config_.enable_planner_fallback ? "Yes" : "No");
      {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << config_.max_computation_time_ms;
        system_info_cache_.general["Max Computation Time"] = oss.str() + " ms";
      }
      system_info_cache_.general["Visualization"] = config_.enable_visualization ? "Enabled" : "Disabled";
      if (!connection_label_.empty()) {
        system_info_cache_.general["Connection Target"] = connection_label_;
      }

      if (perception_plugin_manager_) {
        const auto& plugin_configs = perception_plugin_manager_->getPluginConfigs();
        for (const auto& plugin_config : plugin_configs) {
          std::ostringstream oss;
          oss << plugin_config.name
              << " (priority=" << plugin_config.priority << ")";
          oss << (plugin_config.enabled ? " [ENABLED]" : " [DISABLED]");
          if (!plugin_config.params.is_null() && !plugin_config.params.empty()) {
            oss << " params=" << plugin_config.params.dump();
          }
          system_info_cache_.perception_plugins.push_back(oss.str());
        }
      }

      if (planner_plugin_manager_) {
        system_info_cache_.planner_plugins.push_back(
          "Primary: " + planner_plugin_manager_->getPrimaryPlannerName());
        if (planner_plugin_manager_->isFallbackEnabled() &&
            !planner_plugin_manager_->getFallbackPlannerName().empty()) {
          system_info_cache_.planner_plugins.push_back(
            "Fallback: " + planner_plugin_manager_->getFallbackPlannerName());
        }
      } else {
        system_info_cache_.planner_plugins.push_back("Primary: " + config_.primary_planner);
        if (config_.enable_planner_fallback) {
          system_info_cache_.planner_plugins.push_back("Fallback: " + config_.fallback_planner);
        }
      }

      visualizer_->setSystemInfo(system_info_cache_);

      viz::IVisualizer::ConnectionStatus connection_status;
      connection_status.connected = bridge_ && bridge_->is_connected();
      connection_status.label = connection_label_;
      connection_status.message = connection_status.connected ? "Connected" : "Bridge not connected";
      visualizer_->updateConnectionStatus(connection_status);
    }

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

  // 🔧 检查仿真是否已开始
  if (!simulation_started_.load()) {
    // 仿真未开始，只更新可视化，不执行算法
    if (visualizer_) {
      visualizer_->beginFrame();

      viz::IVisualizer::ConnectionStatus connection_status;
      connection_status.connected = bridge_ && bridge_->is_connected();
      connection_status.label = connection_label_;
      connection_status.message = "⏸️ Waiting for simulation to start...";
      visualizer_->updateConnectionStatus(connection_status);
      visualizer_->showDebugInfo("Status", "⏸️ Waiting for START button");
      visualizer_->showDebugInfo("Tick ID", std::to_string(world_tick.tick_id()));
      {
        std::ostringstream stamp_stream;
        stamp_stream << std::fixed << std::setprecision(3) << world_tick.stamp();
        visualizer_->showDebugInfo("Stamp", stamp_stream.str());
      }

      // 结束可视化帧
      visualizer_->endFrame();
    }

    // 返回空的 PlanUpdate（不执行算法）
    plan_update.set_tick_id(world_tick.tick_id());
    plan_update.set_stamp(world_tick.stamp());
    return false;  // 返回 false 表示未处理
  }

  // 🎨 开始新的可视化帧
  if (visualizer_) {
    visualizer_->beginFrame();

    viz::IVisualizer::ConnectionStatus connection_status;
    connection_status.connected = bridge_ && bridge_->is_connected();
    connection_status.label = connection_label_;
    connection_status.message = connection_status.connected
      ? "✅ Processing world_tick"
      : "Bridge disconnected";
    visualizer_->updateConnectionStatus(connection_status);
    visualizer_->showDebugInfo("Status", connection_status.connected ? "✅ Processing" : "No bridge connection");
    visualizer_->showDebugInfo("Tick ID", std::to_string(world_tick.tick_id()));
    {
      std::ostringstream stamp_stream;
      stamp_stream << std::fixed << std::setprecision(3) << world_tick.stamp();
      visualizer_->showDebugInfo("Stamp", stamp_stream.str());
    }
  }

  auto total_start = std::chrono::steady_clock::now();

  // Step 1: 前置处理（生成标准化的 PerceptionInput）
  auto preprocessing_start = std::chrono::steady_clock::now();

  // 创建前置处理管线并处理
  perception::PreprocessingPipeline preprocessing_pipeline;
  plugin::PerceptionInput perception_input = preprocessing_pipeline.process(world_tick);

  auto preprocessing_end = std::chrono::steady_clock::now();
  double preprocessing_time = std::chrono::duration<double, std::milli>(
      preprocessing_end - preprocessing_start).count();

  // 🔍 调试日志：检查 perception_input 中的障碍物数据
  std::cout << "[AlgorithmManager] ========== Perception Input Check ==========" << std::endl;
  std::cout << "[AlgorithmManager] BEV obstacles in perception_input:" << std::endl;
  std::cout << "[AlgorithmManager]   Circles: " << perception_input.bev_obstacles.circles.size() << std::endl;
  std::cout << "[AlgorithmManager]   Rectangles: " << perception_input.bev_obstacles.rectangles.size() << std::endl;
  std::cout << "[AlgorithmManager]   Polygons: " << perception_input.bev_obstacles.polygons.size() << std::endl;

  // 🎨 可视化感知输入数据
  if (visualizer_) {
    std::cout << "[AlgorithmManager] Calling visualizer->drawBEVObstacles()..." << std::endl;
    visualizer_->drawEgo(perception_input.ego);
    visualizer_->drawGoal(perception_input.task.goal_pose);
    visualizer_->drawBEVObstacles(perception_input.bev_obstacles);

    std::cout << "[AlgorithmManager] Calling visualizer->drawDynamicObstacles() with "
              << perception_input.dynamic_obstacles.size() << " obstacles..." << std::endl;
    // 🔧 修复问题1：打印所有障碍物的信息
    for (size_t i = 0; i < perception_input.dynamic_obstacles.size(); ++i) {
      const auto& obs = perception_input.dynamic_obstacles[i];
      std::cout << "[AlgorithmManager]   Dyn obs #" << i << ": shape=" << obs.shape_type
                << ", pos=(" << obs.current_pose.x << ", " << obs.current_pose.y
                << "), length=" << obs.length << ", width=" << obs.width << std::endl;
    }
    visualizer_->drawDynamicObstacles(perception_input.dynamic_obstacles);
    std::cout << "[AlgorithmManager] Visualizer calls completed" << std::endl;
  }

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

  if (visualizer_) {
    visualizer_->updatePlanningContext(context);
  }

  if (!perception_success) {
    stats_.perception_failures++;
    if (config_.verbose_logging) {
      std::cerr << "[AlgorithmManager] Perception plugin processing failed" << std::endl;
    }
    // 🎨 结束帧（即使失败也要渲染）
    if (visualizer_) {
      plugin::PlanningResult failure_result;
      failure_result.success = false;
      failure_result.failure_reason = "Perception Failed";
      visualizer_->updatePlanningResult(failure_result);
      visualizer_->showDebugInfo("Status", "Perception Failed");
      visualizer_->endFrame();
    }
    return false;
  }

  // 🎨 可视化感知处理结果（如栅格地图）
  if (visualizer_ && context.occupancy_grid) {
    visualizer_->drawOccupancyGrid(*context.occupancy_grid);
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
    if (planning_result.failure_reason.empty()) {
      planning_result.failure_reason = "Planner returned false";
    }
    // 🎨 结束帧（即使失败也要渲染）
    if (visualizer_) {
      visualizer_->updatePlanningResult(planning_result);
      visualizer_->showDebugInfo("Status", "Planning Failed");
      visualizer_->endFrame();
    }
    return false;
  }

  // 🎨 可视化规划结果
  if (visualizer_) {
    visualizer_->updatePlanningResult(planning_result);
    visualizer_->drawTrajectory(planning_result.trajectory, planning_result.planner_name);
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

  // 🎨 显示性能调试信息
  if (visualizer_) {
    viz::IVisualizer::ConnectionStatus connection_status;
    connection_status.connected = bridge_ && bridge_->is_connected();
    connection_status.label = connection_label_;
    connection_status.message = connection_status.connected ? "Last tick processed" : "Bridge disconnected";
    visualizer_->updateConnectionStatus(connection_status);

    auto format_ms = [](double value) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << value;
      return oss.str();
    };

    visualizer_->showDebugInfo("Status", "Success");
    visualizer_->showDebugInfo("Total Time", format_ms(total_time) + " ms");
    visualizer_->showDebugInfo("Preprocessing", format_ms(preprocessing_time) + " ms");
    visualizer_->showDebugInfo("Perception", format_ms(perception_time) + " ms");
    visualizer_->showDebugInfo("Planning", format_ms(planning_time) + " ms");
  }

  // 🎨 结束帧并渲染
  if (visualizer_) {
    visualizer_->endFrame();
  }

  // 🔧 发送感知调试数据到前端（如果 Bridge 已连接且启用）
  // 为了避免数据量过大导致卡顿，降低发送频率（每 10 帧发送一次）
  static int perception_debug_counter = 0;
  if (bridge_ && bridge_->is_connected() && bridge_->is_perception_debug_enabled()) {
    if (++perception_debug_counter >= 10) {
      bridge_->send_perception_debug(context);
      perception_debug_counter = 0;
    }
  }

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

void AlgorithmManager::setBridge(Bridge* bridge, const std::string& connection_label) {
  bridge_ = bridge;
  connection_label_ = connection_label;

  if (!connection_label_.empty()) {
    system_info_cache_.general["Connection Target"] = connection_label_;
    if (visualizer_) {
      visualizer_->setSystemInfo(system_info_cache_);
    }
  }

  if (visualizer_) {
    viz::IVisualizer::ConnectionStatus status;
    status.connected = bridge_ && bridge_->is_connected();
    status.label = connection_label_;
    status.message = status.connected ? "Connected" : "Waiting for connection";
    visualizer_->updateConnectionStatus(status);
  }
}

void AlgorithmManager::renderIdleFrame() {
  if (!visualizer_) {
    return;
  }

  visualizer_->beginFrame();

  viz::IVisualizer::ConnectionStatus status;
  status.connected = bridge_ && bridge_->is_connected();
  status.label = connection_label_;
  status.message = status.connected ? "Waiting for world_tick..." : "Waiting for connection";
  visualizer_->updateConnectionStatus(status);
  visualizer_->showDebugInfo("Status", status.connected ? "Waiting for world_tick..." : "Bridge disconnected");

  visualizer_->endFrame();
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
  active_config_file_ = config_file;
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

  // 从配置加载器获取插件配置
  std::vector<plugin::PerceptionPluginConfig> perception_configs;

  if (plugin_loader.getConfigLoader()) {
    // 使用配置文件中的插件列表
    perception_configs = plugin_loader.getConfigLoader()->getPerceptionPluginConfigs();
    std::cout << "[AlgorithmManager] Loaded " << perception_configs.size()
              << " perception plugin configs from file" << std::endl;
  }

  // 如果配置文件中没有插件配置，使用默认的 GridMapBuilder
  if (perception_configs.empty()) {
    std::cout << "[AlgorithmManager] No perception plugins in config, using default GridMapBuilder" << std::endl;
    plugin::PerceptionPluginConfig grid_config;
    grid_config.name = "GridMapBuilder";
    grid_config.enabled = true;
    grid_config.priority = 100;
    grid_config.params = {
      {"resolution", config_.grid_resolution},
      {"map_width", config_.grid_map_width},
      {"map_height", config_.grid_map_height},
      {"obstacle_cost", 100},
      {"inflation_radius", config_.grid_inflation_radius}
    };
    perception_configs.push_back(grid_config);

    std::cout << "[AlgorithmManager] GridMapBuilder config:" << std::endl;
    std::cout << "  - map_width: " << config_.grid_map_width << " m" << std::endl;
    std::cout << "  - map_height: " << config_.grid_map_height << " m" << std::endl;
    std::cout << "  - resolution: " << config_.grid_resolution << " m/cell" << std::endl;
    std::cout << "  - inflation_radius: " << config_.grid_inflation_radius << " m" << std::endl;
  }

  // 加载插件
  perception_plugin_manager_->loadPlugins(perception_configs);
  perception_plugin_manager_->initialize();

  std::cout << "[AlgorithmManager] Perception plugin manager initialized with "
            << perception_configs.size() << " plugins" << std::endl;

  // 2. 创建规划器插件管理器
  planner_plugin_manager_ = std::make_unique<plugin::PlannerPluginManager>();

  // 创建规划器配置
  nlohmann::json planner_configs;

  // 从配置加载器获取规划器配置
  if (plugin_loader.getConfigLoader()) {
    planner_configs = plugin_loader.getConfigLoader()->getPlannerConfigs();

    // 更新主规划器和降级规划器名称
    std::string primary_planner = plugin_loader.getConfigLoader()->getPrimaryPlannerName();
    std::string fallback_planner = plugin_loader.getConfigLoader()->getFallbackPlannerName();

    if (!primary_planner.empty()) {
      config_.primary_planner = primary_planner;
    }
    if (!fallback_planner.empty()) {
      config_.fallback_planner = fallback_planner;
    }

    std::cout << "[AlgorithmManager] Loaded planner configs from file" << std::endl;
    std::cout << "[AlgorithmManager] Primary planner from config: " << config_.primary_planner << std::endl;
    std::cout << "[AlgorithmManager] Fallback planner from config: " << config_.fallback_planner << std::endl;
  }

  // 如果配置文件中没有规划器配置，使用默认配置
  if (planner_configs.empty()) {
    planner_configs = {
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
  }

  // 加载规划器（使用配置中的规划器名称）
  planner_plugin_manager_->loadPlanners(
      config_.primary_planner,   // 主规划器（从配置读取）
      config_.fallback_planner,  // 降级规划器（从配置读取）
      config_.enable_planner_fallback,  // 启用降级
      planner_configs);
  planner_plugin_manager_->initialize();

  std::cout << "[AlgorithmManager] Planner plugin manager initialized" << std::endl;
  std::cout << "  Primary planner: " << planner_plugin_manager_->getPrimaryPlannerName() << std::endl;
  std::cout << "  Fallback planner: " << planner_plugin_manager_->getFallbackPlannerName() << std::endl;
}


} // namespace navsim

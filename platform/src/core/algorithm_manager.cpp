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
#include "viz/imgui_visualizer.hpp"
#include "sim/local_simulator.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <thread>
#include <atomic>

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

        // 🎮 设置仿真控制回调
        auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(visualizer_.get());
        if (imgui_viz) {
          imgui_viz->setSimulationControlCallbacks(
            [this]() { this->startSimulation(); },   // Start callback
            [this]() { this->pauseSimulation(); },   // Pause callback
            [this]() { this->resetSimulation(); }    // Reset callback
          );
          // 初始状态为暂停
          imgui_viz->updateSimulationStatus(true);
          std::cout << "[AlgorithmManager] Simulation control callbacks set" << std::endl;
        }
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
    if (visualizer_ && !use_local_simulator_) {
      // 只在非本地仿真模式下管理帧（本地仿真模式由 process_simulation_step 管理）
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

  // 🎨 开始新的可视化帧（仅在非本地仿真模式）
  if (visualizer_ && !use_local_simulator_) {
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
    // 🎨 结束帧（即使失败也要渲染）- 仅在非本地仿真模式
    if (visualizer_ && !use_local_simulator_) {
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
    // 🎨 结束帧（即使失败也要渲染）- 仅在非本地仿真模式
    if (visualizer_ && !use_local_simulator_) {
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
    // Pose
    traj_point->set_x(point.pose.x);
    traj_point->set_y(point.pose.y);
    traj_point->set_yaw(point.pose.yaw);

    // Time
    traj_point->set_t(point.time_from_start);

    // Twist (velocity)
    traj_point->set_vx(point.twist.vx);
    traj_point->set_vy(point.twist.vy);
    traj_point->set_omega(point.twist.omega);

    // Acceleration
    traj_point->set_acceleration(point.acceleration);

    // Curvature
    traj_point->set_curvature(point.curvature);

    // Path length
    traj_point->set_path_length(point.path_length);
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

  // 🎨 结束帧并渲染 - 仅在非本地仿真模式
  if (visualizer_ && !use_local_simulator_) {
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

void AlgorithmManager::reset() {
  std::cout << "[AlgorithmManager] Resetting all plugins..." << std::endl;

  // 重置感知插件
  if (perception_plugin_manager_) {
    perception_plugin_manager_->reset();
  }

  // 重置规划器插件
  if (planner_plugin_manager_) {
    planner_plugin_manager_->reset();
  }

  // 重置统计信息
  resetStatistics();

  std::cout << "[AlgorithmManager] All plugins reset successfully" << std::endl;
}

void AlgorithmManager::performFullReset() {
  std::cout << "[AlgorithmManager] Performing full system reset..." << std::endl;

  // 1. 重置所有插件（清空内部状态和缓存）
  reset();

  // 2. 重置 LocalSimulator（恢复到初始状态）
  if (local_simulator_) {
    std::cout << "[AlgorithmManager] Resetting LocalSimulator..." << std::endl;
    local_simulator_->reset();
  }

  // 3. 清空可视化器的缓存数据
  if (visualizer_) {
    std::cout << "[AlgorithmManager] Clearing visualizer cache..." << std::endl;
    // 发送空的规划结果以清空轨迹显示
    plugin::PlanningResult empty_result;
    empty_result.success = false;
    empty_result.planner_name = "";
    visualizer_->updatePlanningResult(empty_result);
  }

  std::cout << "[AlgorithmManager] Full system reset complete" << std::endl;
}

bool AlgorithmManager::loadScenario(const std::string& scenario_file) {
  std::cout << "[AlgorithmManager] Loading scenario: " << scenario_file << std::endl;

  // 1. 检查文件是否存在
  std::ifstream file(scenario_file);
  if (!file.good()) {
    std::cerr << "[AlgorithmManager] Scenario file not found: " << scenario_file << std::endl;
    return false;
  }
  file.close();

  // 2. 停止当前仿真循环（如果正在运行）
  bool was_running = !simulation_should_stop_.load();
  if (was_running) {
    std::cout << "[AlgorithmManager] Stopping current simulation..." << std::endl;
    stop_simulation_loop();
    // 等待仿真循环停止
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 3. 执行完整的系统重置
  performFullReset();

  // 4. 重新加载场景到仿真器
  if (local_simulator_) {
    std::cout << "[AlgorithmManager] Loading scenario into simulator..." << std::endl;
    if (!local_simulator_->load_scenario(scenario_file)) {
      std::cerr << "[AlgorithmManager] Failed to load scenario into simulator" << std::endl;
      return false;
    }
  } else {
    std::cerr << "[AlgorithmManager] No local simulator available" << std::endl;
    return false;
  }

  // 5. 保存当前场景文件路径
  current_scenario_file_ = scenario_file;

  // 6. 重新开始仿真（如果之前在运行）
  if (was_running) {
    std::cout << "[AlgorithmManager] Restarting simulation with new scenario..." << std::endl;
    simulation_should_stop_.store(false);
    // 注意：这里不调用 run_simulation_loop()，因为它会阻塞
    // 仿真循环会在下一次迭代时自动继续
  }

  // 7. 加载新场景后默认暂停，等待用户点击 Start
  simulation_paused_.store(true);

  std::cout << "[AlgorithmManager] Scenario loaded successfully: " << scenario_file << std::endl;
  return true;
}

void AlgorithmManager::startSimulation() {
  simulation_paused_.store(false);
  std::cout << "[AlgorithmManager] Simulation started/resumed" << std::endl;

  // 启动 LocalSimulator（如果有）
  if (local_simulator_) {
    local_simulator_->start();
  }

  // 更新可视化器状态
  if (visualizer_) {
    auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(visualizer_.get());
    if (imgui_viz) {
      imgui_viz->updateSimulationStatus(false);
    }
  }
}

void AlgorithmManager::pauseSimulation() {
  simulation_paused_.store(true);
  std::cout << "[AlgorithmManager] Simulation paused" << std::endl;

  // 暂停 LocalSimulator（如果有）
  if (local_simulator_) {
    local_simulator_->pause();
  }

  // 更新可视化器状态
  if (visualizer_) {
    auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(visualizer_.get());
    if (imgui_viz) {
      imgui_viz->updateSimulationStatus(true);
    }
  }
}

void AlgorithmManager::resetSimulation() {
  std::cout << "[AlgorithmManager] Resetting simulation..." << std::endl;

  // 1. 暂停仿真
  pauseSimulation();

  // 2. 执行完整的系统重置
  performFullReset();

  // 3. 重新加载当前场景（如果有）
  if (!current_scenario_file_.empty() && local_simulator_) {
    std::cout << "[AlgorithmManager] Reloading current scenario: " << current_scenario_file_ << std::endl;
    if (!local_simulator_->load_scenario(current_scenario_file_)) {
      std::cerr << "[AlgorithmManager] Failed to reload scenario" << std::endl;
    }
  }

  std::cout << "[AlgorithmManager] Simulation reset complete (paused, waiting for Start)" << std::endl;
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
      {"StraightLine", {  // 注意：插件注册名称是 "StraightLine"，不是 "StraightLinePlanner"
        {"default_velocity", 1.5},
        {"time_step", 0.1},
        {"planning_horizon", 5.0},
        {"use_trapezoidal_profile", true},
        {"max_acceleration", 1.0}
      }},
      {"AstarPlanner", {  // 注意：插件注册名称是 "AstarPlanner"，不是 "AStarPlanner"
        {"time_step", 0.1},
        {"heuristic_weight", 1.2},
        {"step_size", 0.5},
        {"max_iterations", 10000},
        {"goal_tolerance", 0.5},
        {"default_velocity", 1.5}
      }},
      {"JpsPlanner", {
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

// ========== 本地仿真集成方法 ==========

bool AlgorithmManager::initialize_with_simulator(const Config& config) {
  config_ = config;
  use_local_simulator_ = true;

  std::cout << "[AlgorithmManager] Initializing with LocalSimulator..." << std::endl;

  // 使用标准初始化流程
  if (!initialize()) {
    std::cerr << "[AlgorithmManager] Failed to initialize plugin system" << std::endl;
    return false;
  }

  std::cout << "[AlgorithmManager] LocalSimulator mode enabled" << std::endl;
  return true;
}

void AlgorithmManager::set_local_simulator(std::shared_ptr<sim::LocalSimulator> simulator) {
  local_simulator_ = simulator;
  use_local_simulator_ = true;

  if (local_simulator_) {
    std::cout << "[AlgorithmManager] LocalSimulator attached successfully" << std::endl;
  } else {
    std::cerr << "[AlgorithmManager] Warning: NULL LocalSimulator provided" << std::endl;
  }
}

void AlgorithmManager::set_current_scenario(const std::string& scenario_file) {
  current_scenario_file_ = scenario_file;
  std::cout << "[AlgorithmManager] Current scenario set to: " << scenario_file << std::endl;
}

bool AlgorithmManager::run_simulation_loop(const std::atomic<bool>* external_interrupt) {
  if (!local_simulator_) {
    std::cerr << "[AlgorithmManager] LocalSimulator not set" << std::endl;
    return false;
  }

  if (!use_local_simulator_) {
    std::cerr << "[AlgorithmManager] Not in LocalSimulator mode" << std::endl;
    return false;
  }

  std::cout << "[AlgorithmManager] Starting local simulation loop..." << std::endl;
  std::cout << "[AlgorithmManager] Press Ctrl+C to stop" << std::endl;

  // 重置停止标志
  simulation_should_stop_.store(false);

  // 设置仿真已开始标志（本地仿真模式自动开始）
  simulation_started_.store(true);

  // 启动仿真
  local_simulator_->start();

  // 仿真主循环
  const double target_frequency = 30.0;  // 30Hz 主循环
  const auto loop_period = std::chrono::duration<double>(1.0 / target_frequency);

  auto last_step_time = std::chrono::steady_clock::now();

  // 🎯 性能监控
  auto last_fps_update = std::chrono::steady_clock::now();
  int frame_count = 0;
  double current_fps = 0.0;

  while (!simulation_should_stop_.load()) {
    // 🎨 检查可视化窗口是否被关闭
    if (visualizer_ && visualizer_->shouldClose()) {
      std::cout << "[AlgorithmManager] Visualizer window closed, stopping simulation..." << std::endl;
      break;
    }

    // 🛑 检查外部中断信号（Ctrl+C）
    if (external_interrupt && external_interrupt->load()) {
      std::cout << "[AlgorithmManager] External interrupt received, stopping simulation..." << std::endl;
      break;
    }

    // 🔧 检查场景加载请求
    if (visualizer_) {
      // 尝试将 visualizer_ 转换为 ImGuiVisualizer
      auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(visualizer_.get());
      if (imgui_viz) {
        std::string scenario_path;
        if (imgui_viz->hasScenarioLoadRequest(scenario_path)) {
          std::cout << "[AlgorithmManager] Scenario load request received: " << scenario_path << std::endl;
          if (loadScenario(scenario_path)) {
            std::cout << "[AlgorithmManager] Scenario loaded successfully, continuing simulation..." << std::endl;
          } else {
            std::cerr << "[AlgorithmManager] Failed to load scenario: " << scenario_path << std::endl;
          }
        }
      }
    }

    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = current_time - last_step_time;

    // 🎮 检查仿真是否暂停
    if (simulation_paused_.load()) {
      // 暂停时仍然渲染可视化界面，但不执行仿真步进
      if (visualizer_) {
        visualizer_->beginFrame();
        visualizer_->showDebugInfo("Simulation Status", "PAUSED");
        visualizer_->endFrame();
      }

      // 短暂休眠避免CPU占用过高
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;  // 跳过本次循环，不执行仿真步进
    }

    // 控制循环频率
    if (elapsed >= loop_period) {
      double dt = std::chrono::duration<double>(elapsed).count();

      // 处理单步仿真
      if (!process_simulation_step(dt)) {
        std::cerr << "[AlgorithmManager] Simulation step failed" << std::endl;
        break;
      }

      last_step_time = current_time;
      frame_count++;

      // 🎯 每秒更新一次FPS
      auto fps_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_fps_update).count();
      if (fps_elapsed >= 1000) {
        current_fps = frame_count * 1000.0 / fps_elapsed;

        if (visualizer_) {
          std::ostringstream fps_stream;
          fps_stream << std::fixed << std::setprecision(1) << current_fps << " Hz";
          visualizer_->showDebugInfo("Loop Frequency", fps_stream.str());
          visualizer_->showDebugInfo("Simulation Status", "RUNNING");
        }

        frame_count = 0;
        last_fps_update = current_time;
      }
    }

    // 短暂休眠避免CPU占用过高
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  std::cout << "[AlgorithmManager] Simulation loop ended" << std::endl;
  return true;
}

void AlgorithmManager::stop_simulation_loop() {
  simulation_should_stop_.store(true);
  std::cout << "[AlgorithmManager] Stopping simulation loop..." << std::endl;
}

bool AlgorithmManager::process_simulation_step(double dt) {
  if (!local_simulator_) {
    return false;
  }

  // 🎨 开始新的可视化帧
  if (visualizer_) {
    visualizer_->beginFrame();
  }

  // 1. 获取当前世界状态（在仿真步进之前）
  const auto& world_state = local_simulator_->get_world_state();

  // 🕐 更新仿真时间到可视化器
  if (visualizer_) {
    double sim_time = local_simulator_->get_simulation_time();
    std::ostringstream time_stream;
    time_stream << std::fixed << std::setprecision(3) << sim_time << "s";
    visualizer_->showDebugInfo("Simulation Time", time_stream.str());

    // 同时显示帧ID
    visualizer_->showDebugInfo("Frame ID", std::to_string(local_simulator_->get_frame_id()));
  }

  // 2. 转换为protobuf格式
  auto world_tick = local_simulator_->to_world_tick();

  // 3. 运行算法处理
  proto::PlanUpdate plan_update;
  proto::EgoCmd ego_cmd;
  auto deadline = std::chrono::milliseconds(static_cast<int>(config_.max_computation_time_ms));

  bool planning_success = process(world_tick, deadline, plan_update, ego_cmd);

  // 4. 将规划结果应用到仿真器
  if (planning_success && plan_update.trajectory_size() > 0) {
    // 计算当前应该跟踪的轨迹点索引
    // 使用简单的时间索引：假设轨迹点间隔为 dt
    static double accumulated_time = 0.0;
    accumulated_time += dt;

    // 根据累积时间找到对应的轨迹点
    int target_index = 0;
    double min_time_diff = std::abs(plan_update.trajectory(0).t() - accumulated_time);

    for (int i = 1; i < plan_update.trajectory_size(); ++i) {
      double time_diff = std::abs(plan_update.trajectory(i).t() - accumulated_time);
      if (time_diff < min_time_diff) {
        min_time_diff = time_diff;
        target_index = i;
      } else {
        // 时间差开始增大，说明已经找到最接近的点
        break;
      }
    }

    // 限制索引范围，避免跟踪太远的点
    // 使用前瞻时间：0.1秒（约3个仿真步）
    const double lookahead_time = 0.1;
    int lookahead_index = 0;
    for (int i = 0; i < plan_update.trajectory_size(); ++i) {
      if (plan_update.trajectory(i).t() >= lookahead_time) {
        lookahead_index = i;
        break;
      }
    }

    // 使用前瞻点或当前时间点（取较大者）
    int control_index = std::max(lookahead_index, std::min(target_index, 10));

    const auto& control_point = plan_update.trajectory(control_index);

    // 更新自车位置和速度（简单的轨迹跟踪）
    planning::Pose2d new_pose;
    new_pose.x = control_point.x();
    new_pose.y = control_point.y();
    new_pose.yaw = control_point.yaw();

    planning::Twist2d new_twist;
    new_twist.vx = control_point.vx();
    new_twist.vy = control_point.vy();
    new_twist.omega = control_point.omega();

    // 应用到仿真器
    local_simulator_->set_ego_pose(new_pose);
    local_simulator_->set_ego_twist(new_twist);

    if (config_.verbose_logging && world_state.frame_id % 30 == 0) {  // 每秒打印一次
      std::cout << "[AlgorithmManager] Step " << world_state.frame_id
                << ": Planning success, " << plan_update.trajectory_size()
                << " trajectory points generated" << std::endl;
      std::cout << "  Control index: " << control_index
                << " (t=" << control_point.t() << "s, accumulated_time="
                << accumulated_time << "s)" << std::endl;
      std::cout << "  Ego pose: (" << new_pose.x << ", " << new_pose.y
                << ", " << new_pose.yaw << ")" << std::endl;
      std::cout << "  Ego twist: (" << new_twist.vx << ", " << new_twist.vy
                << ", " << new_twist.omega << ")" << std::endl;
    }
  }

  // 5. 执行仿真步进（应用新的状态）
  if (!local_simulator_->step(dt)) {
    std::cerr << "[AlgorithmManager] Simulator step failed" << std::endl;
    // 🎨 即使失败也要结束帧
    if (visualizer_) {
      visualizer_->endFrame();
    }
    return false;
  }

  // 🎨 结束可视化帧
  if (visualizer_) {
    visualizer_->endFrame();
  }

  // 注意：本地仿真模式不发送数据到 WebSocket
  // WebSocket 在线模式会在 run_websocket_mode() 中处理数据发送

  return true;
}

}  // namespace navsim

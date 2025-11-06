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
#include "control/trajectory_tracker.hpp"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <thread>
#include <atomic>
#include <cmath>
#include <limits>

namespace {
constexpr double kPi = 3.14159265358979323846;

double normalizeAngleRad(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

int classifyVelocitySign(double vx) {
  constexpr double kVelocityEps = 0.05;  // m/s
  if (vx > kVelocityEps) {
    return 1;
  }
  if (vx < -kVelocityEps) {
    return -1;
  }
  return 0;
}

int inferTrajectoryVelocitySign(const std::vector<navsim::plugin::TrajectoryPoint>& trajectory) {
  for (const auto& point : trajectory) {
    int sign = classifyVelocitySign(point.twist.vx);
    if (sign != 0) {
      return sign;
    }
  }
  return 0;
}
}  // namespace

namespace navsim {

AlgorithmManager::AlgorithmManager() : config_(Config{}) {}

AlgorithmManager::AlgorithmManager(const Config& config)
    : config_(config) {}

AlgorithmManager::~AlgorithmManager() = default;

bool AlgorithmManager::initialize() {
  try {
    goal_hold_distance_ = config_.goal_hold_distance;

    std::cout << "[AlgorithmManager] Initializing with plugin system..." << std::endl;
    setupPluginSystem();

    // 初始化轨迹跟踪器
    control::TrajectoryTracker::Config tracker_config;
    tracker_config.mode = control::TrajectoryTracker::TrackingMode::PLAYBACK; //配置跟踪模式
    tracker_config.lookahead_time = 0.3;
    tracker_config.lookahead_distance = 1.0;
    tracker_config.enable_quality_assessment = true;
    tracker_config.max_velocity = 3.0;
    tracker_config.max_acceleration = 2.0;
    tracker_config.max_angular_velocity = 2.0;

    trajectory_tracker_ = std::make_unique<control::TrajectoryTracker>(tracker_config);
    std::cout << "[AlgorithmManager] Trajectory tracker initialized" << std::endl;

    // 初始化可视化器
    bool visualization_ready = false;

    std::cout << "[AlgorithmManager] Initializing visualizer..." << std::endl;
    visualizer_ = viz::createVisualizer(true);
    if (visualizer_ && visualizer_->initialize()) {
      std::cout << "[AlgorithmManager] Visualizer initialized successfully" << std::endl;
      visualization_ready = true;

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
      std::cerr << "[AlgorithmManager] Failed to initialize visualizer, falling back to null visualizer" << std::endl;
      visualizer_ = viz::createVisualizer(false);  // NullVisualizer fallback
    }

    std::cout << "[AlgorithmManager] Initialized successfully" << std::endl;
    std::cout << "  Primary planner: " << config_.primary_planner << std::endl;
    std::cout << "  Fallback planner: " << config_.fallback_planner << std::endl;
    std::cout << "  Max computation time: " << config_.max_computation_time_ms << " ms" << std::endl;
    std::cout << "  Visualization: " << (visualization_ready ? "ENABLED" : "DISABLED (fallback)") << std::endl;

    if (visualizer_) {
      system_info_cache_.general.clear();
      system_info_cache_.perception_plugins.clear();
      system_info_cache_.planner_plugins.clear();

      system_info_cache_.general["Config File"] = active_config_file_.empty()
        ? "config/default.json"
        : active_config_file_;
      system_info_cache_.general["Visualizer"] = visualization_ready
        ? "ImGui (SDL2/OpenGL2)"
        : "NullVisualizer";
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
      system_info_cache_.general["Visualization"] = visualization_ready ? "Enabled" : "Disabled";
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
  // std::cout << "[AlgorithmManager] ========== Perception Input Check ==========" << std::endl;
  // std::cout << "[AlgorithmManager] BEV obstacles in perception_input:" << std::endl;
  // std::cout << "[AlgorithmManager]   Circles: " << perception_input.bev_obstacles.circles.size() << std::endl;
  // std::cout << "[AlgorithmManager]   Rectangles: " << perception_input.bev_obstacles.rectangles.size() << std::endl;
  // std::cout << "[AlgorithmManager]   Polygons: " << perception_input.bev_obstacles.polygons.size() << std::endl;

  // 🎨 可视化感知输入数据
  if (visualizer_) {
    // std::cout << "[AlgorithmManager] Calling visualizer->drawBEVObstacles()..." << std::endl;
    visualizer_->drawEgo(perception_input.ego);
    visualizer_->drawGoal(perception_input.task.goal_pose);
    visualizer_->drawBEVObstacles(perception_input.bev_obstacles);

    // std::cout << "[AlgorithmManager] Calling visualizer->drawDynamicObstacles() with "
    //           << perception_input.dynamic_obstacles.size() << " obstacles..." << std::endl;
    // 🔧 修复问题1：打印所有障碍物的信息
    for (size_t i = 0; i < perception_input.dynamic_obstacles.size(); ++i) {
      const auto& obs = perception_input.dynamic_obstacles[i];
      // std::cout << "[AlgorithmManager]   Dyn obs #" << i << ": shape=" << obs.shape_type
      //           << ", pos=(" << obs.current_pose.x << ", " << obs.current_pose.y
      //           << "), length=" << obs.length << ", width=" << obs.width << std::endl;
    }
    visualizer_->drawDynamicObstacles(perception_input.dynamic_obstacles);
    // std::cout << "[AlgorithmManager] Visualizer calls completed" << std::endl;
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

  const planning::EgoVehicle& current_ego = context.ego;
  bool near_goal = isNearGoal(world_tick);

  plugin::PlanningResult planning_result;
  bool planning_success = false;
  bool use_hold_plan = false;

  if (near_goal && !hold_trajectory_.empty()) {
    if (config_.verbose_logging) {
      std::cout << "[AlgorithmManager] Near goal - attempting to reuse hold trajectory. Cached size="
                << hold_trajectory_.size() << std::endl;
      std::cout << "  Current ego pose: (" << current_ego.pose.x << ", "
                << current_ego.pose.y << "), yaw=" << current_ego.pose.yaw
                << ", twist.vx=" << current_ego.twist.vx
                << ", twist.vy=" << current_ego.twist.vy
                << ", twist.omega=" << current_ego.twist.omega << std::endl;
      const size_t preview_count = std::min<size_t>(5, hold_trajectory_.size());
      std::cout << "  Cached trajectory preview (first " << preview_count << " points):" << std::endl;
      for (size_t i = 0; i < preview_count; ++i) {
        const auto& pt = hold_trajectory_[i];
        std::cout << "    [" << i << "] p=(" << pt.pose.x << ", " << pt.pose.y
                  << "), yaw=" << pt.pose.yaw
                  << ", vx=" << pt.twist.vx
                  << ", vy=" << pt.twist.vy
                  << ", omega=" << pt.twist.omega
                  << ", path=" << pt.path_length
                  << ", t=" << pt.time_from_start << std::endl;
      }
    }

    auto trimmed = trimTrajectoryForCurrentPose(hold_trajectory_, current_ego);
    if (!trimmed.empty()) {
      if (config_.verbose_logging) {
        const size_t preview_count = std::min<size_t>(5, trimmed.size());
        // std::cout << "[AlgorithmManager] Hold trajectory trimmed. New size="
        //           << trimmed.size() << std::endl;
        // for (size_t i = 0; i < preview_count; ++i) {
        //   const auto& pt = trimmed[i];
        //   std::cout << "    [trim " << i << "] p=(" << pt.pose.x << ", " << pt.pose.y
        //             << "), yaw=" << pt.pose.yaw
        //             << ", vx=" << pt.twist.vx
        //             << ", vy=" << pt.twist.vy
        //             << ", omega=" << pt.twist.omega
        //             << ", path=" << pt.path_length
        //             << ", t=" << pt.time_from_start << std::endl;
        // }
      }
      planning_result.success = true;
      planning_result.trajectory = std::move(trimmed);
      planning_result.planner_name = hold_planner_name_.empty() ? "JpsPlanner" : hold_planner_name_;
      planning_success = true;
      use_hold_plan = true;
      hold_trajectory_ = planning_result.trajectory;
      int applied_sign = inferTrajectoryVelocitySign(hold_trajectory_);
      if (applied_sign == 0) {
        applied_sign = classifyVelocitySign(current_ego.twist.vx);
      }
      if (applied_sign == 0) {
        applied_sign = hold_last_velocity_sign_;
      }
      hold_last_velocity_sign_ = applied_sign;
    } else {
      if (config_.verbose_logging) {
        std::cout << "[AlgorithmManager] Hold trajectory trimming failed (empty result). Clearing cache."
                  << std::endl;
      }
      hold_trajectory_.clear();
      hold_planner_name_.clear();
      hold_last_velocity_sign_ = 0;
    }
  }

  if (!planning_success) {
    planning_success = planner_plugin_manager_->plan(context, remaining_time, planning_result);
    if (planning_success && !planning_result.trajectory.empty()) {
      hold_trajectory_ = planning_result.trajectory;
      hold_planner_name_ = planning_result.planner_name;
      hold_last_velocity_sign_ = inferTrajectoryVelocitySign(hold_trajectory_);
    } else if (!planning_success) {
      hold_trajectory_.clear();
      hold_planner_name_.clear();
      hold_last_velocity_sign_ = 0;
    }
  }

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

  if (use_hold_plan) {
    planning_result.planner_name += " [hold]";
  }

  // 🎨 可视化规划结果
  if (visualizer_) {
    visualizer_->updatePlanningResult(planning_result);
    visualizer_->drawTrajectory(planning_result.trajectory, planning_result.planner_name);

    // 🎨 可视化调试路径（支持 JPS 和 TMPC）
    if (planning_result.metadata.count("has_debug_paths") > 0 &&
        planning_result.metadata.count("debug_paths_ptr") > 0) {
      // Get debug paths from the pointer stored in metadata
      auto* debug_paths_ptr = reinterpret_cast<std::vector<std::vector<planning::Pose2d>>*>(
          static_cast<uintptr_t>(planning_result.metadata.at("debug_paths_ptr")));

      if (debug_paths_ptr && !debug_paths_ptr->empty()) {
        if (config_.verbose_logging) {
          std::cout << "[AlgorithmManager] Drawing " << debug_paths_ptr->size() << " debug paths for "
                    << planning_result.planner_name << " visualization" << std::endl;
        }

        std::vector<std::string> path_names;
        std::vector<std::string> colors;

        if (planning_result.planner_name == "JpsPlanner") {
          // JPS paths - simple naming
          for (size_t i = 0; i < debug_paths_ptr->size(); ++i) {
            path_names.push_back("JPS Path " + std::to_string(i));
            colors.push_back("yellow");
          }
        } else if (planning_result.planner_name == "MincoPlanner") {
          // MINCO paths - 2 stages
          path_names = {
            "MINCO Stage1",  // 第一阶段路径
            "MINCO Stage2"   // 第二阶段路径
          };
          colors = {
            "magenta",  // MINCO Stage1 - 洋红色（高对比度）
            "cyan"      // MINCO Stage2 - 青色（高对比度）
          };
        } else if (planning_result.planner_name == "TMPCPlanner") {
          // TMPC paths - use path type information from metadata
          std::vector<std::string>* path_types_ptr = nullptr;
          if (planning_result.metadata.count("debug_path_types_ptr") > 0) {
            path_types_ptr = reinterpret_cast<std::vector<std::string>*>(
                static_cast<uintptr_t>(planning_result.metadata.at("debug_path_types_ptr")));
          }

          if (path_types_ptr && path_types_ptr->size() == debug_paths_ptr->size()) {
            // Use path type information
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
              } else {
                path_names.push_back("TMPC Unknown");
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

        visualizer_->drawDebugPaths(*debug_paths_ptr, path_names, colors);
      }
    }

    // 🎨 可视化近似圆（TMPC 规划器）
    if (planning_result.planner_name == "TMPCPlanner" &&
        planning_result.metadata.count("approximation_circles_ptr") > 0 &&
        planning_result.metadata.count("approximation_circles_count") > 0) {

      // 定义与插件中相同的结构
      struct ApproximationCircle {
        double x, y, radius;
      };

      auto* circles_ptr = reinterpret_cast<std::vector<ApproximationCircle>*>(
          static_cast<uintptr_t>(planning_result.metadata.at("approximation_circles_ptr")));

      if (circles_ptr && !circles_ptr->empty()) {
        std::vector<std::tuple<double, double, double>> circles;
        for (const auto& c : *circles_ptr) {
          circles.emplace_back(c.x, c.y, c.radius);
        }

        // Use dynamic_cast to call ImGuiVisualizer-specific method
        auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(visualizer_.get());
        if (imgui_viz) {
          imgui_viz->drawApproximationCircles(circles);
          if (config_.verbose_logging) {
            std::cout << "[AlgorithmManager] Drawing " << circles.size() << " approximation circles" << std::endl;
          }
        }
      }
    }
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

  // 重置轨迹跟踪器
  if (trajectory_tracker_) {
    trajectory_tracker_->reset();
  }

  // 重置播放状态
  playback_active_ = false;
  playback_elapsed_time_ = 0.0;
  playback_last_plan_tick_id_.reset();
  goal_reached_ = false;
  playback_plan_signature_.reset();
  hold_trajectory_.clear();
  hold_planner_name_.clear();
  hold_last_velocity_sign_ = 0;

  simulation_started_.store(false);
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
  std::cout << "[AlgorithmManager] ========================================" << std::endl;
  std::cout << "[AlgorithmManager] loadScenario() called!" << std::endl;
  std::cout << "[AlgorithmManager] Loading scenario: " << scenario_file << std::endl;

  // 🔧 添加日志到 UI
  auto addUILog = [this](const std::string& msg) {
    if (visualizer_) {
      // 尝试转换为 ImGuiVisualizer
      auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(visualizer_.get());
      if (imgui_viz) {
        imgui_viz->addLog(msg);
      }
    }
  };

  addUILog("🔄 Loading scenario: " + scenario_file);

  // 1. 检查文件是否存在
  std::cout << "[AlgorithmManager] Checking if file exists..." << std::endl;
  std::ifstream file(scenario_file);
  if (!file.good()) {
    std::cerr << "[AlgorithmManager] ERROR: Scenario file not found: " << scenario_file << std::endl;
    std::cerr << "[AlgorithmManager] Please check the file path and try again." << std::endl;
    addUILog("❌ ERROR: File not found!");
    return false;
  }
  file.close();
  std::cout << "[AlgorithmManager] File exists, proceeding..." << std::endl;
  addUILog("✅ File found, loading...");

  // 2. 🔧 不要停止仿真循环，只是暂停仿真
  // 保存当前状态
  bool was_paused = simulation_paused_.load();

  // 暂停仿真（但不停止循环）
  simulation_paused_.store(true);

  // 等待当前帧完成
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // 3. 执行完整的系统重置
  addUILog("🔄 Resetting system...");
  performFullReset();

  // 4. 重新加载场景到仿真器
  if (local_simulator_) {
    std::cout << "[AlgorithmManager] Loading scenario into simulator..." << std::endl;
    addUILog("🔄 Loading into simulator...");
    if (!local_simulator_->load_scenario(scenario_file, addUILog)) {
      std::cerr << "[AlgorithmManager] Failed to load scenario into simulator" << std::endl;
      addUILog("❌ Failed to load scenario!");
      return false;
    }
  } else {
    std::cerr << "[AlgorithmManager] No local simulator available" << std::endl;
    addUILog("❌ No simulator available!");
    return false;
  }

  // 5. 保存当前场景文件路径
  current_scenario_file_ = scenario_file;

  // 6. 加载新场景后默认暂停，等待用户点击 Start
  simulation_paused_.store(true);

  std::cout << "[AlgorithmManager] Scenario loaded successfully: " << scenario_file << std::endl;
  std::cout << "[AlgorithmManager] Simulation paused, click START to begin" << std::endl;
  addUILog("✅ Scenario loaded successfully!");
  addUILog("ℹ️  Click START to begin simulation");
  return true;
}

void AlgorithmManager::startSimulation() {
  goal_reached_ = false;
  playback_plan_signature_.reset();
  simulation_started_.store(true);
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
  simulation_started_.store(false);
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

bool AlgorithmManager::isGoalReached(const sim::WorldState& world_state) const {
  double pos_tol = world_state.goal_tolerance_pos > 1e-6
    ? world_state.goal_tolerance_pos
    : 0.3;
  double yaw_tol = world_state.goal_tolerance_yaw > 1e-6
    ? world_state.goal_tolerance_yaw
    : 0.2;

  double dx = world_state.ego_pose.x - world_state.goal_pose.x;
  double dy = world_state.ego_pose.y - world_state.goal_pose.y;
  double distance = std::hypot(dx, dy);

  double yaw_error = std::fabs(normalizeAngleRad(
      world_state.ego_pose.yaw - world_state.goal_pose.yaw));

  if (distance > pos_tol) {
    // 若位置略超出容差但车速已接近 0，则认为已到达
    double speed = std::hypot(world_state.ego_twist.vx, world_state.ego_twist.vy);
    if (distance <= pos_tol + 0.15 && speed < 0.05) {
      return true;
    }
    return false;
  }

  // 位置已满足容差，姿态/速度任意条件满足即可认定到达
  if (yaw_tol < 1e-6 || yaw_error <= yaw_tol) {
    return true;
  }

  // 允许姿态稍有偏差，只要车辆已基本停止
  double speed = std::hypot(world_state.ego_twist.vx, world_state.ego_twist.vy);
  double angular_speed = std::fabs(world_state.ego_twist.omega);
  return speed < 0.05 && angular_speed < 0.05;
}

bool AlgorithmManager::isNearGoal(const proto::WorldTick& world_tick) const {
  if (!world_tick.has_goal() || !world_tick.has_ego()) {
    return false;
  }
  const auto& ego_pose = world_tick.ego().pose();
  const auto& goal_pose = world_tick.goal().pose();
  double dx = ego_pose.x() - goal_pose.x();
  double dy = ego_pose.y() - goal_pose.y();
  double distance = std::hypot(dx, dy);
  return distance <= goal_hold_distance_;
}

std::vector<plugin::TrajectoryPoint> AlgorithmManager::trimTrajectoryForCurrentPose(
    const std::vector<plugin::TrajectoryPoint>& trajectory,
    const planning::EgoVehicle& current_ego) const {
  if (trajectory.empty()) {
    return {};
  }

  const auto& current_pose = current_ego.pose;

  const int current_sign = classifyVelocitySign(current_ego.twist.vx);
  const int desired_sign = (current_sign != 0) ? current_sign : hold_last_velocity_sign_;

  size_t closest_idx = 0;
  double min_distance = std::numeric_limits<double>::infinity();
  bool found_matching_sign = false;

  size_t fallback_idx = 0;
  double fallback_distance = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < trajectory.size(); ++i) {
    double dx = trajectory[i].pose.x - current_pose.x;
    double dy = trajectory[i].pose.y - current_pose.y;
    double distance = std::hypot(dx, dy);

    if (distance < fallback_distance) {
      fallback_distance = distance;
      fallback_idx = i;
    }

    const int point_sign = classifyVelocitySign(trajectory[i].twist.vx);
    bool sign_match = false;
    if (desired_sign == 0) {
      sign_match = (point_sign == 0);
    } else {
      sign_match = (point_sign == desired_sign && point_sign != 0);
    }

    if (sign_match && distance < min_distance) {
      min_distance = distance;
      closest_idx = i;
      found_matching_sign = true;
    }
  }

  if (!found_matching_sign) {
    closest_idx = fallback_idx;
    if (config_.verbose_logging) {
      std::cout << "[AlgorithmManager] Hold trajectory: no matching velocity segment found; "
                   "falling back to geometric closest point index="
                << closest_idx << std::endl;
    }
  }

  const auto& start_point = trajectory[closest_idx];
  double time_offset = start_point.time_from_start;
  double path_offset = start_point.path_length;

  std::vector<plugin::TrajectoryPoint> trimmed;
  trimmed.reserve(trajectory.size() - closest_idx);

  for (size_t i = closest_idx; i < trajectory.size(); ++i) {
    plugin::TrajectoryPoint point = trajectory[i];
    point.time_from_start = std::max(0.0, point.time_from_start - time_offset);
    point.path_length = std::max(0.0, point.path_length - path_offset);
    trimmed.push_back(point);
  }

  if (trimmed.empty()) {
    return trimmed;
  }

  trimmed.front().pose = current_pose;
  trimmed.front().time_from_start = 0.0;
  trimmed.front().path_length = 0.0;
  trimmed.front().twist = current_ego.twist;

  return trimmed;
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

  // 初始状态：仿真未开始、处于暂停
  simulation_started_.store(false);
  simulation_paused_.store(true);
  goal_reached_ = false;
  playback_active_ = false;
  playback_elapsed_time_ = 0.0;
  playback_plan_signature_.reset();
  if (local_simulator_) {
    local_simulator_->pause();
  }

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
      // 暂停时仍然渲染可视化界面，并显示当前世界状态
      if (visualizer_) {
        visualizer_->beginFrame();

        // 🔧 即使暂停，也要显示当前世界状态（特别是加载新场景后）
        const auto& world_state = local_simulator_->get_world_state();

        // 更新仿真时间
        double sim_time = local_simulator_->get_simulation_time();
        std::ostringstream time_stream;
        time_stream << std::fixed << std::setprecision(3) << sim_time << "s";
        visualizer_->showDebugInfo("Simulation Time", time_stream.str());
        visualizer_->showDebugInfo("Frame ID", std::to_string(local_simulator_->get_frame_id()));

        // 转换为protobuf格式并更新可视化
        auto world_tick = local_simulator_->to_world_tick();

        // 创建前置处理管线并处理
        perception::PreprocessingPipeline preprocessing_pipeline;
        plugin::PerceptionInput perception_input = preprocessing_pipeline.process(world_tick);

        // 更新可视化器的世界数据
        visualizer_->drawEgo(perception_input.ego);
        visualizer_->drawGoal(perception_input.task.goal_pose);
        visualizer_->drawBEVObstacles(perception_input.bev_obstacles);
        visualizer_->drawDynamicObstacles(perception_input.dynamic_obstacles);

        // 如果有感知插件，也处理一下以获取栅格地图
        planning::PlanningContext context;
        context.ego = perception_input.ego;
        context.task = perception_input.task;
        context.dynamic_obstacles = perception_input.dynamic_obstacles;

        if (perception_plugin_manager_->process(perception_input, context)) {
          visualizer_->updatePlanningContext(context);
          if (context.occupancy_grid) {
            visualizer_->drawOccupancyGrid(*context.occupancy_grid);
          }
        }

        visualizer_->showDebugInfo("Simulation Status", "PAUSED");
        visualizer_->endFrame();
      }

      // 短暂休眠避免CPU占用过高
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      last_step_time = std::chrono::steady_clock::now();
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

  double current_sim_time = local_simulator_->get_simulation_time();

  // 4. 将规划结果应用到仿真器
  planning::Pose2d playback_target_pose{};
  planning::Twist2d playback_target_twist{};
  planning::Twist2d control_command_for_log{};
  bool playback_apply_state = false;
  bool control_command_valid = false;
  double playback_duration = 0.0;
  bool tracker_playback_mode =
    trajectory_tracker_->getConfig().mode == control::TrajectoryTracker::TrackingMode::PLAYBACK;

  if (planning_success && plan_update.trajectory_size() > 0) {
    // 🔧 调试：打印前几个轨迹点的速度
    static bool first_print = true;
    if (first_print && plan_update.trajectory_size() > 0) {
      std::cout << "\n[DEBUG] First 10 trajectory points:" << std::endl;
      for (int i = 0; i < std::min(10, plan_update.trajectory_size()); ++i) {
        const auto& pt = plan_update.trajectory(i);
        std::cout << "  [" << i << "] t=" << pt.t()
                  << "s, pos=(" << pt.x() << ", " << pt.y() << ")"
                  << ", vx=" << pt.vx() << ", omega=" << pt.omega() << std::endl;
      }
      first_print = false;
    }

    // 🚗 使用改进的轨迹跟踪器

    // 获取当前仿真时间并设置新轨迹
    double trajectory_start_time = tracker_playback_mode ? 0.0 : current_sim_time;

    double playback_query_time = tracker_playback_mode ? playback_elapsed_time_ : current_sim_time;

    if (tracker_playback_mode) {
      playback_active_ = true;

      bool plan_changed = true;
      PlaybackPlanSignature new_signature{};
      if (plan_update.trajectory_size() > 0) {
        const auto& last = plan_update.trajectory(plan_update.trajectory_size() - 1);
        new_signature.point_count = static_cast<std::size_t>(plan_update.trajectory_size());
        new_signature.last_t = last.t();
        new_signature.last_x = last.x();
        new_signature.last_y = last.y();
        new_signature.last_yaw = last.yaw();
      }

      if (playback_plan_signature_) {
        const auto& existing = *playback_plan_signature_;
        if (existing.point_count == new_signature.point_count &&
            std::abs(existing.last_t - new_signature.last_t) < 1e-6 &&
            std::abs(existing.last_x - new_signature.last_x) < 1e-4 &&
            std::abs(existing.last_y - new_signature.last_y) < 1e-4 &&
            std::abs(normalizeAngleRad(existing.last_yaw - new_signature.last_yaw)) < 1e-3) {
          plan_changed = false;
        }
      } else if (new_signature.point_count == 0) {
        plan_changed = false;
      }

      if (plan_changed) {
        playback_elapsed_time_ = 0.0;
        playback_plan_signature_ = new_signature;
        playback_last_plan_tick_id_ = plan_update.tick_id();
        goal_reached_ = false;
      }

      playback_query_time = playback_elapsed_time_;
    } else {
      playback_active_ = false;
      playback_elapsed_time_ = 0.0;
      playback_last_plan_tick_id_.reset();
      playback_plan_signature_.reset();
    }

    trajectory_tracker_->setTrajectoryFromProto(plan_update, trajectory_start_time);
    playback_duration = tracker_playback_mode ? trajectory_tracker_->getTrajectoryDuration() : 0.0;
    double playback_time_step = config_.playback_time_step;

    // 🔍 轨迹设置调试信息
    if (config_.verbose_logging && world_state.frame_id % 60 == 0) {  // 每2秒打印一次
      std::cout << "[AlgorithmManager] 🎯 轨迹设置调试信息:" << std::endl;
      std::cout << "  规划轨迹点数: " << plan_update.trajectory_size() << std::endl;
      if (plan_update.trajectory_size() > 0) {
        std::cout << "  首个轨迹点: (" << plan_update.trajectory(0).x()
                  << ", " << plan_update.trajectory(0).y() << ")" << std::endl;
        std::cout << "  首个轨迹点速度: vx=" << plan_update.trajectory(0).vx() << std::endl;
        std::cout << "  首个轨迹点时间: " << plan_update.trajectory(0).t() << " s" << std::endl;
      }
      std::cout << "  跟踪器轨迹时长: " << trajectory_tracker_->getTrajectoryDuration() << " s" << std::endl;
      std::cout << "  跟踪器有效轨迹: " << (trajectory_tracker_->hasValidTrajectory() ? "是" : "否") << std::endl;
    }

    const auto& current_world_state = local_simulator_->get_world_state();

    if (tracker_playback_mode) {
      auto target_state = trajectory_tracker_->getTargetStateOriginalYaw(playback_query_time);

      playback_target_pose.x = target_state.pose.x;
      playback_target_pose.y = target_state.pose.y;
      playback_target_pose.yaw = target_state.pose.yaw;

      playback_target_twist.vx = target_state.twist.vx;
      playback_target_twist.vy = target_state.twist.vy;
      playback_target_twist.omega = target_state.twist.omega;

      trajectory_tracker_->updateQualityAssessment(
          playback_target_pose,
          playback_target_twist,
          playback_query_time
      );

      playback_apply_state = true;
      control_command_for_log = playback_target_twist;
      control_command_valid = true;
    } else {
      // 使用跟踪器计算控制指令
      planning::Twist2d new_twist = trajectory_tracker_->getControlCommand(current_sim_time);

      // 更新轨迹跟踪质量评估
      trajectory_tracker_->updateQualityAssessment(
          current_world_state.ego_pose,
          current_world_state.ego_twist,
          current_sim_time
      );

      // 应用控制指令
      local_simulator_->set_ego_twist(new_twist);
      control_command_for_log = new_twist;
      control_command_valid = true;
    }

    // 显示轨迹跟踪质量信息
    if (visualizer_) {
      const auto& quality = trajectory_tracker_->getQualityMetrics();
      const auto& tracking_state = trajectory_tracker_->getTrackingState();
      const auto& ego_pose_for_viz = playback_apply_state ? playback_target_pose : current_world_state.ego_pose;
      const auto& ego_twist_for_viz = playback_apply_state ? playback_target_twist : current_world_state.ego_twist;

      // 实时质量指标显示
      visualizer_->showDebugInfo("=== Trajectory Tracking ===", "");
      visualizer_->showDebugInfo("Position Error",
          std::to_string(static_cast<int>(quality.position_error * 1000)) + " mm");
      visualizer_->showDebugInfo("Velocity Error",
          std::to_string(static_cast<int>(quality.velocity_error * 1000)) + " mm/s");
      visualizer_->showDebugInfo("Heading Error",
          std::to_string(static_cast<int>(quality.heading_error * 180.0 / M_PI)) + " deg");
      visualizer_->showDebugInfo("Smoothness Score",
          std::to_string(static_cast<int>(quality.smoothness_score)) + "/100");
      visualizer_->showDebugInfo("Overall Score",
          std::to_string(static_cast<int>(quality.overall_score)) + "/100");

      // 轨迹完成度
      double completion = trajectory_tracker_->getCompletionPercentage(tracker_playback_mode ? playback_query_time : current_sim_time);
      visualizer_->showDebugInfo("Trajectory Progress",
          std::to_string(static_cast<int>(completion)) + "%");

      // 动力学约束状态
      std::string constraint_status = "OK";
      if (quality.velocity_limit_violated) constraint_status = "VEL_LIMIT";
      if (quality.angular_velocity_limit_violated) constraint_status = "ANG_LIMIT";
      visualizer_->showDebugInfo("Constraints", constraint_status);

      // 🎯 绘制轨迹跟踪状态可视化
      auto target_state = trajectory_tracker_->getTargetStateOriginalYaw(tracker_playback_mode ? playback_query_time : current_sim_time);
      planning::Pose2d target_pose(target_state.pose.x, target_state.pose.y, target_state.pose.yaw);

      visualizer_->drawTrajectoryTracking(
        ego_pose_for_viz,
        target_pose,
        target_state,
        quality.position_error,
        quality.heading_error
      );
    }

    if (config_.verbose_logging && world_state.frame_id % 30 == 0) {  // 每秒打印一次
      const auto& quality = trajectory_tracker_->getQualityMetrics();
      auto target_state = trajectory_tracker_->getTargetStateOriginalYaw(tracker_playback_mode ? playback_query_time : current_sim_time);

      std::cout << "[AlgorithmManager] Step " << world_state.frame_id
                << ": Planning success, " << plan_update.trajectory_size()
                << " trajectory points generated" << std::endl;

      // 🔍 详细的轨迹跟踪调试信息
      std::cout << "=== 轨迹跟踪状态调试 ===" << std::endl;
      std::cout << "  仿真时间: " << (tracker_playback_mode ? playback_query_time : current_sim_time) << " s" << std::endl;
      std::cout << "  实际位置: (" << (playback_apply_state ? playback_target_pose.x : current_world_state.ego_pose.x) << ", "
                << (playback_apply_state ? playback_target_pose.y : current_world_state.ego_pose.y) << ", "
                << (playback_apply_state ? playback_target_pose.yaw : current_world_state.ego_pose.yaw) << ")" << std::endl;
      std::cout << "  目标位置: (" << target_state.pose.x << ", "
                << target_state.pose.y << ", "
                << target_state.pose.yaw << ")" << std::endl;
      std::cout << "  实际速度: vx=" << (playback_apply_state ? playback_target_twist.vx : current_world_state.ego_twist.vx)
                << ", vy=" << (playback_apply_state ? playback_target_twist.vy : current_world_state.ego_twist.vy)
                << ", omega=" << (playback_apply_state ? playback_target_twist.omega : current_world_state.ego_twist.omega) << std::endl;
      std::cout << "  目标速度: vx=" << target_state.twist.vx
                << ", vy=" << target_state.twist.vy
                << ", omega=" << target_state.twist.omega << std::endl;
      if (control_command_valid) {
        std::cout << "  控制指令: vx=" << control_command_for_log.vx
                  << ", vy=" << control_command_for_log.vy
                  << ", omega=" << control_command_for_log.omega << std::endl;
      }
      std::cout << "  跟踪质量:" << std::endl;
      std::cout << "    位置误差: " << quality.position_error * 1000 << " mm" << std::endl;
      std::cout << "    速度误差: " << quality.velocity_error * 1000 << " mm/s" << std::endl;
      std::cout << "    航向误差: " << quality.heading_error * 180.0 / M_PI << " deg" << std::endl;
      std::cout << "    平滑度评分: " << quality.smoothness_score << "/100" << std::endl;
      std::cout << "    综合评分: " << quality.overall_score << "/100" << std::endl;

      // 检查是否跟踪器有有效轨迹
      if (!trajectory_tracker_->hasValidTrajectory()) {
        std::cout << "  ⚠️  警告: 轨迹跟踪器没有有效轨迹!" << std::endl;
      } else {
        double completion = trajectory_tracker_->getCompletionPercentage(tracker_playback_mode ? playback_query_time : current_sim_time);
        std::cout << "  轨迹完成度: " << completion << "%" << std::endl;
        std::cout << "  轨迹总时长: " << trajectory_tracker_->getTrajectoryDuration() << " s" << std::endl;
      }
      std::cout << "===========================" << std::endl;
    }
  }

  // 5. 执行仿真步进（应用新的状态）
  bool playback_step_mode = playback_apply_state && tracker_playback_mode;

  if (playback_step_mode) {
    double duration = playback_duration;
    if (duration <= 0.0) {
      duration = trajectory_tracker_->getTrajectoryDuration();
    }
    double step_dt = std::max(1e-4, config_.playback_time_step);
    double total_time_to_play = std::min(dt, duration - playback_elapsed_time_);
    double remaining_time_in_step = total_time_to_play;

    while (remaining_time_in_step > 1e-9) {
      double use_dt = std::min(remaining_time_in_step, step_dt);
      remaining_time_in_step -= use_dt;

      if (!local_simulator_->step(use_dt)) {
        std::cerr << "[AlgorithmManager] Simulator step failed" << std::endl;
        if (visualizer_) {
          visualizer_->endFrame();
        }
        return false;
      }

      playback_elapsed_time_ += use_dt;
      playback_elapsed_time_ = std::min(playback_elapsed_time_, duration);

      auto post_step_state = trajectory_tracker_->getTargetStateOriginalYaw(playback_elapsed_time_);

      playback_target_pose.x = post_step_state.pose.x;
      playback_target_pose.y = post_step_state.pose.y;
      playback_target_pose.yaw = post_step_state.pose.yaw;

      playback_target_twist.vx = post_step_state.twist.vx;
      playback_target_twist.vy = post_step_state.twist.vy;
      playback_target_twist.omega = post_step_state.twist.omega;

      local_simulator_->apply_ego_state(playback_target_pose, playback_target_twist);

      if (playback_elapsed_time_ >= duration) {
        playback_active_ = false;
        break;
      }
    }

    playback_elapsed_time_ = std::min(playback_elapsed_time_, duration);

    trajectory_tracker_->updateQualityAssessment(
        playback_target_pose,
        playback_target_twist,
        playback_elapsed_time_
    );

    if (duration > 0.0 && (duration - playback_elapsed_time_) <= 1e-4) {
      playback_elapsed_time_ = duration;
      playback_active_ = false;
      goal_reached_ = true;
      if (!simulation_paused_.load()) {
        pauseSimulation();
      }
      if (visualizer_) {
        visualizer_->showDebugInfo("Goal", "Trajectory completed");
        visualizer_->showDebugInfo("Simulation Status", "Playback finished");
        if (auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(visualizer_.get())) {
          imgui_viz->addLog("✅ Trajectory playback completed, simulation paused");
        }
      }
    }
  } else {
    if (!local_simulator_->step(dt)) {
      std::cerr << "[AlgorithmManager] Simulator step failed" << std::endl;
      if (visualizer_) {
        visualizer_->endFrame();
      }
      return false;
    }
  }

  if (local_simulator_) {
    const auto& updated_world_state = local_simulator_->get_world_state();
    if (!goal_reached_ && isGoalReached(updated_world_state)) {
      goal_reached_ = true;
      playback_active_ = false;
      playback_elapsed_time_ = 0.0;
      playback_last_plan_tick_id_.reset();
      if (trajectory_tracker_) {
        trajectory_tracker_->reset();
      }
      std::cout << "[AlgorithmManager] Goal reached. Pausing simulation." << std::endl;
      pauseSimulation();
      if (visualizer_) {
        visualizer_->showDebugInfo("Goal", "Reached");
        visualizer_->showDebugInfo("Simulation Status", "Goal reached");
        if (auto* imgui_viz = dynamic_cast<viz::ImGuiVisualizer*>(visualizer_.get())) {
          imgui_viz->addLog("✅ Goal reached, simulation paused");
        }
      }
    }
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

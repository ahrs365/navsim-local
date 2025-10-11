#include "algorithm_manager.hpp"
#include <iostream>
#include <iomanip>

namespace navsim {

AlgorithmManager::AlgorithmManager() : config_(Config{}) {}

AlgorithmManager::AlgorithmManager(const Config& config)
    : config_(config) {}

AlgorithmManager::~AlgorithmManager() = default;

bool AlgorithmManager::initialize() {
  try {
    // 初始化感知管线
    setupPerceptionPipeline();

    // 初始化规划器管理器
    setupPlannerManager();

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
  auto start_time = std::chrono::steady_clock::now();

  stats_.total_processed++;

  // Step 1: 感知处理
  auto perception_start = std::chrono::steady_clock::now();

  planning::PlanningContext context;
  bool perception_success = perception_pipeline_->process(world_tick, context);

  auto perception_end = std::chrono::steady_clock::now();
  double perception_time = std::chrono::duration<double, std::milli>(perception_end - perception_start).count();

  if (!perception_success) {
    stats_.perception_failures++;
    if (config_.verbose_logging) {
      std::cerr << "[AlgorithmManager] Perception processing failed" << std::endl;
    }
    return false;
  }

  // 发送感知调试数据（如果启用）
  if (bridge_) {
    bridge_->send_perception_debug(context);
  }

  // Step 2: 规划处理
  auto planning_start = std::chrono::steady_clock::now();

  // 调整规划截止时间，为感知处理预留时间
  auto remaining_time = deadline - std::chrono::duration_cast<std::chrono::milliseconds>(
    planning_start - start_time);
  remaining_time = std::max(remaining_time, std::chrono::milliseconds(5)); // 至少5ms

  planning::PlanningResult planning_result;
  bool planning_success = planner_manager_->plan(context, remaining_time, planning_result);

  auto planning_end = std::chrono::steady_clock::now();
  double planning_time = std::chrono::duration<double, std::milli>(planning_end - planning_start).count();

  if (!planning_success) {
    stats_.planning_failures++;
    if (config_.verbose_logging) {
      std::cerr << "[AlgorithmManager] Planning failed: "
                << planning_result.diagnostics.failure_reason << std::endl;
    }
    return false;
  }

  // Step 3: 协议转换
  bool conversion_success =
    planning::ProtocolConverter::convertToPlanUpdate(planning_result,
                                                   world_tick.tick_id(),
                                                   world_tick.stamp(),
                                                   plan_update) &&
    planning::ProtocolConverter::convertToEgoCmd(planning_result,
                                               world_tick.tick_id(),
                                               world_tick.stamp(),
                                               ego_cmd);

  if (!conversion_success) {
    if (config_.verbose_logging) {
      std::cerr << "[AlgorithmManager] Protocol conversion failed" << std::endl;
    }
    return false;
  }

  // Step 4: 更新统计信息
  auto end_time = std::chrono::steady_clock::now();
  double total_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  updateStatistics(total_time, perception_time, planning_time, true);
  stats_.successful_processed++;

  // 详细日志
  if (config_.verbose_logging) {
    std::cout << "[AlgorithmManager] Processing successful:" << std::endl;
    std::cout << "  Total time: " << std::fixed << std::setprecision(1) << total_time << " ms" << std::endl;
    std::cout << "  Perception time: " << perception_time << " ms" << std::endl;
    std::cout << "  Planning time: " << planning_time << " ms" << std::endl;
    std::cout << "  Trajectory points: " << planning_result.trajectory.size() << std::endl;
    std::cout << "  Planning cost: " << planning_result.cost.total << std::endl;
  }

  return true;
}

void AlgorithmManager::updateConfig(const Config& config) {
  config_ = config;

  // 重新初始化模块 (如果需要)
  if (perception_pipeline_ || planner_manager_) {
    std::cout << "[AlgorithmManager] Reinitializing with new config..." << std::endl;
    initialize();
  }
}

void AlgorithmManager::setBridge(Bridge* bridge) {
  bridge_ = bridge;
  if (planner_manager_) {
    planner_manager_->setBridge(bridge);
  }
}

void AlgorithmManager::setupPerceptionPipeline() {
  perception_pipeline_ = std::make_unique<perception::PerceptionPipeline>();

  // 根据配置添加感知处理器
  if (config_.enable_occupancy_grid) {
    perception::OccupancyGridBuilder::Config grid_config;
    grid_config.resolution = 0.1;
    grid_config.map_width = 50.0;   // 50m x 50m 地图
    grid_config.map_height = 50.0;
    grid_config.inflation_radius = 0.3; // 30cm 膨胀半径

    auto grid_builder = std::make_unique<perception::OccupancyGridBuilder>(grid_config);
    perception_pipeline_->addProcessor(std::move(grid_builder), true);
  }

  if (config_.enable_bev_obstacles) {
    perception::BEVObstacleExtractor::Config bev_config;
    bev_config.detection_range = 30.0;  // 30m 检测范围
    bev_config.confidence_threshold = 0.5;

    auto bev_extractor = std::make_unique<perception::BEVObstacleExtractor>(bev_config);
    perception_pipeline_->addProcessor(std::move(bev_extractor), true);
  }

  if (config_.enable_dynamic_prediction) {
    perception::DynamicObstaclePredictor::Config pred_config;
    pred_config.prediction_horizon = 3.0;  // 3秒预测时域
    pred_config.prediction_model = "constant_velocity";

    auto predictor = std::make_unique<perception::DynamicObstaclePredictor>(pred_config);
    perception_pipeline_->addProcessor(std::move(predictor), true);
  }

  std::cout << "[AlgorithmManager] Perception pipeline configured with "
            << perception_pipeline_->getProcessorStatus().size() << " processors" << std::endl;
}

void AlgorithmManager::setupPlannerManager() {
  planning::PlannerManager::Config manager_config;
  manager_config.primary_planner = config_.primary_planner;
  manager_config.fallback_planner = config_.fallback_planner;
  manager_config.enable_fallback = config_.enable_planner_fallback;

  // 启用WebSocket实时可视化 (发送到前端)
  manager_config.enable_visualization = true;
  manager_config.viz_config.enable_output = true;
  manager_config.viz_config.update_interval_ms = 200.0;  // 5Hz更新

  planner_manager_ = std::make_unique<planning::PlannerManager>(manager_config);

  // 注册可用的规划器

  // 1. 直线规划器 (总是可用的基础规划器)
  planning::StraightLinePlanner::Config straight_config;
  straight_config.time_step = 0.1;
  straight_config.default_velocity = 1.5;  // 1.5 m/s 默认速度
  straight_config.arrival_tolerance = 0.3;

  auto straight_planner = std::make_unique<planning::StraightLinePlanner>(straight_config);
  planner_manager_->registerPlanner(std::move(straight_planner));

  // 2. A*规划器 (需要栅格地图)
  if (config_.enable_occupancy_grid) {
    planning::AStarPlanner::Config astar_config;
    astar_config.time_step = 0.1;
    astar_config.step_size = 0.3;
    astar_config.heuristic_weight = 1.2;
    astar_config.max_iterations = 5000;

    auto astar_planner = std::make_unique<planning::AStarPlanner>(astar_config);
    planner_manager_->registerPlanner(std::move(astar_planner));
  }

  // 3. 优化规划器 (未来实现)
  planning::OptimizationPlanner::Config opt_config;
  auto opt_planner = std::make_unique<planning::OptimizationPlanner>(opt_config);
  planner_manager_->registerPlanner(std::move(opt_planner));

  // 设置活跃规划器
  planner_manager_->setActivePlanner(config_.primary_planner);

  auto available_planners = planner_manager_->getAvailablePlanners();
  std::cout << "[AlgorithmManager] Registered planners: ";
  for (const auto& name : available_planners) {
    std::cout << name << " ";
  }
  std::cout << std::endl;
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

} // namespace navsim
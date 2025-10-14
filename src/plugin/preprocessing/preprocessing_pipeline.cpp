#include "plugin/preprocessing/preprocessing.hpp"
#include <chrono>
#include <iostream>

namespace navsim {
namespace perception {

PreprocessingPipeline::PreprocessingPipeline(const Config& config)
    : config_(config),
      bev_extractor_(config.bev_config),
      dynamic_predictor_(config.predictor_config) {
}

plugin::PerceptionInput PreprocessingPipeline::process(
    const proto::WorldTick& world_tick) {
  auto start_time = std::chrono::steady_clock::now();

  plugin::PerceptionInput input;

  // 1. 提取自车状态
  input.ego = BasicDataConverter::convertEgo(world_tick);

  // 2. 提取规划任务
  input.task = BasicDataConverter::convertTask(world_tick);

  // 3. 提取 BEV 障碍物
  std::cout << "[PreprocessingPipeline] Extracting BEV obstacles..." << std::endl;
  auto bev_obstacles = bev_extractor_.extract(world_tick);
  if (bev_obstacles) {
    input.bev_obstacles = *bev_obstacles;
    std::cout << "[PreprocessingPipeline] BEV obstacles extracted successfully:" << std::endl;
    std::cout << "[PreprocessingPipeline]   Circles: " << input.bev_obstacles.circles.size() << std::endl;
    std::cout << "[PreprocessingPipeline]   Rectangles: " << input.bev_obstacles.rectangles.size() << std::endl;
    std::cout << "[PreprocessingPipeline]   Polygons: " << input.bev_obstacles.polygons.size() << std::endl;
  } else {
    std::cout << "[PreprocessingPipeline] WARNING: BEV extractor returned nullptr!" << std::endl;
  }

  // 4. 预测动态障碍物
  input.dynamic_obstacles = dynamic_predictor_.predict(world_tick);

  // 5. 保存原始数据指针（可选）
  input.raw_world_tick = &world_tick;

  // 6. 设置时间戳
  input.timestamp = world_tick.stamp();

  // 7. 设置 tick ID
  input.tick_id = world_tick.tick_id();

  // 更新统计信息
  auto end_time = std::chrono::steady_clock::now();
  double elapsed_ms =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();

  stats_.total_processed++;
  stats_.average_time_ms =
      (stats_.average_time_ms * (stats_.total_processed - 1) + elapsed_ms) /
      stats_.total_processed;

  return input;
}

void PreprocessingPipeline::setConfig(const Config& config) {
  config_ = config;
  bev_extractor_ = BEVExtractor(config.bev_config);
  dynamic_predictor_ = DynamicObstaclePredictor(config.predictor_config);
}

void PreprocessingPipeline::reset() {
  bev_extractor_.reset();
  dynamic_predictor_.reset();
  stats_ = Statistics();
}

} // namespace perception
} // namespace navsim


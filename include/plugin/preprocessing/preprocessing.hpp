#pragma once

#include "core/planning_context.hpp"
#include "plugin/data/perception_input.hpp"
#include "world_tick.pb.h"
#include <memory>
#include <vector>

namespace navsim {
namespace perception {

/**
 * @brief BEV 障碍物提取器
 *
 * 从 WorldTick 中提取 BEV (Bird's Eye View) 障碍物信息。
 * 这是一个公共前置处理组件，不是插件。
 *
 * 参考原项目的 BEVObstacleExtractor 实现。
 */
class BEVExtractor {
public:
  struct Config {
    double detection_range = 50.0;    // 检测范围 (m)
    double confidence_threshold = 0.5; // 置信度阈值
  };

  /**
   * @brief 构造函数
   */
  BEVExtractor() = default;
  explicit BEVExtractor(const Config& config) : config_(config) {}

  /**
   * @brief 从 WorldTick 提取 BEV 障碍物
   *
   * @param world_tick 输入的世界状态
   * @return BEV 障碍物数据
   */
  std::unique_ptr<planning::BEVObstacles> extract(const proto::WorldTick& world_tick);

  /**
   * @brief 重置提取器状态
   */
  void reset();

private:
  Config config_;

  // 静态地图缓存（因为静态地图只在版本变更时传输）
  mutable proto::StaticMap cached_static_map_;
  mutable bool has_cached_static_map_ = false;

  void extractStaticObstacles(const proto::WorldTick& world_tick,
                             planning::BEVObstacles& obstacles);
  void extractDynamicObstacles(const proto::WorldTick& world_tick,
                              planning::BEVObstacles& obstacles);

  // 统计信息
  size_t total_extractions_ = 0;
};

/**
 * @brief 动态障碍物预测器
 *
 * 预测动态障碍物的未来轨迹。
 * 这是一个公共前置处理组件，不是插件。
 *
 * 参考原项目的 DynamicObstaclePredictor 实现。
 */
class DynamicObstaclePredictor {
public:
  struct Config {
    double prediction_horizon = 5.0;   // 预测时域 (s)
    double time_step = 0.1;           // 时间步长 (s)
    std::string prediction_model = "constant_velocity"; // 预测模型
  };

  /**
   * @brief 构造函数
   */
  DynamicObstaclePredictor() = default;
  explicit DynamicObstaclePredictor(const Config& config) : config_(config) {}

  /**
   * @brief 预测动态障碍物
   *
   * @param world_tick 输入的世界状态
   * @return 预测的动态障碍物列表
   */
  std::vector<planning::DynamicObstacle> predict(const proto::WorldTick& world_tick);

  /**
   * @brief 设置预测参数
   */
  void setConfig(const Config& config) { config_ = config; }

  /**
   * @brief 重置预测器状态
   */
  void reset();

private:
  Config config_;

  void predictConstantVelocity(const proto::WorldTick& world_tick,
                              std::vector<planning::DynamicObstacle>& obstacles);

  // 统计信息
  size_t total_predictions_ = 0;
};

/**
 * @brief 基础数据转换器
 *
 * 将 WorldTick 中的基础数据转换为规划所需的格式。
 * 这是一个公共前置处理组件，不是插件。
 *
 * 参考原项目的 BasicDataConverter 实现。
 */
class BasicDataConverter {
public:
  /**
   * @brief 转换自车状态
   */
  static planning::EgoVehicle convertEgo(const proto::WorldTick& world_tick);

  /**
   * @brief 转换任务目标
   */
  static planning::PlanningTask convertTask(const proto::WorldTick& world_tick);

  /**
   * @brief 转换基础上下文信息
   */
  static void convertBasicContext(const proto::WorldTick& world_tick,
                                 planning::PlanningContext& context);
};

/**
 * @brief 前置处理管道
 *
 * 整合所有前置处理组件，将 WorldTick 转换为 PerceptionInput。
 */
class PreprocessingPipeline {
public:
  struct Config {
    BEVExtractor::Config bev_config;
    DynamicObstaclePredictor::Config predictor_config;
  };

  /**
   * @brief 构造函数
   */
  PreprocessingPipeline() = default;
  explicit PreprocessingPipeline(const Config& config);

  /**
   * @brief 处理 WorldTick，生成 PerceptionInput
   *
   * @param world_tick 输入的世界状态
   * @return 标准化的感知输入
   */
  plugin::PerceptionInput process(const proto::WorldTick& world_tick);

  /**
   * @brief 设置配置
   */
  void setConfig(const Config& config);

  /**
   * @brief 重置所有组件
   */
  void reset();

  /**
   * @brief 获取统计信息
   */
  struct Statistics {
    size_t total_processed = 0;
    double average_time_ms = 0.0;
  };

  Statistics getStatistics() const {
    return stats_;
  }

private:
  Config config_;

  // 前置处理组件
  BEVExtractor bev_extractor_;
  DynamicObstaclePredictor dynamic_predictor_;

  // 统计信息
  Statistics stats_;
};

} // namespace perception
} // namespace navsim


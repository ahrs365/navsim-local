#pragma once

#include "planning_context.hpp"
#include "world_tick.pb.h"
#include <memory>
#include <functional>

namespace navsim {
namespace perception {

/**
 * @brief 感知处理器基类
 * 负责将原始的 WorldTick 消息转换为规划所需的感知数据
 */
class PerceptionProcessor {
public:
  virtual ~PerceptionProcessor() = default;

  /**
   * @brief 处理 WorldTick 消息，生成感知数据
   * @param world_tick 原始世界状态消息
   * @param context 输出的规划上下文 (会被修改)
   * @return 处理是否成功
   */
  virtual bool process(const proto::WorldTick& world_tick,
                      planning::PlanningContext& context) = 0;

  /**
   * @brief 获取处理器名称 (用于调试和配置)
   */
  virtual std::string getName() const = 0;
};

/**
 * @brief 栅格地图构建器
 * 将 WorldTick 中的静态/动态障碍物转换为栅格占据地图
 */
class OccupancyGridBuilder : public PerceptionProcessor {
public:
  struct Config {
    double resolution = 0.1;      // 栅格分辨率 (m/cell)
    double map_width = 100.0;     // 地图宽度 (m)
    double map_height = 100.0;    // 地图高度 (m)
    uint8_t obstacle_cost = 100;  // 障碍物代价值
    double inflation_radius = 0.5; // 膨胀半径 (m)
  };

  OccupancyGridBuilder();
  explicit OccupancyGridBuilder(const Config& config);

  bool process(const proto::WorldTick& world_tick,
               planning::PlanningContext& context) override;

  std::string getName() const override { return "OccupancyGridBuilder"; }

private:
  Config config_;

  void addStaticObstacles(const proto::WorldTick& world_tick,
                         planning::OccupancyGrid& grid);
  void addDynamicObstacles(const proto::WorldTick& world_tick,
                          planning::OccupancyGrid& grid);
  void inflateObstacles(planning::OccupancyGrid& grid);
};

/**
 * @brief BEV障碍物提取器
 * 将 WorldTick 中的障碍物转换为BEV表示
 */
class BEVObstacleExtractor : public PerceptionProcessor {
public:
  struct Config {
    double detection_range = 50.0;    // 检测范围 (m)
    double confidence_threshold = 0.5; // 置信度阈值
    bool track_dynamic_only = false;   // 是否只跟踪动态障碍物
  };

  BEVObstacleExtractor();
  explicit BEVObstacleExtractor(const Config& config);

  bool process(const proto::WorldTick& world_tick,
               planning::PlanningContext& context) override;

  std::string getName() const override { return "BEVObstacleExtractor"; }

private:
  Config config_;

  // 静态地图缓存（因为静态地图只在版本变更时传输）
  mutable proto::StaticMap cached_static_map_;
  mutable bool has_cached_static_map_ = false;

  void extractStaticObstacles(const proto::WorldTick& world_tick,
                             planning::BEVObstacles& obstacles);
  void extractDynamicObstacles(const proto::WorldTick& world_tick,
                              planning::BEVObstacles& obstacles);
};

/**
 * @brief 动态障碍物预测器
 * 为动态障碍物生成预测轨迹
 */
class DynamicObstaclePredictor : public PerceptionProcessor {
public:
  struct Config {
    double prediction_horizon = 5.0;   // 预测时域 (s)
    double time_step = 0.1;           // 时间步长 (s)
    int max_trajectories = 3;         // 每个障碍物最大轨迹数
    std::string prediction_model = "constant_velocity"; // 预测模型
  };

  DynamicObstaclePredictor();
  explicit DynamicObstaclePredictor(const Config& config);

  bool process(const proto::WorldTick& world_tick,
               planning::PlanningContext& context) override;

  std::string getName() const override { return "DynamicObstaclePredictor"; }

private:
  Config config_;

  void predictConstantVelocity(const proto::WorldTick& world_tick,
                              std::vector<planning::DynamicObstacle>& obstacles);
  void predictConstantAcceleration(const proto::WorldTick& world_tick,
                                  std::vector<planning::DynamicObstacle>& obstacles);
};

/**
 * @brief 感知处理管线
 * 组合多个感知处理器，按顺序执行
 */
class PerceptionPipeline {
public:
  /**
   * @brief 添加感知处理器
   * @param processor 处理器实例
   * @param enabled 是否启用
   */
  void addProcessor(std::unique_ptr<PerceptionProcessor> processor, bool enabled = true);

  /**
   * @brief 启用/禁用特定处理器
   */
  void enableProcessor(const std::string& name, bool enabled);

  /**
   * @brief 处理 WorldTick，生成规划上下文
   * @param world_tick 输入的世界状态
   * @param context 输出的规划上下文
   * @return 处理是否成功
   */
  bool process(const proto::WorldTick& world_tick,
               planning::PlanningContext& context);

  /**
   * @brief 获取处理器状态
   */
  std::vector<std::pair<std::string, bool>> getProcessorStatus() const;

private:
  struct ProcessorEntry {
    std::unique_ptr<PerceptionProcessor> processor;
    bool enabled;
  };

  std::vector<ProcessorEntry> processors_;
};

/**
 * @brief 基础数据转换器
 * 处理 ego、goal 等基础数据的转换
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

} // namespace perception
} // namespace navsim
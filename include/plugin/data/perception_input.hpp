#pragma once

#include "core/planning_context.hpp"
#include <cstdint>

// 前向声明
namespace navsim {
namespace proto {
class WorldTick;
}
}

namespace navsim {
namespace plugin {

/**
 * @brief 感知插件的标准化输入数据
 * 
 * 这是所有感知插件的统一输入接口。数据由公共前置处理层生成，
 * 包含已解析的 BEV 障碍物、动态障碍物预测等标准化数据。
 * 
 * 设计原则：
 * - 感知插件专注于构建特定的地图表示（栅格地图、ESDF、点云等）
 * - 避免重复解析原始数据（BEV 提取和动态预测只执行一次）
 * - 提供可选的原始数据访问（高级插件可能需要）
 */
struct PerceptionInput {
  // ========== 基础数据 (必需) ==========
  
  /**
   * @brief 自车状态
   * 包含位置、速度、车辆参数、动力学约束等
   */
  planning::EgoVehicle ego;
  
  /**
   * @brief 规划任务
   * 包含目标点、任务类型、容差等
   */
  planning::PlanningTask task;
  
  // ========== 标准化感知数据 (已解析) ==========
  
  /**
   * @brief BEV 障碍物
   * 由 BEVExtractor 从 WorldTick 提取
   * 包含圆形、矩形、多边形障碍物
   */
  planning::BEVObstacles bev_obstacles;
  
  /**
   * @brief 动态障碍物预测
   * 由 DynamicObstaclePredictor 生成
   * 包含障碍物的预测轨迹
   */
  std::vector<planning::DynamicObstacle> dynamic_obstacles;
  
  // ========== 原始数据 (可选) ==========
  
  /**
   * @brief 原始 WorldTick 数据指针
   *
   * 大多数感知插件不需要访问原始数据，因为标准化数据已经足够。
   * 但某些高级插件可能需要访问原始传感器数据（如点云、图像等）。
   *
   * 注意：这是一个非拥有指针，插件不应该存储它。
   */
  const navsim::proto::WorldTick* raw_world_tick = nullptr;
  
  // ========== 元数据 ==========
  
  /**
   * @brief 时间戳 (秒)
   */
  double timestamp = 0.0;
  
  /**
   * @brief Tick ID
   * 用于调试和日志记录
   */
  uint64_t tick_id = 0;
  
  // ========== 构造函数 ==========
  
  /**
   * @brief 默认构造函数
   */
  PerceptionInput() = default;
  
  /**
   * @brief 完整构造函数
   */
  PerceptionInput(
      const planning::EgoVehicle& ego_,
      const planning::PlanningTask& task_,
      const planning::BEVObstacles& bev_obstacles_,
      const std::vector<planning::DynamicObstacle>& dynamic_obstacles_,
      double timestamp_,
      uint64_t tick_id_,
      const proto::WorldTick* raw_world_tick_ = nullptr)
      : ego(ego_),
        task(task_),
        bev_obstacles(bev_obstacles_),
        dynamic_obstacles(dynamic_obstacles_),
        raw_world_tick(raw_world_tick_),
        timestamp(timestamp_),
        tick_id(tick_id_) {}
};

} // namespace plugin
} // namespace navsim


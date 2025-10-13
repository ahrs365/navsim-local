#pragma once

#include "core/planning_context.hpp"
#include <string>
#include <vector>
#include <map>

namespace navsim {
namespace plugin {

/**
 * @brief 轨迹点
 * 规划器输出的轨迹由一系列轨迹点组成
 */
struct TrajectoryPoint {
  // 位置和朝向
  planning::Pose2d pose;
  
  // 速度
  planning::Twist2d twist;
  
  // 加速度
  double acceleration = 0.0;  // 纵向加速度 (m/s²)
  
  // 转向
  double steering_angle = 0.0;  // 转向角 (rad)
  
  // 曲率
  double curvature = 0.0;  // 曲率 (1/m)
  
  // 时间
  double time_from_start = 0.0;  // 从起点开始的时间 (s)
  
  // 路径长度
  double path_length = 0.0;  // 从起点开始的路径长度 (m)
  
  // 默认构造函数
  TrajectoryPoint() = default;
  
  // 简化构造函数
  TrajectoryPoint(const planning::Pose2d& pose_, const planning::Twist2d& twist_, double time_)
      : pose(pose_), twist(twist_), time_from_start(time_) {}
};

/**
 * @brief 规划结果
 * 
 * 规划器的输出，包含轨迹、成功状态、元数据等。
 */
struct PlanningResult {
  // ========== 轨迹 ==========
  
  /**
   * @brief 规划的轨迹
   * 按时间顺序排列的轨迹点序列
   */
  std::vector<TrajectoryPoint> trajectory;
  
  // ========== 状态 ==========
  
  /**
   * @brief 规划是否成功
   */
  bool success = false;
  
  /**
   * @brief 失败原因（如果 success = false）
   */
  std::string failure_reason;
  
  /**
   * @brief 规划器名称
   * 用于调试和日志记录
   */
  std::string planner_name;
  
  // ========== 性能指标 ==========
  
  /**
   * @brief 计算时间 (毫秒)
   */
  double computation_time_ms = 0.0;
  
  /**
   * @brief 迭代次数
   * 对于迭代式规划器（如优化规划器）
   */
  int iterations = 0;
  
  // ========== 代价和约束 ==========
  
  /**
   * @brief 总代价
   * 轨迹的总代价值（如果适用）
   */
  double total_cost = 0.0;
  
  /**
   * @brief 是否满足所有约束
   */
  bool constraints_satisfied = true;
  
  /**
   * @brief 约束违反信息
   * 键：约束名称，值：违反程度
   */
  std::map<std::string, double> constraint_violations;
  
  // ========== 元数据 ==========
  
  /**
   * @brief 自定义元数据
   * 规划器可以添加任意元数据用于调试和分析
   * 
   * 常见用途：
   * - "smoothness": 轨迹平滑度
   * - "clearance": 与障碍物的最小距离
   * - "comfort": 舒适度指标
   * - "efficiency": 效率指标
   */
  std::map<std::string, double> metadata;
  
  // ========== 工具函数 ==========
  
  /**
   * @brief 清空结果
   */
  void clear() {
    trajectory.clear();
    success = false;
    failure_reason.clear();
    planner_name.clear();
    computation_time_ms = 0.0;
    iterations = 0;
    total_cost = 0.0;
    constraints_satisfied = true;
    constraint_violations.clear();
    metadata.clear();
  }
  
  /**
   * @brief 获取轨迹长度（点数）
   */
  size_t size() const {
    return trajectory.size();
  }
  
  /**
   * @brief 检查轨迹是否为空
   */
  bool empty() const {
    return trajectory.empty();
  }
  
  /**
   * @brief 获取轨迹总时长 (秒)
   */
  double getTotalTime() const {
    if (trajectory.empty()) {
      return 0.0;
    }
    return trajectory.back().time_from_start;
  }
  
  /**
   * @brief 获取轨迹总长度 (米)
   */
  double getTotalLength() const {
    if (trajectory.empty()) {
      return 0.0;
    }
    return trajectory.back().path_length;
  }
  
  /**
   * @brief 设置元数据
   */
  void setMetadata(const std::string& key, double value) {
    metadata[key] = value;
  }
  
  /**
   * @brief 获取元数据
   */
  double getMetadata(const std::string& key, double default_value = 0.0) const {
    auto it = metadata.find(key);
    if (it != metadata.end()) {
      return it->second;
    }
    return default_value;
  }
  
  /**
   * @brief 检查是否有指定的元数据
   */
  bool hasMetadata(const std::string& key) const {
    return metadata.find(key) != metadata.end();
  }
};

} // namespace plugin
} // namespace navsim


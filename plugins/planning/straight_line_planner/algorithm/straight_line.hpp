#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace straight_line {
namespace algorithm {

/**
 * @brief StraightLine 算法核心实现
 * 
 * 这是纯算法层，不依赖任何平台 API。
 * 只使用标准库和 Eigen，便于复用到其他项目。
 * 
 * 设计原则：
 * - 输入输出都是简单的数据结构（Eigen 向量、STL 容器）
 * - 无状态或状态可重置
 * - 易于单元测试
 */
class StraightLine {
public:
  /**
   * @brief 算法配置参数
   *
   * 注意：这是纯数据结构，不包含 JSON 解析逻辑。
   * JSON 解析应该在 adapter 层完成。
   */
  struct Config {
    double default_velocity = 2.0;        // 默认速度 (m/s)
    double time_step = 0.1;               // 时间步长 (s)
    double planning_horizon = 5.0;        // 规划时域 (s)
    bool use_trapezoidal_profile = true;  // 是否使用梯形速度曲线
    double max_acceleration = 2.0;        // 最大加速度 (m/s²)
  };
  
  /**
   * @brief 路径点
   */
  struct Waypoint {
    Eigen::Vector3d position;  // (x, y, yaw)
    double velocity = 0.0;     // 速度 (m/s)
    double acceleration = 0.0; // 加速度 (m/s²)
    double timestamp = 0.0;    // 时间戳 (s)
    double path_length = 0.0;  // 路径长度 (m)
  };
  
  /**
   * @brief 规划结果
   */
  struct Result {
    bool success = false;
    std::vector<Waypoint> path;
    std::string failure_reason;
    double computation_time_ms = 0.0;
  };
  
  /**
   * @brief 构造函数
   */
  StraightLine() = default;
  explicit StraightLine(const Config& config);
  
  /**
   * @brief 设置配置
   */
  void setConfig(const Config& config);
  
  /**
   * @brief 规划路径
   * 
   * @param start 起点 (x, y, yaw)
   * @param goal 终点 (x, y, yaw)
   * @return 规划结果
   * 
   * 示例：
   * ```cpp
   * StraightLine planner(config);
   * auto result = planner.plan(
   *   Eigen::Vector3d(0, 0, 0),
   *   Eigen::Vector3d(10, 10, 0)
   * );
   * if (result.success) {
   *   for (const auto& wp : result.path) {
   *     std::cout << "(" << wp.position.x() << ", " << wp.position.y() << ")" << std::endl;
   *   }
   * }
   * ```
   */
  Result plan(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& goal);
  
  /**
   * @brief 重置算法状态
   */
  void reset();
  
private:
  /**
   * @brief 生成直线路径点
   */
  std::vector<Waypoint> generateStraightLine(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& goal,
      int num_points) const;

  /**
   * @brief 计算匀速速度曲线
   */
  void computeVelocityProfile(
      std::vector<Waypoint>& trajectory,
      double total_distance) const;

  /**
   * @brief 计算梯形速度曲线
   */
  void computeTrapezoidalProfile(
      std::vector<Waypoint>& trajectory,
      double total_distance) const;

  Config config_;
};

} // namespace algorithm
} // namespace straight_line


#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace {{NAMESPACE}} {
namespace algorithm {

/**
 * @brief {{PLUGIN_NAME}} 算法核心实现
 * 
 * 这是纯算法层，不依赖任何平台 API。
 * 只使用标准库和 Eigen，便于复用到其他项目。
 * 
 * 设计原则：
 * - 输入输出都是简单的数据结构（Eigen 向量、STL 容器）
 * - 无状态或状态可重置
 * - 易于单元测试
 */
class {{PLUGIN_NAME}} {
public:
  /**
   * @brief 算法配置参数
   *
   * 注意：这是纯数据结构，不包含 JSON 解析逻辑。
   * JSON 解析应该在 adapter 层完成。
   */
  struct Config {
    // TODO: 添加您的配置参数
    double max_velocity = 2.0;      // 最大速度 (m/s)
    double max_acceleration = 2.0;  // 最大加速度 (m/s²)
    double step_size = 0.1;         // 步长 (m)
    int max_iterations = 1000;      // 最大迭代次数
  };
  
  /**
   * @brief 路径点
   */
  struct Waypoint {
    Eigen::Vector3d position;  // (x, y, yaw)
    double velocity = 0.0;     // 速度 (m/s)
    double timestamp = 0.0;    // 时间戳 (s)
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
  {{PLUGIN_NAME}}() = default;
  explicit {{PLUGIN_NAME}}(const Config& config);
  
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
   * {{PLUGIN_NAME}} planner(config);
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
  Config config_;
  
  // TODO: 添加您的私有成员变量
  // 例如：地图、缓存、统计信息等
};

} // namespace algorithm
} // namespace {{NAMESPACE}}


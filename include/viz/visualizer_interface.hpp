#pragma once

#include "core/planning_context.hpp"
#include "plugin/data/perception_input.hpp"
#include "plugin/data/planning_result.hpp"
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace navsim {
namespace viz {

/**
 * @brief 可视化器接口
 * 
 * 提供统一的可视化接口，支持多种实现：
 * - ImGuiVisualizer: 实时 GUI 可视化
 * - NullVisualizer: 空实现（禁用可视化时零开销）
 */
class IVisualizer {
public:
  virtual ~IVisualizer() = default;

  struct SystemInfo {
    std::map<std::string, std::string> general;
    std::vector<std::string> perception_plugins;
    std::vector<std::string> planner_plugins;
  };

  struct ConnectionStatus {
    bool connected = false;
    std::string label;   // 目标地址或房间信息
    std::string message; // 额外说明
  };

  /**
   * @brief 初始化可视化器
   * @return 是否成功初始化
   */
  virtual bool initialize() = 0;

  /**
   * @brief 开始新的一帧
   * 在每次 process() 开始时调用
   */
  virtual void beginFrame() = 0;

  /**
   * @brief 更新系统静态信息
   */
  virtual void setSystemInfo(const SystemInfo& info) = 0;

  /**
   * @brief 更新连接状态
   */
  virtual void updateConnectionStatus(const ConnectionStatus& status) = 0;

  /**
   * @brief 绘制自车
   */
  virtual void drawEgo(const planning::EgoVehicle& ego) = 0;

  /**
   * @brief 绘制目标点
   */
  virtual void drawGoal(const planning::Pose2d& goal) = 0;

  /**
   * @brief 绘制 BEV 障碍物
   */
  virtual void drawBEVObstacles(const planning::BEVObstacles& obstacles) = 0;

  /**
   * @brief 绘制动态障碍物
   */
  virtual void drawDynamicObstacles(const std::vector<planning::DynamicObstacle>& obstacles) = 0;

  /**
   * @brief 绘制栅格地图
   */
  virtual void drawOccupancyGrid(const planning::OccupancyGrid& grid) = 0;

  /**
   * @brief 绘制规划轨迹
   */
  virtual void drawTrajectory(const std::vector<plugin::TrajectoryPoint>& trajectory,
                               const std::string& planner_name = "") = 0;

  /**
   * @brief 更新规划上下文信息
   */
  virtual void updatePlanningContext(const planning::PlanningContext& context) = 0;

  /**
   * @brief 更新规划结果信息
   */
  virtual void updatePlanningResult(const plugin::PlanningResult& result) = 0;

  /**
   * @brief 显示调试信息
   */
  virtual void showDebugInfo(const std::string& key, const std::string& value) = 0;

  /**
   * @brief 结束当前帧并渲染
   * 在每次 process() 结束时调用
   */
  virtual void endFrame() = 0;

  /**
   * @brief 检查窗口是否应该关闭
   */
  virtual bool shouldClose() const = 0;

  /**
   * @brief 清理资源
   */
  virtual void shutdown() = 0;
};

/**
 * @brief 空可视化器（禁用可视化时使用）
 * 所有函数都是空实现，编译器会优化掉这些调用
 */
class NullVisualizer : public IVisualizer {
public:
  bool initialize() override { return true; }
  void beginFrame() override {}
  void setSystemInfo(const SystemInfo&) override {}
  void updateConnectionStatus(const ConnectionStatus&) override {}
  void drawEgo(const planning::EgoVehicle&) override {}
  void drawGoal(const planning::Pose2d&) override {}
  void drawBEVObstacles(const planning::BEVObstacles&) override {}
  void drawDynamicObstacles(const std::vector<planning::DynamicObstacle>&) override {}
  void drawOccupancyGrid(const planning::OccupancyGrid&) override {}
  void drawTrajectory(const std::vector<plugin::TrajectoryPoint>&, const std::string&) override {}
  void updatePlanningContext(const planning::PlanningContext&) override {}
  void updatePlanningResult(const plugin::PlanningResult&) override {}
  void showDebugInfo(const std::string&, const std::string&) override {}
  void endFrame() override {}
  bool shouldClose() const override { return false; }
  void shutdown() override {}
};

/**
 * @brief 创建可视化器工厂函数
 */
std::unique_ptr<IVisualizer> createVisualizer(bool enable_gui = true);

} // namespace viz
} // namespace navsim

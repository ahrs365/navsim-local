#pragma once

#include "viz/visualizer_interface.hpp"
#include "world_tick.pb.h"
#include "plan_update.pb.h"
#include "ego_cmd.pb.h"
#include <memory>
#include <chrono>
#include <string>

// 前向声明
namespace navsim {
namespace plugin {
  class PerceptionPluginManager;
  class PlannerPluginManager;
}
namespace viz {
  class IVisualizer;
}
}

namespace navsim {

// 前向声明
class Bridge;

/**
 * @brief 算法管理器
 * 整合感知、规划、控制模块，提供统一的接口
 */
class AlgorithmManager {
public:
  struct Config {
    // 插件系统配置
    std::string config_file = "";          // 插件配置文件路径（为空则使用默认配置）

    // 规划配置
    std::string primary_planner = "StraightLinePlanner";
    std::string fallback_planner = "StraightLinePlanner";
    bool enable_planner_fallback = true;

    // 性能配置
    double max_computation_time_ms = 25.0;  // 最大计算时间
    bool verbose_logging = false;           // 详细日志

    // 可视化配置
    bool enable_visualization = false;      // 启用实时可视化
  };

  AlgorithmManager();
  explicit AlgorithmManager(const Config& config);
  ~AlgorithmManager();

  /**
   * @brief 初始化算法模块
   */
  bool initialize();

  /**
   * @brief 处理世界状态，生成规划结果
   * @param world_tick 输入的世界状态
   * @param deadline 规划截止时间
   * @param plan_update 输出的规划更新 (轨迹)
   * @param ego_cmd 输出的控制指令
   * @return 处理是否成功
   */
  bool process(const proto::WorldTick& world_tick,
               std::chrono::milliseconds deadline,
               proto::PlanUpdate& plan_update,
               proto::EgoCmd& ego_cmd);

  /**
   * @brief 获取算法统计信息
   */
  struct Statistics {
    int total_processed = 0;
    int successful_processed = 0;
    int perception_failures = 0;
    int planning_failures = 0;
    double avg_computation_time_ms = 0.0;
    double avg_perception_time_ms = 0.0;
    double avg_planning_time_ms = 0.0;
  };

  Statistics getStatistics() const { return stats_; }

  /**
   * @brief 重置统计信息
   */
  void resetStatistics() { stats_ = Statistics{}; }

  /**
   * @brief 获取当前配置
   */
  const Config& getConfig() const { return config_; }

  /**
   * @brief 更新配置
   */
  void updateConfig(const Config& config);

  /**
   * @brief 设置Bridge引用（用于WebSocket可视化）
   */
  void setBridge(Bridge* bridge, const std::string& connection_label = "");

  /**
   * @brief 设置仿真状态（由 Bridge 的仿真状态回调调用）
   */
  void setSimulationStarted(bool started) {
    simulation_started_.store(started);
  }

  /**
   * @brief 获取仿真状态
   */
  bool isSimulationStarted() const {
    return simulation_started_.load();
  }

  /**
   * @brief 在等待数据时渲染空闲帧，确保窗口保持响应
   */
  void renderIdleFrame();

private:
  Config config_;
  Statistics stats_;

  // 插件系统模块
  std::unique_ptr<plugin::PerceptionPluginManager> perception_plugin_manager_;
  std::unique_ptr<plugin::PlannerPluginManager> planner_plugin_manager_;

  // Bridge引用（用于感知调试数据发送）
  Bridge* bridge_ = nullptr;

  // 可视化器
  std::unique_ptr<viz::IVisualizer> visualizer_;
  viz::IVisualizer::SystemInfo system_info_cache_;
  std::string connection_label_;
  std::string active_config_file_;

  // 仿真状态
  std::atomic<bool> simulation_started_{false};

  // 内部函数
  void setupPluginSystem();
  void updateStatistics(double total_time, double perception_time, double planning_time, bool success);
};

} // namespace navsim

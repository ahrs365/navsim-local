#pragma once

#include "core/planning_context.hpp"
#include "core/bridge.hpp"

namespace navsim {
namespace visualization {

/**
 * @brief WebSocket可视化器
 * 通过Bridge将感知数据发送到前端进行可视化
 */
class WebSocketVisualizer {
public:
  struct Config {
    bool enable_output = true;
    double update_interval_ms = 200.0;  // 更新间隔(ms)，默认5Hz
  };

private:
  Config config_;
  Bridge* bridge_ = nullptr;  // 不拥有Bridge的所有权
  double last_update_time_ = 0.0;

public:
  WebSocketVisualizer() = default;
  explicit WebSocketVisualizer(const Config& config);

  /**
   * @brief 设置Bridge引用
   */
  void setBridge(Bridge* bridge);

  /**
   * @brief 输出规划上下文到前端
   */
  void outputPlanningContext(const planning::PlanningContext& context);

  /**
   * @brief 更新配置
   */
  void updateConfig(const Config& config);

private:
  bool shouldUpdate(double current_time);
};

} // namespace visualization
} // namespace navsim
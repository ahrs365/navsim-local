#include "core/websocket_visualizer.hpp"
#include <iostream>

namespace navsim {
namespace visualization {

WebSocketVisualizer::WebSocketVisualizer(const Config& config) : config_(config) {}

void WebSocketVisualizer::setBridge(Bridge* bridge) {
  bridge_ = bridge;
}

void WebSocketVisualizer::outputPlanningContext(const planning::PlanningContext& context) {
  if (!config_.enable_output || !bridge_) {
    return;
  }

  if (!shouldUpdate(context.timestamp)) {
    return;
  }

  // 通过Bridge发送感知数据到前端
  bridge_->send_perception_debug(context);
}

bool WebSocketVisualizer::shouldUpdate(double current_time) {
  if (current_time - last_update_time_ >= config_.update_interval_ms / 1000.0) {
    last_update_time_ = current_time;
    return true;
  }
  return false;
}

void WebSocketVisualizer::updateConfig(const Config& config) {
  config_ = config;
}

} // namespace visualization
} // namespace navsim
#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "ego_cmd.pb.h"
#include "plan_update.pb.h"
#include "world_tick.pb.h"
#include "core/planning_context.hpp"

namespace navsim {

class Bridge {
 public:
  using WorldTickCallback = std::function<void(const proto::WorldTick&)>;
  using SimulationStateCallback = std::function<void(bool)>;  // 仿真状态回调：true=运行，false=暂停

  Bridge();
  ~Bridge();

  // 连接到 WebSocket 服务器
  // url: WebSocket URL (如 "ws://127.0.0.1:8080/ws")
  // room_id: 房间 ID (如 "demo")
  // Bridge 会拼接为: ws://host/ws?room=<room_id>
  void connect(const std::string& url, const std::string& room_id);

  // 启动接收循环（设置回调）
  void start(const WorldTickCallback& on_world_tick);

  // 设置仿真状态回调（监听开始/暂停事件）
  void set_simulation_state_callback(const SimulationStateCallback& callback);

  // 获取当前仿真状态
  bool is_simulation_running() const;

  // 发送 plan 消息（不发送 ego_cmd）
  void publish(const proto::PlanUpdate& plan, double compute_ms);

  // 发送心跳消息
  void send_heartbeat(double loop_hz);

  // 发送感知调试数据到前端
  void send_perception_debug(const planning::PlanningContext& context);

  // 设置感知调试开关
  void set_perception_debug_enabled(bool enabled);

  // 获取感知调试状态
  bool is_perception_debug_enabled() const;

  // 停止连接
  void stop();

  // 检查连接状态
  bool is_connected() const;

  // 获取统计信息
  uint64_t get_ws_rx() const;
  uint64_t get_ws_tx() const;
  uint64_t get_dropped_ticks() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navsim

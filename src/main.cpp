#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "bridge.hpp"
#include "planner.hpp"
#include "algorithm_manager.hpp"
#include "world_tick.pb.h"
#include "plan_update.pb.h"
#include "ego_cmd.pb.h"

namespace navsim {
namespace {

struct SharedState {
  std::mutex mutex;
  std::condition_variable cv;
  std::optional<proto::WorldTick> latest_world;
  uint64_t latest_tick_id = 0;
  double latest_stamp = 0.0;
  bool shutdown = false;
};

std::atomic<bool> g_interrupt{false};

void signal_handler(int) {
  g_interrupt.store(true);
}

void print_usage(const char* prog) {
  std::cerr << "Usage: " << prog << " <ws_url> <room_id>" << std::endl;
  std::cerr << "Example: " << prog << " ws://127.0.0.1:8080/ws demo" << std::endl;
}

}  // namespace
}  // namespace navsim

int main(int argc, char* argv[]) {
  using namespace std::chrono_literals;

  // 解析命令行参数
  if (argc != 3) {
    navsim::print_usage(argv[0]);
    return 1;
  }

  std::string ws_url = argv[1];
  std::string room_id = argv[2];

  std::cout << "=== NavSim Local Algorithm ===" << std::endl;
  std::cout << "WebSocket URL: " << ws_url << std::endl;
  std::cout << "Room ID: " << room_id << std::endl;
  std::cout << "===============================" << std::endl;

  std::signal(SIGINT, navsim::signal_handler);

  // 初始化算法管理器
  navsim::AlgorithmManager::Config algo_config;

  // 检查环境变量以决定使用哪个系统
  const char* use_plugin_env = std::getenv("USE_PLUGIN_SYSTEM");
  algo_config.use_plugin_system = (use_plugin_env && std::string(use_plugin_env) == "1");

  algo_config.primary_planner = "AStarPlanner";  // 使用 A* 作为主规划器
  algo_config.fallback_planner = "StraightLinePlanner";  // 使用直线作为降级规划器
  algo_config.enable_occupancy_grid = true;
  algo_config.enable_bev_obstacles = true;
  algo_config.enable_dynamic_prediction = true;

  // 检查环境变量以决定是否启用详细日志
  const char* verbose_env = std::getenv("VERBOSE");
  algo_config.verbose_logging = (verbose_env && std::string(verbose_env) == "1");

  std::cout << "Using " << (algo_config.use_plugin_system ? "PLUGIN" : "LEGACY") << " system" << std::endl;

  navsim::AlgorithmManager algorithm_manager(algo_config);
  if (!algorithm_manager.initialize()) {
    std::cerr << "Failed to initialize algorithm manager" << std::endl;
    return 1;
  }

  // 保留原有的Bridge和共享状态
  navsim::Bridge bridge;
  navsim::SharedState shared;

  // 设置Bridge引用以支持WebSocket可视化
  algorithm_manager.setBridge(&bridge);

  // 连接到 WebSocket 服务器
  bridge.connect(ws_url, room_id);

  // 启动 Planner 线程
  std::thread planner_thread([&]() {
    std::optional<navsim::proto::PlanUpdate> last_plan;
    auto last_heartbeat = std::chrono::steady_clock::now();
    uint64_t loop_count = 0;
    auto loop_start = std::chrono::steady_clock::now();

    while (true) {
      navsim::proto::WorldTick world;
      uint64_t tick_id = 0;
      {
        std::unique_lock<std::mutex> lock(shared.mutex);
        shared.cv.wait_for(lock, 100ms, [&]() {
          return shared.shutdown || shared.latest_world.has_value();
        });
        if (shared.shutdown && !shared.latest_world) {
          break;
        }
        if (!shared.latest_world) {
          continue;  // 超时，继续等待
        }
        world = *shared.latest_world;
        tick_id = shared.latest_tick_id;
        shared.latest_world.reset();
      }

      // 使用新的算法管理器进行规划
      const auto start = std::chrono::steady_clock::now();
      navsim::proto::PlanUpdate plan;
      navsim::proto::EgoCmd cmd;
      const auto deadline = std::chrono::milliseconds(25);  // 稍微减少，为感知处理留时间

      bool ok = algorithm_manager.process(world, deadline, plan, cmd);
      const auto duration = std::chrono::steady_clock::now() - start;
      const auto ms = std::chrono::duration<double, std::milli>(duration).count();

      if (!ok) {
        std::cerr << "[AlgorithmManager] WARN: Failed to process, sending fallback" << std::endl;
        // 发送静止计划（兜底策略）
        plan.Clear();
        plan.set_tick_id(tick_id);
        plan.set_stamp(std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count());
        // plan.set_status("fallback");  // protobuf中没有status字段

        auto* pt = plan.add_trajectory();
        pt->set_x(world.ego().pose().x());
        pt->set_y(world.ego().pose().y());
        pt->set_yaw(world.ego().pose().yaw());
        pt->set_t(0.0);
      }

      if (duration > deadline) {
        std::cerr << "[AlgorithmManager] WARN: Deadline exceeded (" << ms << " ms)" << std::endl;
      } else {
        std::cout << "[AlgorithmManager] Processed in " << std::fixed << std::setprecision(1)
                  << ms << " ms, trajectory points: " << plan.trajectory_size() << std::endl;
      }

      // 发送 plan
      bridge.publish(plan, ms);
      last_plan = plan;
      loop_count++;

      // 每 5 秒发送一次心跳
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_heartbeat).count() >= 5) {
        // 计算 loop_hz
        auto elapsed = std::chrono::duration<double>(now - loop_start).count();
        double loop_hz = loop_count / elapsed;

        bridge.send_heartbeat(loop_hz);

        last_heartbeat = now;
        loop_start = now;
        loop_count = 0;
      }
    }
  });

  // 启动 Bridge（设置回调）
  bridge.start([&](const navsim::proto::WorldTick& world) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.latest_world = world;
    shared.latest_tick_id = world.tick_id();
    shared.latest_stamp = world.stamp();
    shared.cv.notify_one();
  });

  // 主线程等待中断信号
  std::cout << "[Main] Waiting for world_tick messages... (Press Ctrl+C to exit)" << std::endl;
  while (!navsim::g_interrupt.load()) {
    std::this_thread::sleep_for(100ms);
  }

  // 清理
  std::cout << "[Main] Shutting down..." << std::endl;
  {
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.shutdown = true;
  }
  shared.cv.notify_all();
  planner_thread.join();
  bridge.stop();

  // 打印统计信息
  std::cout << "=== Statistics ===" << std::endl;
  std::cout << "WebSocket RX: " << bridge.get_ws_rx() << std::endl;
  std::cout << "WebSocket TX: " << bridge.get_ws_tx() << std::endl;
  std::cout << "Dropped ticks: " << bridge.get_dropped_ticks() << std::endl;
  std::cout << "==================" << std::endl;

  std::cout << "navsim_algo exiting" << std::endl;
  return 0;
}

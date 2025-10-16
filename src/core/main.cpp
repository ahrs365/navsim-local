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
#include <fstream>

#include "core/bridge.hpp"
#include "core/algorithm_manager.hpp"
#include "world_tick.pb.h"
#include "plan_update.pb.h"
#include "ego_cmd.pb.h"
#include <json.hpp>

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
  std::cerr << "Usage: " << prog << " <ws_url> <room_id> [--config=<path>]" << std::endl;
  std::cerr << "Example: " << prog << " ws://127.0.0.1:8080/ws demo" << std::endl;
  std::cerr << "         " << prog << " ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json" << std::endl;
}

// 从配置文件加载算法配置
bool load_config_from_file(const std::string& config_path, navsim::AlgorithmManager::Config& config) {
  std::ifstream file(config_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open config file: " << config_path << std::endl;
    return false;
  }

  try {
    nlohmann::json j;
    file >> j;

    if (j.contains("algorithm")) {
      auto& algo = j["algorithm"];
      if (algo.contains("primary_planner")) {
        config.primary_planner = algo["primary_planner"].get<std::string>();
      }
      if (algo.contains("fallback_planner")) {
        config.fallback_planner = algo["fallback_planner"].get<std::string>();
      }
      if (algo.contains("enable_planner_fallback")) {
        config.enable_planner_fallback = algo["enable_planner_fallback"].get<bool>();
      }
      if (algo.contains("max_computation_time_ms")) {
        config.max_computation_time_ms = algo["max_computation_time_ms"].get<double>();
      }
      if (algo.contains("verbose_logging")) {
        config.verbose_logging = algo["verbose_logging"].get<bool>();
      }
      if (algo.contains("enable_visualization")) {
        config.enable_visualization = algo["enable_visualization"].get<bool>();
      }
    }

    // 🔧 读取栅格地图配置
    if (j.contains("perception") && j["perception"].contains("plugins")) {
      for (const auto& plugin : j["perception"]["plugins"]) {
        if (plugin.contains("name") && plugin["name"] == "GridMapBuilder") {
          if (plugin.contains("params")) {
            const auto& params = plugin["params"];
            if (params.contains("map_width")) {
              config.grid_map_width = params["map_width"].get<double>();
            }
            if (params.contains("map_height")) {
              config.grid_map_height = params["map_height"].get<double>();
            }
            if (params.contains("resolution")) {
              config.grid_resolution = params["resolution"].get<double>();
            }
            if (params.contains("inflation_radius")) {
              config.grid_inflation_radius = params["inflation_radius"].get<double>();
            }
          }
          break;  // 找到 GridMapBuilder 后退出循环
        }
      }
    }

    std::cout << "✅ Loaded config from: " << config_path << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse config file: " << e.what() << std::endl;
    return false;
  }
}

}  // namespace
}  // namespace navsim

int main(int argc, char* argv[]) {
  using namespace std::chrono_literals;

  // 解析命令行参数
  if (argc < 3) {
    navsim::print_usage(argv[0]);
    return 1;
  }

  std::string ws_url = argv[1];
  std::string room_id = argv[2];
  std::string config_file;

  // 解析可选参数
  for (int i = 3; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg.find("--config=") == 0) {
      config_file = arg.substr(9);
    }
  }

  std::cout << "=== NavSim Local Algorithm ===" << std::endl;
  std::cout << "WebSocket URL: " << ws_url << std::endl;
  std::cout << "Room ID: " << room_id << std::endl;
  if (!config_file.empty()) {
    std::cout << "Config File: " << config_file << std::endl;
  }
  std::cout << "===============================" << std::endl;

  std::signal(SIGINT, navsim::signal_handler);

  // 初始化算法管理器
  navsim::AlgorithmManager::Config algo_config;

  // 默认配置
  algo_config.primary_planner = "StraightLinePlanner";
  algo_config.fallback_planner = "StraightLinePlanner";
  algo_config.enable_visualization = false;  // 默认禁用

  // 从配置文件加载（如果提供）
  if (!config_file.empty()) {
    algo_config.config_file = config_file;  // ✅ 设置配置文件路径
    navsim::load_config_from_file(config_file, algo_config);
  }

  // 检查环境变量以决定是否启用详细日志
  const char* verbose_env = std::getenv("VERBOSE");
  if (verbose_env && std::string(verbose_env) == "1") {
    algo_config.verbose_logging = true;
  }

  std::cout << "Using PLUGIN system" << std::endl;

  navsim::AlgorithmManager algorithm_manager(algo_config);
  if (!algorithm_manager.initialize()) {
    std::cerr << "Failed to initialize algorithm manager" << std::endl;
    return 1;
  }

  // 保留原有的Bridge和共享状态
  navsim::Bridge bridge;
  navsim::SharedState shared;

  // 设置Bridge引用以支持WebSocket可视化
  algorithm_manager.setBridge(&bridge, ws_url + "?room=" + room_id);

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
          algorithm_manager.renderIdleFrame();
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

      // 🔧 如果仿真未开始，process() 会返回 false 并渲染空闲帧
      // 此时不发送 plan，直接跳过
      if (!algorithm_manager.isSimulationStarted()) {
        // 仿真未开始，不发送 plan
        continue;
      }

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

      // 🔧 发送感知调试数据（如果启用）
      // 注意：我们需要从 algorithm_manager 获取 PlanningContext
      // 但是 process() 方法没有返回 context，所以我们需要修改架构
      // 暂时先注释掉，需要重构 AlgorithmManager 来暴露 context
      // bridge.send_perception_debug(context);

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

  // 🔧 设置仿真状态回调（监听开始/暂停事件）
  bridge.set_simulation_state_callback([&algorithm_manager](bool running) {
    // 更新 AlgorithmManager 的仿真状态
    algorithm_manager.setSimulationStarted(running);
    if (running) {
      std::cout << "[Main] ✅ Simulation STARTED - algorithm will now process ticks" << std::endl;
    } else {
      std::cout << "[Main] ⏸️  Simulation PAUSED/RESET - algorithm will skip processing" << std::endl;
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

  // 主线程等待中断信号或可视化窗口关闭
  std::cout << "[Main] ⏸️  Waiting for simulation to start..." << std::endl;
  std::cout << "[Main] Please click the 'Start' button in the Web interface" << std::endl;
  while (!navsim::g_interrupt.load()) {
    std::this_thread::sleep_for(100ms);

#ifdef ENABLE_VISUALIZATION
    // 检查可视化窗口是否关闭
    // 注意：这需要在 AlgorithmManager 中暴露 visualizer 的 shouldClose() 方法
    // 暂时先不实现，因为可视化器在 planner_thread 中使用
#endif
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

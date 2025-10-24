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
#include "sim/local_simulator.hpp"
#include "world_tick.pb.h"
#include "plan_update.pb.h"
#include "ego_cmd.pb.h"
#include <json.hpp>

namespace navsim {
namespace {

struct CommandLineArgs {
  bool use_local_sim = false;
  std::string scenario_file;
  std::string ws_url;
  std::string room_id;
  std::string config_file;
  bool enable_visualization = false;

  bool is_valid() const {
    if (use_local_sim) {
      // 本地仿真模式：必须有scenario文件
      return !scenario_file.empty();
    } else {
      // WebSocket在线模式：必须有ws_url和room_id
      return !ws_url.empty() && !room_id.empty();
    }
  }
};

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
  std::cerr << "Usage: " << std::endl;
  std::cerr << "  WebSocket mode: " << prog << " <ws_url> <room_id> [--config=<path>]" << std::endl;
  std::cerr << "  Local sim mode: " << prog << " --local-sim --scenario=<scene_file> [--config=<path>] [--visualize]" << std::endl;
  std::cerr << std::endl;
  std::cerr << "Examples:" << std::endl;
  std::cerr << "  # WebSocket online mode (scene from frontend)" << std::endl;
  std::cerr << "  " << prog << " ws://127.0.0.1:8080/ws demo" << std::endl;
  std::cerr << "  " << prog << " ws://127.0.0.1:8080/ws demo --config=config/default.json" << std::endl;
  std::cerr << std::endl;
  std::cerr << "  # Local simulation mode (scene from JSON file)" << std::endl;
  std::cerr << "  " << prog << " --local-sim --scenario=scenarios/map1.json" << std::endl;
  std::cerr << "  " << prog << " --local-sim --scenario=scenarios/map1.json --config=config/default.json" << std::endl;
  std::cerr << "  " << prog << " --local-sim --scenario=scenarios/map1.json --visualize" << std::endl;
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

// 解析命令行参数
bool parse_command_line(int argc, char* argv[], CommandLineArgs& args) {
  if (argc < 2) {
    return false;
  }

  // 检查是否是本地仿真模式
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--local-sim") {
      args.use_local_sim = true;
      break;
    }
  }

  if (args.use_local_sim) {
    // 本地仿真模式参数解析
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg.find("--scenario=") == 0) {
        args.scenario_file = arg.substr(11);
      } else if (arg.find("--config=") == 0) {
        args.config_file = arg.substr(9);
      } else if (arg == "--visualize") {
        args.enable_visualization = true;
      } else if (arg == "--local-sim") {
        // Already handled
        continue;
      } else {
        std::cerr << "Unknown argument for local sim mode: " << arg << std::endl;
        return false;
      }
    }
  } else {
    // WebSocket模式参数解析（保持兼容）
    if (argc < 3) {
      return false;
    }

    args.ws_url = argv[1];
    args.room_id = argv[2];

    // 解析可选参数
    for (int i = 3; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg.find("--config=") == 0) {
        args.config_file = arg.substr(9);
      }
    }
  }

  return args.is_valid();
}

// 本地仿真模式主函数
int run_local_simulation(const CommandLineArgs& args) {
  std::cout << "=== NavSim Local Simulation Mode ===" << std::endl;
  std::cout << "Scenario: " << args.scenario_file << std::endl;
  if (!args.config_file.empty()) {
    std::cout << "Config: " << args.config_file << std::endl;
  }
  std::cout << "Visualization: " << (args.enable_visualization ? "ENABLED (ImGui)" : "DISABLED") << std::endl;
  std::cout << "====================================" << std::endl;

  std::signal(SIGINT, navsim::signal_handler);

  // 1. 创建并初始化LocalSimulator
  auto simulator = std::make_shared<navsim::sim::LocalSimulator>();

  navsim::sim::SimulatorConfig sim_config;
  sim_config.time_step = 0.01;  // 10ms step
  sim_config.time_scale = 1.0;  // Real-time
  sim_config.enable_adaptive_stepping = false;

  if (!simulator->initialize(sim_config)) {
    std::cerr << "Failed to initialize LocalSimulator" << std::endl;
    return 1;
  }

  // 2. 加载场景
  if (!simulator->load_scenario(args.scenario_file)) {
    std::cerr << "Failed to load scenario: " << args.scenario_file << std::endl;
    return 1;
  }

  // 3. 初始化算法管理器
  navsim::AlgorithmManager::Config algo_config;
  // 使用默认配置（JpsPlanner），不要硬编码
  // algo_config.primary_planner 会使用 AlgorithmManager::Config 的默认值
  algo_config.enable_visualization = args.enable_visualization;
  algo_config.verbose_logging = true;

  // 从配置文件加载（如果提供）
  if (!args.config_file.empty()) {
    algo_config.config_file = args.config_file;
    navsim::load_config_from_file(args.config_file, algo_config);
  }

  navsim::AlgorithmManager algorithm_manager;
  if (!algorithm_manager.initialize_with_simulator(algo_config)) {
    std::cerr << "Failed to initialize AlgorithmManager" << std::endl;
    return 1;
  }

  // 4. 连接仿真器和算法管理器
  algorithm_manager.set_local_simulator(simulator);

  std::cout << "[Main] Starting local simulation..." << std::endl;

  // 5. 运行仿真循环
  // 根据是否启用可视化选择不同的线程模型
  if (args.enable_visualization) {
    // 🎨 可视化模式：主线程运行仿真循环
    // SDL2 要求窗口的创建、事件处理和渲染必须在同一个线程中
    std::cout << "[Main] Running simulation loop in main thread (visualization enabled)" << std::endl;
    std::cout << "[Main] Press Ctrl+C or close the window to stop" << std::endl;

    // 传递 g_interrupt 信号，让仿真循环能够响应 Ctrl+C
    algorithm_manager.run_simulation_loop(&navsim::g_interrupt);

    std::cout << "[Main] Local simulation ended" << std::endl;
  } else {
    // 无可视化模式：仿真循环在单独的线程中运行
    std::cout << "[Main] Running simulation loop in separate thread (no visualization)" << std::endl;
    std::cout << "[Main] Press Ctrl+C to stop" << std::endl;

    std::thread sim_thread([&algorithm_manager]() {
      algorithm_manager.run_simulation_loop();
    });

    // 6. 等待中断信号
    while (!navsim::g_interrupt.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[Main] Shutting down..." << std::endl;

    // 停止仿真循环
    algorithm_manager.stop_simulation_loop();

    // 清理
    sim_thread.join();

    std::cout << "[Main] Local simulation ended" << std::endl;
  }

  return 0;
}

// WebSocket在线模式主函数
int run_websocket_mode(const CommandLineArgs& args) {
  std::cout << "=== NavSim WebSocket Online Mode ===" << std::endl;
  std::cout << "WebSocket URL: " << args.ws_url << std::endl;
  std::cout << "Room ID: " << args.room_id << std::endl;
  if (!args.config_file.empty()) {
    std::cout << "Config: " << args.config_file << std::endl;
  }
  std::cout << "=====================================" << std::endl;

  std::signal(SIGINT, navsim::signal_handler);

  // 1. 创建并初始化LocalSimulator
  auto simulator = std::make_shared<navsim::sim::LocalSimulator>();

  navsim::sim::SimulatorConfig sim_config;
  sim_config.time_step = 0.01;  // 10ms step
  sim_config.time_scale = 1.0;  // Real-time
  sim_config.enable_adaptive_stepping = false;

  if (!simulator->initialize(sim_config)) {
    std::cerr << "Failed to initialize LocalSimulator" << std::endl;
    return 1;
  }

  // 2. 初始化算法管理器
  navsim::AlgorithmManager::Config algo_config;
  algo_config.primary_planner = "StraightLinePlanner";
  algo_config.enable_visualization = false;  // WebSocket模式不使用本地可视化
  algo_config.verbose_logging = true;

  // 从配置文件加载（如果提供）
  if (!args.config_file.empty()) {
    algo_config.config_file = args.config_file;
    navsim::load_config_from_file(args.config_file, algo_config);
  }

  navsim::AlgorithmManager algorithm_manager;
  if (!algorithm_manager.initialize_with_simulator(algo_config)) {
    std::cerr << "Failed to initialize AlgorithmManager" << std::endl;
    return 1;
  }

  // 3. 连接仿真器和算法管理器
  algorithm_manager.set_local_simulator(simulator);

  // 4. 创建并连接Bridge
  auto bridge = std::make_unique<navsim::Bridge>();
  bridge->connect(args.ws_url, args.room_id);

  if (!bridge->is_connected()) {
    std::cerr << "[Main] Failed to connect to WebSocket server" << std::endl;
    return 1;
  }

  std::cout << "[Main] WebSocket connected successfully" << std::endl;

  // 将Bridge传递给AlgorithmManager
  algorithm_manager.setBridge(bridge.get(), args.ws_url + "/" + args.room_id);

  // 5. 设置world_tick回调
  std::shared_ptr<SharedState> state = std::make_shared<SharedState>();

  bridge->start([&algorithm_manager, &bridge, state](const proto::WorldTick& world_tick) {
    // 打印接收到的场景数据（使用 cerr 确保立即输出）
    std::cerr << "\n========== CALLBACK TRIGGERED: world_tick #" << world_tick.tick_id() << " ==========" << std::endl;

    // 打印 ego 状态
    if (world_tick.has_ego()) {
      const auto& ego = world_tick.ego();
      std::cout << "Ego pose: ("
                << ego.pose().x() << ", "
                << ego.pose().y() << ", "
                << ego.pose().yaw() << ")" << std::endl;
      std::cout << "Ego twist: vx=" << ego.twist().vx()
                << ", vy=" << ego.twist().vy()
                << ", omega=" << ego.twist().omega() << std::endl;
    }

    // 打印 goal
    if (world_tick.has_goal()) {
      const auto& goal = world_tick.goal();
      std::cout << "Goal pose: ("
                << goal.pose().x() << ", "
                << goal.pose().y() << ", "
                << goal.pose().yaw() << ")" << std::endl;
    }

    // 打印静态地图信息
    if (world_tick.has_static_map()) {
      const auto& static_map = world_tick.static_map();
      int total_static = static_map.circles_size() + static_map.polygons_size();
      std::cout << "Static map: " << static_map.circles_size() << " circles, "
                << static_map.polygons_size() << " polygons (total: " << total_static << ")" << std::endl;

      // 打印前3个圆形障碍物
      for (int i = 0; i < std::min(3, static_map.circles_size()); ++i) {
        const auto& circle = static_map.circles(i);
        std::cout << "  Circle #" << i << ": center=(" << circle.x() << ", " << circle.y()
                  << "), radius=" << circle.r() << std::endl;
      }

      // 打印前3个多边形障碍物
      for (int i = 0; i < std::min(3, static_map.polygons_size()); ++i) {
        const auto& polygon = static_map.polygons(i);
        std::cout << "  Polygon #" << i << ": " << polygon.points_size() << " points" << std::endl;
      }
    }

    // 打印动态障碍物数量
    std::cout << "Dynamic obstacles: " << world_tick.dynamic_obstacles_size() << std::endl;

    // 打印前3个动态障碍物的详细信息
    for (int i = 0; i < std::min(3, world_tick.dynamic_obstacles_size()); ++i) {
      const auto& obs = world_tick.dynamic_obstacles(i);
      std::cout << "  Dynamic obstacle #" << i << ": id=" << obs.id()
                << ", pose=(" << obs.pose().x() << ", " << obs.pose().y() << ")"
                << ", twist=(" << obs.twist().vx() << ", " << obs.twist().vy() << ")" << std::endl;
    }

    // ⚠️ 临时方案：检查场景数据是否有效
    // 如果没有障碍物且起点终点都是默认值，跳过处理
    // TODO: 等 navsim-online 服务器改造完成后移除此检查
    int total_obstacles = 0;
    if (world_tick.has_static_map()) {
      total_obstacles += world_tick.static_map().circles_size() + world_tick.static_map().polygons_size();
    }
    total_obstacles += world_tick.dynamic_obstacles_size();

    bool has_valid_goal = false;
    if (world_tick.has_goal()) {
      const auto& goal = world_tick.goal().pose();
      // 检查 goal 是否不是默认值 (18, 6, 0)
      has_valid_goal = (std::abs(goal.x() - 18.0) > 0.1) || (std::abs(goal.y() - 6.0) > 0.1);
    }

    if (total_obstacles == 0 && !has_valid_goal) {
      std::cerr << "⚠️  Skipping: Scene data is not valid (no obstacles, default goal)" << std::endl;
      std::cerr << "   Please set up the scene in the frontend and click 'Start'" << std::endl;
      std::cerr << "================================================\n" << std::endl;
      return;
    }

    std::cout << "================================================\n" << std::endl;

    // 接收到world_tick后，运行算法处理
    proto::PlanUpdate plan_update;
    proto::EgoCmd ego_cmd;
    auto deadline = std::chrono::milliseconds(100);

    bool success = algorithm_manager.process(world_tick, deadline, plan_update, ego_cmd);

    // 发送plan_update到前端
    if (success && plan_update.trajectory_size() > 0) {
      double compute_ms = 50.0;  // TODO: 从统计信息中获取
      bridge->publish(plan_update, compute_ms);
      std::cerr << "[WebSocket Mode] ✅ Sent plan_update with " << plan_update.trajectory_size() << " points" << std::endl;
    } else {
      std::cerr << "[WebSocket Mode] ❌ Planning failed!" << std::endl;
    }

    std::cerr << "================================================\n" << std::endl;
  });

  std::cout << "[Main] Waiting for world_tick messages from frontend..." << std::endl;

  // 6. 等待中断信号
  while (!navsim::g_interrupt.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "[Main] Shutting down..." << std::endl;

  // 清理
  bridge->stop();

  std::cout << "[Main] WebSocket mode ended" << std::endl;
  return 0;
}

}  // namespace
}  // namespace navsim

int main(int argc, char* argv[]) {
  using namespace std::chrono_literals;

  // 解析命令行参数
  navsim::CommandLineArgs args;
  if (!navsim::parse_command_line(argc, argv, args)) {
    navsim::print_usage(argv[0]);
    return 1;
  }

  // 根据模式分发
  if (args.use_local_sim) {
    return navsim::run_local_simulation(args);
  } else {
    return navsim::run_websocket_mode(args);
  }
}

#include "core/bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

#include <ixwebsocket/IXWebSocket.h>
#include <json.hpp>

namespace navsim {

// ========== Bridge::Impl ==========
class Bridge::Impl {
 public:
  Impl() = default;
  ~Impl() = default;

  // WebSocket 客户端
  ix::WebSocket ws_;
  std::string room_id_;
  WorldTickCallback callback_;
  SimulationStateCallback sim_state_callback_;
  std::atomic<bool> connected_{false};
  std::mutex mutex_;

  // 统计信息
  std::atomic<uint64_t> ws_rx_{0};           // 接收消息数
  std::atomic<uint64_t> ws_tx_{0};           // 发送消息数
  std::atomic<uint64_t> dropped_ticks_{0};   // 丢弃的 tick 数

  // 感知调试状态
  std::atomic<bool> perception_debug_enabled_{false};

  // 仿真状态
  std::atomic<bool> simulation_running_{false};

  // 滑动窗口统计（最近 100 帧）
  std::deque<double> compute_ms_window_;
  mutable std::mutex window_mutex_;

  // 获取当前时间戳（秒）
  static double now() {
    auto t = std::chrono::system_clock::now();
    auto duration = t.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
  }

  // 计算 compute_ms 中位数（p50）
  double compute_ms_p50() const {
    std::lock_guard<std::mutex> lock(window_mutex_);
    if (compute_ms_window_.empty()) {
      return 0.0;
    }
    auto sorted = compute_ms_window_;
    std::sort(sorted.begin(), sorted.end());
    return sorted[sorted.size() / 2];
  }

  // 更新 compute_ms 窗口
  void update_compute_ms(double ms) {
    std::lock_guard<std::mutex> lock(window_mutex_);
    compute_ms_window_.push_back(ms);
    if (compute_ms_window_.size() > 100) {
      compute_ms_window_.pop_front();
    }
  }

  // JSON ↔ Protobuf 转换（后续 Phase 3 实现）
  bool json_to_world_tick(const nlohmann::json& j, proto::WorldTick* tick, double* delay_ms);
  nlohmann::json plan_to_json(const proto::PlanUpdate& plan, double compute_ms);
  nlohmann::json heartbeat_to_json(double loop_hz, double compute_ms_p50);
  nlohmann::json context_to_json(const planning::PlanningContext& context);

  // WebSocket 回调
  void on_message(const ix::WebSocketMessagePtr& msg);

  // 延迟补偿（使用标量速度 v）
  void compensate_delay(proto::WorldTick* tick, double delay_sec);
};

// ========== Bridge 实现 ==========

Bridge::Bridge() : impl_(std::make_unique<Impl>()) {}

Bridge::~Bridge() {
  stop();
}

void Bridge::connect(const std::string& url, const std::string& room_id) {
  impl_->room_id_ = room_id;

  // URL 组装：命令行传入 url 和 room_id，Bridge 拼接
  std::string full_url = url + "?room=" + room_id;
  impl_->ws_.setUrl(full_url);

  std::cout << "[Bridge] Connecting to " << full_url << std::endl;

  // 设置 WebSocket 回调
  impl_->ws_.setOnMessageCallback([this](const ix::WebSocketMessagePtr& msg) {
    impl_->on_message(msg);
  });

  // 启动 WebSocket（自动重连已内置，指数回退 0.5s → 5s）
  impl_->ws_.start();

  // 等待连接建立（最多 5 秒）
  for (int i = 0; i < 50; ++i) {
    if (impl_->connected_) {
      std::cout << "[Bridge] Connected successfully" << std::endl;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cerr << "[Bridge] WARN: Connection timeout, will retry in background" << std::endl;
}

void Bridge::start(const WorldTickCallback& on_world_tick) {
  impl_->callback_ = on_world_tick;
  std::cout << "[Bridge] Started, waiting for world_tick messages..." << std::endl;
}

void Bridge::set_simulation_state_callback(const SimulationStateCallback& callback) {
  impl_->sim_state_callback_ = callback;
}

bool Bridge::is_simulation_running() const {
  return impl_->simulation_running_.load();
}

void Bridge::publish(const proto::PlanUpdate& plan, double compute_ms) {
  // 断线时直接丢弃，不阻塞
  if (!impl_->connected_) {
    std::cerr << "[Bridge] WARN: Not connected, dropping plan" << std::endl;
    return;
  }

  // 更新 compute_ms 窗口
  impl_->update_compute_ms(compute_ms);

  // 转换为 JSON（Phase 3 实现）
  nlohmann::json j = impl_->plan_to_json(plan, compute_ms);

  // 【调试输出】打印 topic 和数据格式
  std::cout << "[DEBUG] Sending plan:" << std::endl;
  std::cout << "  Topic: " << j["topic"].get<std::string>() << std::endl;
  std::cout << "  Data keys: ";
  for (auto it = j["data"].begin(); it != j["data"].end(); ++it) {
    std::cout << it.key() << " ";
  }
  std::cout << std::endl;
  if (j["data"].contains("trajectory")) {
    std::cout << "  Trajectory points: " << j["data"]["trajectory"].size() << std::endl;
  }
  if (j["data"].contains("points")) {
    std::cout << "  ⚠️  WARNING: Using 'points' field (should be 'trajectory')" << std::endl;
  }

  // 发送
  std::string msg = j.dump();
  impl_->ws_.send(msg);
  impl_->ws_tx_++;

  std::cout << "[Bridge] Sent plan with " << plan.trajectory_size() << " points, compute_ms="
            << std::fixed << std::setprecision(1) << compute_ms << "ms" << std::endl;
}

void Bridge::send_heartbeat(double loop_hz) {
  // 断线时直接丢弃，不阻塞
  if (!impl_->connected_) {
    return;
  }

  // 转换为 JSON
  nlohmann::json j = impl_->heartbeat_to_json(loop_hz, impl_->compute_ms_p50());

  // 发送
  std::string msg = j.dump();
  impl_->ws_.send(msg);
  impl_->ws_tx_++;

  std::cout << "[Bridge] Sent heartbeat: loop_hz=" << std::fixed << std::setprecision(1)
            << loop_hz << ", compute_ms_p50=" << impl_->compute_ms_p50() << "ms" << std::endl;
}

void Bridge::send_perception_debug(const planning::PlanningContext& context) {
  // 断线时直接丢弃，不阻塞
  if (!impl_->connected_) {
    return;
  }

  // 只有在启用感知调试时才发送数据
  if (!impl_->perception_debug_enabled_) {
    return;
  }

  // 调试输出：检查PlanningContext数据
  std::cout << "[Bridge] Debug: PlanningContext data:" << std::endl;
  std::cout << "  - has occupancy_grid: " << (context.occupancy_grid ? "yes" : "no") << std::endl;
  std::cout << "  - has bev_obstacles: " << (context.bev_obstacles ? "yes" : "no") << std::endl;
  std::cout << "  - dynamic_obstacles count: " << context.dynamic_obstacles.size() << std::endl;

  if (context.occupancy_grid) {
    const auto& grid = *context.occupancy_grid;
    int occupied_count = 0;
    for (const auto& cell : grid.data) {
      if (cell > 50) occupied_count++;
    }
    std::cout << "  - occupancy_grid size: " << grid.config.width << "x" << grid.config.height
              << ", occupied cells: " << occupied_count << std::endl;
  }

  if (context.bev_obstacles) {
    const auto& bev = *context.bev_obstacles;
    std::cout << "  - BEV obstacles: circles=" << bev.circles.size()
              << ", rectangles=" << bev.rectangles.size()
              << ", polygons=" << bev.polygons.size() << std::endl;
  }

  for (size_t i = 0; i < std::min(size_t(3), context.dynamic_obstacles.size()); ++i) {
    const auto& obs = context.dynamic_obstacles[i];
    std::cout << "  - Dynamic obstacle " << i << ": id=" << obs.id
              << ", type=" << obs.type
              << ", trajectories=" << obs.predicted_trajectories.size() << std::endl;
  }

  // 转换为 JSON
  nlohmann::json j = impl_->context_to_json(context);

  // 发送
  std::string msg = j.dump();
  impl_->ws_.send(msg);
  impl_->ws_tx_++;

  std::cout << "[Bridge] Sent perception debug data (size: " << msg.length() << " bytes)" << std::endl;
}

void Bridge::set_perception_debug_enabled(bool enabled) {
  impl_->perception_debug_enabled_ = enabled;
}

bool Bridge::is_perception_debug_enabled() const {
  return impl_->perception_debug_enabled_;
}

void Bridge::stop() {
  if (impl_->ws_.getReadyState() == ix::ReadyState::Open) {
    std::cout << "[Bridge] Stopping..." << std::endl;
    impl_->ws_.stop();
    impl_->connected_ = false;
  }
}

bool Bridge::is_connected() const {
  return impl_->connected_;
}

uint64_t Bridge::get_ws_rx() const {
  return impl_->ws_rx_;
}

uint64_t Bridge::get_ws_tx() const {
  return impl_->ws_tx_;
}

uint64_t Bridge::get_dropped_ticks() const {
  return impl_->dropped_ticks_;
}

// ========== Impl 回调实现 ==========

void Bridge::Impl::on_message(const ix::WebSocketMessagePtr& msg) {
  if (msg->type == ix::WebSocketMessageType::Open) {
    std::cout << "[Bridge] WebSocket connection opened" << std::endl;
    connected_ = true;
    return;
  }

  if (msg->type == ix::WebSocketMessageType::Close) {
    std::cerr << "[Bridge] ERROR: WebSocket connection closed" << std::endl;
    connected_ = false;
    return;
  }

  if (msg->type == ix::WebSocketMessageType::Error) {
    std::cerr << "[Bridge] ERROR: WebSocket error: " << msg->errorInfo.reason << std::endl;
    connected_ = false;
    return;
  }

  if (msg->type == ix::WebSocketMessageType::Message) {
    ws_rx_++;  // 统计接收消息数

    try {
      // 解析 JSON
      auto j = nlohmann::json::parse(msg->str);
      std::string topic = j.value("topic", "");

      // 处理 world_tick（兼容带/不带前导 / 的格式）
      std::string expected_topic1 = "/room/" + room_id_ + "/world_tick";  // 服务器格式
      std::string expected_topic2 = "room/" + room_id_ + "/world_tick";   // 文档格式

      if (topic == expected_topic1 || topic == expected_topic2) {
        proto::WorldTick tick;
        double delay_ms = 0.0;

        // 转换为 Protobuf
        if (json_to_world_tick(j["data"], &tick, &delay_ms)) {
          std::cout << "[Bridge] Received world_tick #" << tick.tick_id()
                    << ", delay=" << std::fixed << std::setprecision(1) << delay_ms << "ms" << std::endl;

          // 调用回调
          if (callback_) {
            callback_(tick);
          }
        }
      }
      // 处理感知调试控制消息
      else if (topic.find("/perception/debug/control") != std::string::npos) {
        try {
          bool enable = j["data"].value("enable", false);
          perception_debug_enabled_ = enable;
          std::cout << "[Bridge] Perception debug " << (enable ? "enabled" : "disabled") << std::endl;
        } catch (const std::exception& e) {
          std::cerr << "[Bridge] Error processing perception debug control: " << e.what() << std::endl;
        }
      }
      // 🔧 新增：处理仿真控制消息
      else if (topic.find("/sim_ctrl") != std::string::npos) {
        try {
          if (j.contains("data") && j["data"].is_object()) {
            std::string command = j["data"].value("command", "");
            if (command == "start" || command == "resume") {
              simulation_running_ = true;
              std::cout << "[Bridge] ✅ Simulation STARTED - algorithm will now process ticks" << std::endl;

              // 调用仿真状态回调
              if (sim_state_callback_) {
                sim_state_callback_(true);
              }
            } else if (command == "pause") {
              simulation_running_ = false;
              std::cout << "[Bridge] ⏸️  Simulation PAUSED - algorithm will skip processing" << std::endl;

              // 调用仿真状态回调
              if (sim_state_callback_) {
                sim_state_callback_(false);
              }
            } else if (command == "reset") {
              simulation_running_ = false;
              std::cout << "[Bridge] 🔄 Simulation RESET - algorithm will skip processing" << std::endl;

              // 调用仿真状态回调
              if (sim_state_callback_) {
                sim_state_callback_(false);
              }
            }
          }
        } catch (const std::exception& e) {
          std::cerr << "[Bridge] Error processing sim_ctrl: " << e.what() << std::endl;
        }
      }
      // 忽略其他消息（heartbeat, error 等）

    } catch (const std::exception& e) {
      std::cerr << "[Bridge] ERROR: JSON parse error: " << e.what() << std::endl;
    }
  }
}

// ========== JSON ↔ Protobuf 转换 ==========

bool Bridge::Impl::json_to_world_tick(const nlohmann::json& j, proto::WorldTick* tick, double* delay_ms) {
  try {
    // 验证 schema（服务器使用 "schema" 而不是 "schema_ver"）
    if (j.contains("schema")) {
      std::string schema = j["schema"];
      if (schema != "navsim.v1") {
        std::cerr << "[Bridge] WARN: schema mismatch: " << schema
                  << " (expected navsim.v1)" << std::endl;
      }
    } else if (j.contains("schema_ver")) {
      std::string schema_ver = j["schema_ver"];
      if (schema_ver != "1.0.0") {
        std::cerr << "[Bridge] WARN: schema_ver mismatch: " << schema_ver
                  << " (expected 1.0.0)" << std::endl;
      }
    } else {
      std::cerr << "[Bridge] WARN: schema/schema_ver missing" << std::endl;
    }

    // 提取 tick_id 和 stamp
    if (!j.contains("tick_id") || !j.contains("stamp")) {
      std::cerr << "[Bridge] ERROR: tick_id or stamp missing" << std::endl;
      return false;
    }
    tick->set_tick_id(j["tick_id"]);
    tick->set_stamp(j["stamp"]);

    // 解析 ego
    if (j.contains("ego")) {
      const auto& ego_json = j["ego"];
      auto* ego = tick->mutable_ego();

      // ego.pose（服务器格式：{pose: {x, y, yaw}, twist: {vx, vy, omega}}）
      if (ego_json.contains("pose")) {
        const auto& pose_json = ego_json["pose"];
        auto* ego_pose = ego->mutable_pose();
        ego_pose->set_x(pose_json.value("x", 0.0));
        ego_pose->set_y(pose_json.value("y", 0.0));
        // 服务器使用 yaw，不是 theta
        ego_pose->set_yaw(pose_json.value("yaw", 0.0));
      }

      // ego.twist（服务器直接提供 vx, vy, omega）
      if (ego_json.contains("twist")) {
        const auto& twist_json = ego_json["twist"];
        auto* ego_twist = ego->mutable_twist();
        ego_twist->set_vx(twist_json.value("vx", 0.0));
        ego_twist->set_vy(twist_json.value("vy", 0.0));
        ego_twist->set_omega(twist_json.value("omega", 0.0));
      }
    } else {
      std::cerr << "[Bridge] WARN: ego missing" << std::endl;
    }

    // 解析 goal（服务器格式：{pose: {x, y, yaw}, tol: {pos, yaw}}）
    if (j.contains("goal")) {
      const auto& goal_json = j["goal"];
      auto* goal = tick->mutable_goal();

      // goal.pose
      if (goal_json.contains("pose")) {
        const auto& pose_json = goal_json["pose"];
        auto* goal_pose = goal->mutable_pose();
        goal_pose->set_x(pose_json.value("x", 0.0));
        goal_pose->set_y(pose_json.value("y", 0.0));
        goal_pose->set_yaw(pose_json.value("yaw", 0.0));  // 服务器使用 yaw
      }

      // goal.tol
      if (goal_json.contains("tol")) {
        const auto& tol_json = goal_json["tol"];
        auto* tol = goal->mutable_tol();
        tol->set_pos(tol_json.value("pos", 0.2));
        tol->set_yaw(tol_json.value("yaw", 0.2));
      }
    } else {
      std::cerr << "[Bridge] WARN: goal missing" << std::endl;
    }

    // 解析底盘配置
    if (j.contains("chassis")) {
      const auto& chassis_json = j["chassis"];
      auto* chassis = tick->mutable_chassis();

      chassis->set_model(chassis_json.value("model", "differential"));
      chassis->set_wheelbase(chassis_json.value("wheelbase", 0.5));
      chassis->set_track_width(chassis_json.value("track_width", 0.4));

      if (chassis_json.contains("limits")) {
        const auto& limits_json = chassis_json["limits"];
        auto* limits = chassis->mutable_limits();
        limits->set_v_max(limits_json.value("v_max", 2.0));
        limits->set_a_max(limits_json.value("a_max", 2.0));
        limits->set_omega_max(limits_json.value("omega_max", 2.0));
        limits->set_steer_max(limits_json.value("steer_max", 0.0));
      }

      if (chassis_json.contains("geometry")) {
        const auto& geom_json = chassis_json["geometry"];
        auto* geometry = chassis->mutable_geometry();
        geometry->set_body_length(geom_json.value("body_length", 0.6));
        geometry->set_body_width(geom_json.value("body_width", 0.5));
        geometry->set_body_height(geom_json.value("body_height", 0.3));
        geometry->set_wheel_radius(geom_json.value("wheel_radius", 0.08));
        geometry->set_wheel_width(geom_json.value("wheel_width", 0.05));
        geometry->set_front_overhang(geom_json.value("front_overhang", 0.05));
        geometry->set_rear_overhang(geom_json.value("rear_overhang", 0.05));
        geometry->set_caster_count(geom_json.value("caster_count", 2));
        geometry->set_track_width_ratio(geom_json.value("track_width_ratio", 0.0));
      }
    }

    // 解析静态地图数据
    if (j.contains("map") && j["map"].contains("static")) {
      const auto& static_json = j["map"]["static"];
      auto* static_map = tick->mutable_static_map();

      // 解析静态圆形障碍物
      if (static_json.contains("circles")) {
        for (const auto& circle_json : static_json["circles"]) {
          auto* circle = static_map->add_circles();
          circle->set_x(circle_json.value("x", 0.0));
          circle->set_y(circle_json.value("y", 0.0));
          circle->set_r(circle_json.value("r", 0.3));
        }
      }

      // 解析静态多边形障碍物
      if (static_json.contains("polygons")) {
        for (const auto& poly_json : static_json["polygons"]) {
          auto* polygon = static_map->add_polygons();
          if (poly_json.contains("points")) {
            for (const auto& point_json : poly_json["points"]) {
              auto* point = polygon->add_points();
              point->set_x(point_json.value("x", 0.0));
              point->set_y(point_json.value("y", 0.0));
              point->set_yaw(point_json.value("yaw", 0.0));
            }
          }
        }
      }

      // 解析地图配置
      if (static_json.contains("origin")) {
        const auto& origin_json = static_json["origin"];
        auto* origin = static_map->mutable_origin();
        origin->set_x(origin_json.value("x", 0.0));
        origin->set_y(origin_json.value("y", 0.0));
        origin->set_yaw(0.0);  // 默认yaw为0
      }

      if (static_json.contains("resolution")) {
        static_map->set_resolution(static_json.value("resolution", 0.1));
      }
    }

    // 解析动态障碍物
    if (j.contains("dynamic")) {
      for (const auto& dyn_json : j["dynamic"]) {
        auto* dyn_obs = tick->add_dynamic_obstacles();

        // 基本信息
        dyn_obs->set_id(dyn_json.value("id", "unknown"));
        dyn_obs->set_model(dyn_json.value("model", "cv"));

        // 形状信息
        if (dyn_json.contains("shape")) {
          const auto& shape_json = dyn_json["shape"];
          auto* shape = dyn_obs->mutable_shape();

          std::string shape_type = shape_json.value("type", "circle");
          if (shape_type == "circle") {
            auto* circle = shape->mutable_circle();
            circle->set_r(shape_json.value("r", 0.3));
            // 圆形障碍物的x,y在state中设置
          } else if (shape_type == "rect" || shape_type == "box") {
            auto* rect = shape->mutable_rectangle();
            rect->set_w(shape_json.value("w", 1.0));
            rect->set_h(shape_json.value("h", 1.0));
            rect->set_yaw(shape_json.value("yaw", 0.0));
          }
        }

        // 状态信息（位置和速度）
        if (dyn_json.contains("state")) {
          const auto& state_json = dyn_json["state"];

          // 位置
          auto* pose = dyn_obs->mutable_pose();
          pose->set_x(state_json.value("x", 0.0));
          pose->set_y(state_json.value("y", 0.0));
          pose->set_yaw(state_json.value("yaw", 0.0));

          // 速度
          auto* twist = dyn_obs->mutable_twist();
          twist->set_vx(state_json.value("vx", 0.0));
          twist->set_vy(state_json.value("vy", 0.0));
          twist->set_omega(state_json.value("omega", 0.0));
        }
      }
    }

    // 计算延迟
    double current_time = now();
    double delay_sec = current_time - tick->stamp();
    *delay_ms = delay_sec * 1000.0;

    // 延迟补偿
    if (delay_sec > 0.001) {  // 大于 1ms 才补偿
      compensate_delay(tick, delay_sec);
    }

    // 延迟警告（>100ms）
    if (delay_sec > 0.1) {
      std::cerr << "[Bridge] WARN: High delay: " << *delay_ms << "ms" << std::endl;
    }

    return true;

  } catch (const std::exception& e) {
    std::cerr << "[Bridge] ERROR: json_to_world_tick failed: " << e.what() << std::endl;
    return false;
  }
}

nlohmann::json Bridge::Impl::plan_to_json(const proto::PlanUpdate& plan, double compute_ms) {
  nlohmann::json j;
  // 修改为 plan_update 以匹配前端期望（注意前导斜杠）
  j["topic"] = "/room/" + room_id_ + "/plan_update";

  // 构造 data
  nlohmann::json data;
  data["schema_ver"] = "1.0.0";
  data["tick_id"] = plan.tick_id();
  data["stamp"] = plan.stamp();
  data["n_points"] = plan.trajectory_size();
  data["compute_ms"] = compute_ms;

  // 转换 trajectory（前端期望 trajectory 字段，不是 points）
  nlohmann::json trajectory = nlohmann::json::array();
  double s = 0.0;  // 累积弧长

  for (int i = 0; i < plan.trajectory_size(); ++i) {
    const auto& pt = plan.trajectory(i);

    nlohmann::json point;
    point["x"] = pt.x();
    point["y"] = pt.y();
    point["yaw"] = pt.yaw();  // 前端期望 yaw 字段
    point["t"] = pt.t();

    // 计算 s（累积弧长）
    if (i > 0) {
      const auto& prev_pt = plan.trajectory(i - 1);
      double dx = pt.x() - prev_pt.x();
      double dy = pt.y() - prev_pt.y();
      s += std::sqrt(dx * dx + dy * dy);
    }
    point["s"] = s;

    // 计算 kappa（曲率，暂时填 0.0）
    point["kappa"] = 0.0;

    // 计算 v（速度，暂时填常数 0.8 m/s）
    point["v"] = 0.8;

    trajectory.push_back(point);
  }

  // 前端期望 trajectory 字段
  data["trajectory"] = trajectory;

  // 添加 summary（占位值）
  data["summary"] = {
    {"min_dyn_dist", 1.5},
    {"max_kappa", 0.3},
    {"total_length", s}
  };

  j["data"] = data;
  return j;
}

nlohmann::json Bridge::Impl::heartbeat_to_json(double loop_hz, double compute_ms_p50) {
  nlohmann::json j;
  j["topic"] = "/room/" + room_id_ + "/control/heartbeat";
  j["data"] = {
    {"schema_ver", "1.0.0"},
    {"stamp", now()},
    {"ws_rx", ws_rx_.load()},
    {"ws_tx", ws_tx_.load()},
    {"dropped_ticks", dropped_ticks_.load()},
    {"loop_hz", loop_hz},
    {"compute_ms_p50", compute_ms_p50}
  };
  return j;
}

nlohmann::json Bridge::Impl::context_to_json(const planning::PlanningContext& context) {
  nlohmann::json j;
  j["topic"] = "/room/" + room_id_ + "/perception/debug";

  nlohmann::json data;
  data["schema_ver"] = "1.0.0";
  data["stamp"] = now();

  // 基础信息
  data["ego"] = {
    {"pose", {{"x", context.ego.pose.x}, {"y", context.ego.pose.y}, {"yaw", context.ego.pose.yaw}}},
    {"twist", {{"vx", context.ego.twist.vx}, {"vy", context.ego.twist.vy}, {"omega", context.ego.twist.omega}}},
    {"kinematics", {{"wheelbase", context.ego.kinematics.wheelbase}, {"width", context.ego.kinematics.width}}}
  };

  data["task"] = {
    {"goal_pose", {{"x", context.task.goal_pose.x}, {"y", context.task.goal_pose.y}, {"yaw", context.task.goal_pose.yaw}}},
    {"tolerance", {{"position", context.task.tolerance.position}, {"yaw", context.task.tolerance.yaw}}}
  };

  data["planning_horizon"] = context.planning_horizon;

  // 感知数据
  if (context.occupancy_grid) {
    const auto& grid = *context.occupancy_grid;

    // 直接传输完整栅格数据，不进行采样
    std::vector<std::vector<int>> grid_data;
    for (int y = 0; y < grid.config.height; y++) {
      std::vector<int> row;
      for (int x = 0; x < grid.config.width; x++) {
        int index = y * grid.config.width + x;
        if (index < static_cast<int>(grid.data.size())) {
          row.push_back(grid.data[index]);
        } else {
          row.push_back(0); // 默认值为自由空间
        }
      }
      grid_data.push_back(row);
    }

    data["occupancy_grid"] = {
      {"config", {
        {"origin", {{"x", grid.config.origin.x}, {"y", grid.config.origin.y}}},
        {"resolution", grid.config.resolution},
        {"width", grid.config.width},
        {"height", grid.config.height}
      }},
      {"grid_data", grid_data}
    };
  }

  if (context.bev_obstacles) {
    const auto& obstacles = *context.bev_obstacles;
    nlohmann::json bev_data;

    // 初始化空数组，确保前端始终能找到这些字段
    bev_data["circles"] = nlohmann::json::array();
    bev_data["rectangles"] = nlohmann::json::array();
    bev_data["polygons"] = nlohmann::json::array();

    for (const auto& circle : obstacles.circles) {
      bev_data["circles"].push_back({
        {"center", {{"x", circle.center.x}, {"y", circle.center.y}}},
        {"radius", circle.radius},
        {"confidence", circle.confidence}
      });
    }

    for (const auto& rect : obstacles.rectangles) {
      bev_data["rectangles"].push_back({
        {"pose", {{"x", rect.pose.x}, {"y", rect.pose.y}, {"yaw", rect.pose.yaw}}},
        {"width", rect.width},
        {"height", rect.height},
        {"confidence", rect.confidence}
      });
    }

    for (const auto& poly : obstacles.polygons) {
      nlohmann::json vertices;
      for (const auto& vertex : poly.vertices) {
        vertices.push_back({{"x", vertex.x}, {"y", vertex.y}});
      }
      bev_data["polygons"].push_back({
        {"vertices", vertices},
        {"confidence", poly.confidence}
      });
    }

    data["bev_obstacles"] = bev_data;
  }

  // ESDF 距离场地图
  if (context.esdf_map) {
    const auto& esdf = *context.esdf_map;

    // ✅ esdf.config.width 和 esdf.config.height 已经是格子数了！
    int grid_width = esdf.config.width;
    int grid_height = esdf.config.height;

    // 计算地图尺寸（米）
    double map_width_m = grid_width * esdf.config.resolution;
    double map_height_m = grid_height * esdf.config.resolution;

    std::cout << "[Bridge] ESDF map found: " << map_width_m << "m x " << map_height_m << "m"
              << " @ " << esdf.config.resolution << "m/cell"
              << ", grid size: " << grid_width << "x" << grid_height
              << ", data size: " << esdf.data.size() << std::endl;

    // 🔧 优化：采样 ESDF 数据以减少传输量
    // 根据格子数自动调整采样步长
    int sample_step = 1;  // 默认不采样
    if (grid_width > 400 || grid_height > 400) {
      sample_step = 5;  // 大地图（500x500）采样为 100x100
      std::cout << "[Bridge] ⚠️  Large ESDF map detected, applying 5x downsampling for network optimization" << std::endl;
    } else if (grid_width > 200 || grid_height > 200) {
      sample_step = 2;  // 中等地图（300x300）采样为 150x150
      std::cout << "[Bridge] ⚠️  Medium ESDF map detected, applying 2x downsampling for network optimization" << std::endl;
    }
    // 小地图（<200x200）不采样

    int sampled_width = (grid_width + sample_step - 1) / sample_step;
    int sampled_height = (grid_height + sample_step - 1) / sample_step;
    double sampled_resolution = esdf.config.resolution * sample_step;

    std::vector<std::vector<double>> esdf_data;
    for (int y = 0; y < grid_height; y += sample_step) {
      std::vector<double> row;
      for (int x = 0; x < grid_width; x += sample_step) {
        int index = y * grid_width + x;
        if (index < static_cast<int>(esdf.data.size())) {
          row.push_back(esdf.data[index]);
        } else {
          row.push_back(esdf.config.max_distance);
        }
      }
      esdf_data.push_back(row);
    }

    data["esdf_map"] = {
      {"config", {
        {"origin", {{"x", esdf.config.origin.x}, {"y", esdf.config.origin.y}}},
        {"resolution", sampled_resolution},  // 采样后的分辨率
        {"width", sampled_width},            // 采样后的格子数
        {"height", sampled_height},          // 采样后的格子数
        {"max_distance", esdf.config.max_distance}
      }},
      {"data", esdf_data}
    };

    if (sample_step > 1) {
      std::cout << "[Bridge] ESDF map downsampled: " << grid_width << "x" << grid_height
                << " → " << sampled_width << "x" << sampled_height
                << " (resolution: " << esdf.config.resolution << "m → " << sampled_resolution << "m)"
                << ", data reduced: " << esdf.data.size() << " → " << (sampled_width * sampled_height)
                << " (" << (100 - 100.0 * sampled_width * sampled_height / esdf.data.size()) << "% reduction)" << std::endl;
    } else {
      std::cout << "[Bridge] ESDF map sent without downsampling: " << sampled_width << "x" << sampled_height << std::endl;
    }
  } else {
    std::cout << "[Bridge] WARNING: context.esdf_map is null!" << std::endl;
  }

  // 总是输出动态障碍物数组，即使是空的
  nlohmann::json dyn_data = nlohmann::json::array();
  for (const auto& obs : context.dynamic_obstacles) {
    nlohmann::json trajectories;
    for (const auto& traj : obs.predicted_trajectories) {
      nlohmann::json poses;
      for (size_t i = 0; i < std::min(traj.poses.size(), traj.timestamps.size()); ++i) {
        poses.push_back({
          {"x", traj.poses[i].x}, {"y", traj.poses[i].y}, {"yaw", traj.poses[i].yaw},
          {"t", traj.timestamps[i]}
        });
      }
      trajectories.push_back({{"poses", poses}, {"probability", traj.probability}});
    }

    dyn_data.push_back({
      {"id", obs.id},
      {"type", obs.type},
      {"current_pose", {{"x", obs.current_pose.x}, {"y", obs.current_pose.y}, {"yaw", obs.current_pose.yaw}}},
      {"current_twist", {{"vx", obs.current_twist.vx}, {"vy", obs.current_twist.vy}, {"omega", obs.current_twist.omega}}},
      {"predicted_trajectories", trajectories}
    });
  }
  data["dynamic_obstacles"] = dyn_data;

  j["data"] = data;
  return j;
}

void Bridge::Impl::compensate_delay(proto::WorldTick* tick, double delay_sec) {
  // 预测起点前滚（使用标量速度 v）
  auto* ego_pose = tick->mutable_ego()->mutable_pose();
  auto* ego_twist = tick->mutable_ego()->mutable_twist();

  double theta = ego_pose->yaw();
  double vx = ego_twist->vx();
  double vy = ego_twist->vy();

  // 计算标量速度 v
  double v = std::sqrt(vx * vx + vy * vy);

  // 简单线性预测（后续可改为更精确的运动模型）
  ego_pose->set_x(ego_pose->x() + v * std::cos(theta) * delay_sec);
  ego_pose->set_y(ego_pose->y() + v * std::sin(theta) * delay_sec);

  // 角速度补偿（如果有）
  double omega = ego_twist->omega();
  if (std::abs(omega) > 1e-6) {
    ego_pose->set_yaw(ego_pose->yaw() + omega * delay_sec);
  }
}

}  // namespace navsim

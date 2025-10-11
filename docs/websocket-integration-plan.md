# WebSocket 通信实现方案与开发计划

> **文档版本**: v1.1
> **创建日期**: 2025-01-XX
> **最后更新**: 2025-01-XX
> **状态**: 已修订（根据现有协议调整）

---

## 📋 目录

- [零、关键修订说明](#零关键修订说明)
- [一、技术方案概述](#一技术方案概述)
- [二、详细实现方案](#二详细实现方案)
- [三、开发计划](#三开发计划)
- [四、关键技术细节](#四关键技术细节)
- [五、文件修改清单](#五文件修改清单)
- [六、风险与应对](#六风险与应对)
- [七、验收标准](#七验收标准)
- [八、时间估算](#八时间估算)

---

## 零、关键修订说明

### ⚠️ 与初版的重要差异

根据现有协议和服务端实现，以下内容已修正：

1. **Topic 命名规范**
   - ❌ 错误：`/room/demo/world_tick`
   - ✅ 正确：`room/<room_id>/world_tick`
   - 说明：不带前导 `/`，与服务端一致

2. **消息名称调整**
   - ❌ 错误：发送 `plan_update` 和 `ego_cmd`
   - ✅ 正确：只发送 `plan`，`ego_cmd` 暂不启用
   - 说明：服务端当前只订阅 `room/<id>/plan`

3. **字段映射修正**
   - `yaw` → `theta`（角度命名统一）
   - `vx, vy, omega` → `v, kappa`（速度和曲率）
   - 增加 `schema_ver: "1.0.0"` 字段

4. **连接 URL 格式**
   - ✅ 正确：`ws://host:port/ws?room=<room_id>`
   - 说明：room 参数在 URL query，不在 topic 路径中重复

5. **消息队列优化**
   - ❌ 初版：单槽覆盖
   - ✅ 修订：SPSC 队列容量 4-8，取最新，统计 dropped_ticks

6. **延迟补偿**
   - 新增：收到 world_tick 时计算 `Δ = now - tick.stamp`
   - 新增：预测起点前滚 Δ 再交给 Planner

7. **心跳机制**
   - Topic: `room/<room_id>/control/heartbeat`
   - Data: `{ws_rx, ws_tx, dropped_ticks, loop_hz, compute_ms_p50}`

---

## 一、技术方案概述

### 1.1 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                    navsim-online (Server)                    │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  FastAPI WebSocket Server (ws://127.0.0.1:8080/ws)    │ │
│  │  - 20Hz 广播 world_tick (JSON)                         │ │
│  │  - 接收 plan (JSON)                                    │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                            ▲ │
                            │ │ WebSocket (JSON)
                            │ ▼
┌─────────────────────────────────────────────────────────────┐
│                    navsim-local (Client)                     │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  Bridge (WebSocket Client + JSON↔Protobuf 转换)       │ │
│  │  - ixwebsocket 库                                      │ │
│  │  - nlohmann/json 库                                    │ │
│  └────────────────────────────────────────────────────────┘ │
│                            ▲ │                               │
│                            │ │ Protobuf                      │
│                            │ ▼                               │
│  ┌────────────────────────────────────────────────────────┐ │
│  │  Planner (规划算法)                                    │ │
│  │  - 接收 WorldTick (protobuf)                           │ │
│  │  - 输出 PlanUpdate (protobuf)                          │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 核心技术选型

| 组件 | 技术 | 说明 |
|------|------|------|
| **WebSocket 客户端** | ixwebsocket | 已在 third_party 中，成熟稳定，支持自动重连 |
| **JSON 解析** | nlohmann/json | 已在 third_party 中，header-only |
| **消息格式** | JSON ↔ Protobuf | 线上用 JSON，本地用 Protobuf |
| **线程模型** | 双线程 | WebSocket 线程 + 规划线程 |
| **消息队列** | SPSC 队列（容量 4-8） | 取最新 world_tick，统计 dropped_ticks |
| **重连策略** | 指数回退 | 0.5s → 5s，最大重试间隔 5s |

---

## 二、详细实现方案

### 2.1 消息格式映射

#### **方向1：线上 → 本地（world_tick）**

**JSON 格式**（服务器发送）：
```json
{
  "topic": "room/demo/world_tick",
  "data": {
    "schema_ver": "1.0.0",
    "tick_id": 123456,
    "stamp": 1738200000.120,
    "map_version": 12,
    "ego": {
      "x": 2.1,
      "y": 0.7,
      "theta": 1.57,
      "v": 0.8,
      "kappa": 0.0
    },
    "goal": {
      "x": 18.0,
      "y": 7.0,
      "theta": 0.0
    },
    "obstacles": [
      {
        "id": "obs-1",
        "x": 5.0,
        "y": 2.0,
        "radius": 0.5,
        "vx": 0.3,
        "vy": 0.0
      }
    ]
  }
}
```

**Protobuf 格式**（本地使用）：
```protobuf
message WorldTick {
  uint64 tick_id = 1;
  double stamp = 2;
  EgoState ego = 3;      // {pose: {x, y, theta}, twist: {v, kappa}}
  Goal goal = 4;         // {pose: {x, y, theta}, tol: {pos, yaw}}
  repeated TrajectoryPoint last_plan = 5;
}
```

**转换策略**：
- 提取 `data` 字段，验证 `schema_ver`（在 data 内）
  - 缺失或不匹配时记录 **WARN** 日志，尝试兼容解析
- **字段映射**（单位说明）：
  - `ego.theta` → `ego.pose.yaw`（Protobuf 中使用 yaw，单位：**弧度**）
  - `ego.v` → 计算 `ego.twist.vx = v * cos(theta)`（单位：**m/s**，标量速度）
  - `ego.kappa` → `ego.twist.omega = v * kappa`（单位：**曲率 1/m**）
  - `goal.theta` → `goal.pose.yaw`（单位：**弧度**）
- **延迟补偿**（使用标量速度）：
  - 计算 `Δ = now() - tick.stamp`（单位：秒）
  - 预测起点：`ego.x += v * cos(theta) * Δ`，`ego.y += v * sin(theta) * Δ`
  - 记录延迟到日志（>100ms 记录 **WARN**）
- **obstacles 字段**：先保留占位解析，后续扩展避障时不用改解码层

#### **方向2：本地 → 线上（plan）**

**Protobuf 格式**（本地生成）：
```protobuf
message PlanUpdate {
  uint64 tick_id = 1;
  double stamp = 2;
  repeated TrajectoryPoint trajectory = 3;  // {x, y, yaw, t}
}
```

**JSON 格式**（发送到服务器）：
```json
{
  "topic": "room/demo/plan",
  "data": {
    "schema_ver": "1.0.0",
    "tick_id": 123456,
    "stamp": 1738200000.175,
    "n_points": 50,
    "compute_ms": 25.3,
    "points": [
      {
        "x": 2.10,
        "y": 0.70,
        "theta": 1.57,
        "kappa": 0.0,
        "s": 0.0,
        "t": 0.0,
        "v": 0.8
      }
    ],
    "summary": {
      "min_dyn_dist": 1.5,
      "max_kappa": 0.3,
      "total_length": 10.5
    }
  }
}
```

**转换策略**：
- Topic 名称：`room/<room_id>/plan`（不是 `plan_update`）
- 添加 `schema_ver: "1.0.0"`（在 data 内）
- **字段映射**（必须包含所有字段，即便占位）：
  - `trajectory[i].yaw` → `points[i].theta`（单位：**弧度**）
  - `trajectory[i].t` → `points[i].t`（单位：秒）
  - 计算 `s`（累积弧长，单位：米）
  - 计算 `kappa`（曲率，单位：**1/m**，暂时填 0.0）
  - 计算 `v`（速度，单位：**m/s**，暂时填常数 0.8）
  - **必须字段**：`x, y, theta, kappa, s, t, v`（7 个字段）
- 添加 `n_points`、`compute_ms`
- 添加 `summary`（最小动态距离等，初期可填占位值）
- **JSON 输出精度**：使用 double，避免科学计数法（设置 `std::fixed`）
- **兜底策略**：如果规划失败，发送静止计划（v=0，至少 1 个点，包含所有必须字段）

#### **方向3：本地 → 线上（ego_cmd，暂不启用）**

**说明**：当前服务端未订阅 `ego_cmd`，暂不发送。待服务端支持后再启用。

**预留格式**（Protobuf）：
```protobuf
message EgoCmd {
  double acceleration = 1;
  double steering = 2;
}
```

**预留格式**（JSON）：
```json
{
  "topic": "room/demo/control/ego_cmd",
  "data": {
    "schema_ver": "1.0.0",
    "tick_id": 123456,
    "stamp": 1738200000.176,
    "cmd": {
      "v": 0.8,
      "a": 0.2,
      "steer": 0.08
    }
  }
}
```

**实施策略**：
- Phase 1-4 不发送 `ego_cmd`
- Phase 5 测试时，如果服务端需要，再启用
- 放在 `control/ego_cmd` 子 topic 下

### 2.2 Bridge 实现设计

#### **类结构**：

```cpp
class Bridge {
 public:
  Bridge();
  ~Bridge();
  
  // 连接到服务器
  void connect(const std::string& url, const std::string& room_id);
  
  // 启动接收（注册回调）
  void start(const WorldTickCallback& on_world_tick);
  
  // 发布规划结果
  void publish(const proto::PlanUpdate& plan, const proto::EgoCmd& cmd);
  
  // 停止并断开
  void stop();
  
  // 获取连接状态
  bool is_connected() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
```

#### **内部实现（Impl）**：

```cpp
class Bridge::Impl {
 public:
  ix::WebSocket ws_;
  std::string room_id_;
  WorldTickCallback callback_;
  std::atomic<bool> connected_{false};
  std::mutex mutex_;

  // 统计信息
  std::atomic<uint64_t> ws_rx_{0};      // 接收消息数
  std::atomic<uint64_t> ws_tx_{0};      // 发送消息数
  std::atomic<uint64_t> dropped_ticks_{0};  // 丢弃的 tick 数（队列满时覆盖）

  // 滑动窗口统计（最近 100 帧）
  std::deque<double> compute_ms_window_;
  double compute_ms_p50() const;  // 计算中位数

  // JSON ↔ Protobuf 转换
  bool json_to_world_tick(const nlohmann::json& j, proto::WorldTick* tick, double* delay_ms);
  nlohmann::json plan_to_json(const proto::PlanUpdate& plan, double compute_ms);
  nlohmann::json heartbeat_to_json(double loop_hz, double compute_ms_p50);

  // WebSocket 回调
  void on_message(const ix::WebSocketMessagePtr& msg);

  // 延迟补偿（使用标量速度 v）
  void compensate_delay(proto::WorldTick* tick, double delay_sec);
};
```

### 2.3 线程模型

```
┌─────────────────────────────────────────────────────────┐
│  Main Thread                                             │
│  - 解析命令行参数 (url, room_id)                         │
│  - 启动 Bridge (拼接 ws://host/ws?room=<id>)            │
│  - 启动 Planner Thread                                   │
│  - 等待中断信号 (Ctrl+C)                                 │
└─────────────────────────────────────────────────────────┘
                    │
                    ├──────────────────────────────────────┐
                    │                                      │
                    ▼                                      ▼
┌─────────────────────────────────┐  ┌──────────────────────────────────┐
│  WebSocket Thread (ixwebsocket) │  │  Planner Thread                  │
│  - 接收 world_tick (JSON)       │  │  - 从队列取最新 world_tick       │
│  - 解析并验证 schema_ver (WARN) │  │  - 调用 planner.solve()          │
│  - 转换为 Protobuf              │  │  - 记录 compute_ms (滑动窗口)    │
│  - 延迟补偿 (Δ = now - stamp)   │  │  - 通过 Bridge 发送 plan         │
│  - 推送到 SPSC 队列 (容量 8)    │  │  - 失败时发送静止计划            │
│  - 发送 plan (JSON)             │  │  - 每 5s 发送心跳                │
│  - 断线时丢弃 plan，不阻塞      │  │                                  │
└─────────────────────────────────┘  └──────────────────────────────────┘
                    ▲                                      │
                    │                                      │
                    └──────────────────────────────────────┘
          SPSC 队列（容量 8，入队覆盖旧元素，Planner 只消费最新）
```

**队列策略明确**：
- **容量**：8（SPSC 无锁队列）
- **入队**：队列满时覆盖最旧元素，`dropped_ticks++`
- **出队**：Planner 每回合只消费最新一条，跳过中间帧
- **统计**：`dropped_ticks` 记录被覆盖的帧数

**URL 组装规则**：
- 命令行传入：`url`（如 `ws://127.0.0.1:8080/ws`）和 `room_id`（如 `demo`）
- Bridge 拼接：`ws://host/ws?room=<room_id>`
- 避免重复：不在 topic 路径中再次包含 room_id

**心跳机制**：
- 由 Planner Thread 每 5s 发送一次
- Topic: `room/<room_id>/control/heartbeat`
- 包含：`ws_rx, ws_tx, dropped_ticks, loop_hz, compute_ms_p50`

---

## 三、开发计划

### **Phase 1：CMake 集成（30 分钟）**

**目标**：将 ixwebsocket 和 nlohmann/json 集成到构建系统

**任务**：
- [ ] 修改 `CMakeLists.txt`，添加 ixwebsocket 子目录
- [ ] 添加 nlohmann/json 头文件路径
- [ ] 链接 ixwebsocket 库到 navsim_algo
- [ ] 配置 ixwebsocket 重连参数：
  ```cmake
  # 指数回退：0.5s → 5s
  target_compile_definitions(navsim_algo PRIVATE
    IX_WS_MIN_WAIT_BETWEEN_RECONNECTION_RETRIES=500
    IX_WS_MAX_WAIT_BETWEEN_RECONNECTION_RETRIES=5000
  )
  ```
- [ ] 验证编译通过

**验收**：
```bash
cd navsim-local
cmake -B build -S .
cmake --build build
# 无编译错误，无警告
```

**注意事项**：
- 检查 OpenSSL 和 zlib 依赖是否安装
- 如果编译失败，查看 `third_party/ixwebsocket/README.md`

### **Phase 2：Bridge 基础框架 + 心跳机制（1 小时）**

**目标**：实现 Bridge 的 WebSocket 连接、基础消息收发和心跳机制

**任务**：
- [ ] 创建 `Bridge::Impl` 类
- [ ] 实现 `connect()` 方法（URL 组装规则）：
  ```cpp
  void Bridge::connect(const std::string& url, const std::string& room_id) {
    impl_->room_id_ = room_id;
    // URL 组装：命令行传入 url 和 room_id，Bridge 拼接
    impl_->ws_.setUrl(url + "?room=" + room_id);
    impl_->ws_.setOnMessageCallback([this](const auto& msg) {
      impl_->on_message(msg);
    });
    impl_->ws_.start();  // 自动重连已内置（指数回退 0.5s → 5s）
  }
  ```
- [ ] 实现 `on_message()` 回调（先打印原始 JSON）
- [ ] 实现 `stop()` 方法（断开连接）
- [ ] 添加连接状态管理（`connected_` 标志）
- [ ] **实现心跳机制**：
  - 每 5s 发送一次心跳
  - Topic: `room/<room_id>/control/heartbeat`
  - Data: `{ws_rx, ws_tx, dropped_ticks, loop_hz, compute_ms_p50}`
  - 由 Planner Thread 调用 `bridge.send_heartbeat()`
- [ ] 验证自动重连（手动断开服务器，观察重连）

**验收**：
```bash
# 终端 1：启动服务器
cd navsim-online
./run_navsim.sh

# 终端 2：启动本地客户端
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo
# 能成功连接，打印接收到的原始 JSON
# 每 5s 发送一次心跳
```

**注意事项**：
- **URL 组装规则**：命令行传入 `url` 和 `room_id`，Bridge 拼接为 `ws://host/ws?room=<room_id>`
- Topic 格式：`room/<room_id>/world_tick`（不带前导 `/`）
- 心跳在 Phase 2 实现，避免后续遗漏

### **Phase 3：JSON ↔ Protobuf 转换（1.5 小时）**

**目标**：实现消息格式转换

**任务**：
- [ ] 实现 `json_to_world_tick()`（JSON → Protobuf）
  - 验证 `schema_ver == "1.0.0"`（在 data 内）
    - 缺失或不匹配时记录 **WARN** 日志，尝试兼容解析
  - 解析 `topic`（必须是 `room/<id>/world_tick`）
  - 提取 `tick_id`, `stamp`
  - **字段映射**（单位：theta 弧度，v m/s，kappa 1/m）：
    - `ego.theta` → `ego.pose.yaw`（弧度）
    - `ego.v, ego.theta` → `ego.twist.vx = v * cos(theta)`（标量速度）
    - `ego.v, ego.kappa` → `ego.twist.omega = v * kappa`（曲率 1/m）
    - `goal.theta` → `goal.pose.yaw`（弧度）
  - **延迟补偿**（使用标量速度 v）：
    - 计算 `delay = now() - tick.stamp`（秒）
    - 调用 `compensate_delay(tick, delay)`
    - 输出 `delay_ms` 到日志（>100ms 记录 **WARN**）
  - **obstacles 字段**：先保留占位解析
  - 错误处理（字段缺失记录 **WARN**，类型错误记录 **ERROR**）

- [ ] 实现 `plan_to_json()`（Protobuf → JSON）
  - 构造 `topic = "room/<room_id>/plan"`
  - 添加 `schema_ver: "1.0.0"`（在 data 内）
  - **字段映射**（必须包含所有字段）：
    - `trajectory[i].yaw` → `points[i].theta`（弧度）
    - 计算 `s`（累积弧长，米）
    - 计算 `kappa`（曲率 1/m，暂时填 0.0）
    - 计算 `v`（速度 m/s，暂时填常数 0.8）
    - **必须字段**：`x, y, theta, kappa, s, t, v`（7 个字段）
  - 添加 `n_points`, `compute_ms`
  - 添加 `summary`（占位值）
  - **JSON 输出精度**：使用 `std::fixed`，避免科学计数法
  - **兜底**：如果 trajectory 为空，生成静止计划（v=0，至少 1 个点，包含所有必须字段）
  - **断线处理**：断线时直接丢弃 plan 并统计，不阻塞

- [ ] 实现 `heartbeat_to_json()`
  - Topic: `room/<room_id>/control/heartbeat`
  - Data: `{ws_rx, ws_tx, dropped_ticks, loop_hz, compute_ms_p50}`
  - `compute_ms_p50` 用滑动窗口统计（最近 100 帧）

**验收**：
```bash
# 先跑通 E2E，再补单元测试
# 重点验证字段映射正确性和单位一致性
```

### **Phase 4：集成到 main.cpp（1 小时）**

**目标**：替换 demo 数据生成，使用真实 WebSocket 通信

**任务**：
- [ ] 移除 `make_demo_world_tick()` 函数
- [ ] 修改 `main()` 函数：
  - 从命令行参数读取 `ws_url` 和 `room_id`
  - 调用 `bridge.connect()`
  - 启动规划线程（等待 world_tick）
  - 通过 `bridge.publish()` 发送结果
- [ ] 添加优雅退出逻辑
- [ ] 添加日志输出（连接状态、消息统计）

**验收**：
```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo
# 能接收 world_tick，发送 plan_update
```

### **Phase 5：端到端测试与优化（1 小时）**

**目标**：联调线上线下，验证性能指标

**任务**：
- [ ] 启动线上服务器
- [ ] 启动本地算法
- [ ] 在前端观察：
  - 自车是否按规划轨迹移动
  - 话题控制台是否收到 `plan_update`
  - 延迟是否 < 120ms
  
- [ ] 性能优化：
  - 添加消息时间戳对比（计算延迟）
  - 优化 JSON 解析性能
  - 添加消息丢弃策略（旧帧）
  
- [ ] 鲁棒性测试：
  - 网络断开重连
  - 服务器重启
  - 长时间运行（30 分钟）

**验收**：
- ✅ 端到端延迟 p95 ≤ 120ms
- ✅ 前端能显示本地规划的轨迹
- ✅ 长跑 30 分钟无异常

---

## 四、关键技术细节

### 4.0 单位和精度规范

**字段单位一致化**：
- **theta**：弧度（rad）
- **kappa**：曲率（1/m）
- **v**：标量速度（m/s）
- **x, y**：位置（m）
- **s**：弧长（m）
- **t**：时间（s）
- **stamp**：时间戳（秒，Unix 时间）
- **delay**：延迟（秒或毫秒，日志中注明）

**JSON 输出精度**：
```cpp
// 使用 std::fixed 避免科学计数法
nlohmann::json j;
j["x"] = ego_x;  // 自动转换为 double
j["theta"] = ego_theta;  // 弧度，保留足够精度

// 输出时设置精度
std::ostringstream oss;
oss << std::fixed << std::setprecision(6);  // 6 位小数
oss << j.dump();
```

**延迟补偿使用标量速度**：
```cpp
// 正确：使用标量速度 v
double dx = v * cos(theta) * delay;
double dy = v * sin(theta) * delay;

// 错误：不要使用 vx, vy（JSON 中没有这些字段）
```

### 4.1 消息过滤

**问题**：服务器会广播多种消息（`world_tick`, `heartbeat`, `error` 等）

**解决方案**：
```cpp
void Bridge::Impl::on_message(const ix::WebSocketMessagePtr& msg) {
  ws_rx_++;  // 统计接收消息数

  try {
    auto j = nlohmann::json::parse(msg->str);
    std::string topic = j.value("topic", "");

    // 只处理 world_tick（注意：不带前导 /）
    if (topic == "room/" + room_id_ + "/world_tick") {
      proto::WorldTick tick;
      double delay_ms = 0.0;
      if (json_to_world_tick(j["data"], &tick, &delay_ms)) {
        std::cout << "[Bridge] Received world_tick #" << tick.tick_id()
                  << ", delay=" << delay_ms << "ms" << std::endl;
        callback_(tick);
      }
    }
    // 忽略其他消息（heartbeat, error 等）
  } catch (const std::exception& e) {
    std::cerr << "[Bridge] JSON parse error: " << e.what() << std::endl;
  }
}
```

### 4.2 时间戳同步与延迟补偿

**问题1**：本地生成的 `plan` 需要携带对应的 `tick_id`

**解决方案**：
```cpp
// 在 SharedState 中保存最新的 tick_id 和 stamp
struct SharedState {
  std::optional<proto::WorldTick> latest_world;
  uint64_t latest_tick_id = 0;
  double latest_stamp = 0.0;
};

// 发送时使用
bridge.publish(plan, shared.latest_tick_id);
```

**问题2**：网络延迟导致 world_tick 到达时已过时

**解决方案**：
```cpp
void Bridge::Impl::compensate_delay(proto::WorldTick* tick, double delay_sec) {
  // 预测起点前滚
  auto* ego_pose = tick->mutable_ego()->mutable_pose();
  auto* ego_twist = tick->mutable_ego()->mutable_twist();

  double theta = ego_pose->yaw();
  double v = ego_twist->vx();  // 已转换为 vx

  // 简单线性预测（后续可改为更精确的运动模型）
  ego_pose->set_x(ego_pose->x() + v * std::cos(theta) * delay_sec);
  ego_pose->set_y(ego_pose->y() + v * std::sin(theta) * delay_sec);

  // 记录延迟到日志
  if (delay_sec > 0.1) {  // 超过 100ms 警告
    std::cerr << "[Bridge] High delay: " << (delay_sec * 1000) << "ms" << std::endl;
  }
}
```

### 4.3 错误处理与兜底策略

**日志级别区分**：
- **WARN**：schema_ver 不匹配、字段缺失、延迟 >100ms
- **ERROR**：JSON 解析失败、类型错误、WebSocket 断开

**策略**：
- **JSON 解析失败** → 记录 **ERROR** 日志，跳过该消息，不影响后续
- **WebSocket 断开** → 记录 **ERROR** 日志，自动重连（ixwebsocket 内置，指数回退 0.5s → 5s）
- **规划超时** → 发送静止计划（v=0，至少 1 个点，包含所有必须字段，带最近的 tick_id）
- **字段缺失** → 使用默认值，记录 **WARN** 日志
- **schema_ver 不匹配** → 记录 **WARN** 日志，尝试兼容解析
- **publish(plan) 断线时** → 直接丢弃并统计，不阻塞

**静止计划示例**（包含所有必须字段）：
```cpp
proto::PlanUpdate fallback_plan;
fallback_plan.set_tick_id(latest_tick_id);
fallback_plan.set_stamp(current_time());

auto* pt = fallback_plan.add_trajectory();
pt->set_x(ego_x);
pt->set_y(ego_y);
pt->set_yaw(ego_theta);
pt->set_t(0.0);

// 转换为 JSON 时必须包含所有字段
// {x, y, theta, kappa=0, s=0, t=0, v=0}

bridge.publish(fallback_plan);  // 发送静止计划
```

---

## 五、文件修改清单

| 文件 | 修改类型 | 说明 |
|------|---------|------|
| `CMakeLists.txt` | 修改 | 添加 ixwebsocket 和 json 依赖 |
| `include/bridge.hpp` | 修改 | 添加 `connect()` 方法 |
| `src/bridge.cpp` | 重写 | 实现 WebSocket 通信 |
| `src/main.cpp` | 修改 | 移除 demo 生成，使用真实通信 |
| `README.md` | 更新 | 添加使用说明 |

---

## 六、风险与应对

| 风险 | 影响 | 应对措施 |
|------|------|---------|
| ixwebsocket 编译失败 | 高 | 检查依赖（OpenSSL/zlib），提供 Dockerfile |
| JSON 字段不匹配 | 中 | 添加详细日志，容错处理 |
| 性能不达标 | 中 | 使用 WebWorker（前端），优化 JSON 解析 |
| 内存泄漏 | 低 | 使用智能指针，Valgrind 检测 |

---

## 七、验收标准（DoD）

- [ ] **编译通过**：无警告，无错误
- [ ] **连接成功**：能连接到 `ws://127.0.0.1:8080/ws?room=demo`（支持 ws:// 和 wss://）
- [ ] **接收消息**：能解析 `world_tick` 并转换为 Protobuf
- [ ] **发送消息**：能发送 `plan`（不发送 `ego_cmd`）
- [ ] **心跳机制**：每 5s 发送一次心跳，包含统计信息
- [ ] **前端显示**：轨迹在 Three.js 场景中正确渲染
- [ ] **性能达标**：p95 延迟 ≤ 120ms
- [ ] **稳定性**：长跑 30 分钟无崩溃
- [ ] **日志规范**：WARN/ERROR 级别区分清晰
- [ ] **文档完善**：README 包含运行说明和 wss:// 支持说明（ixwebsocket + OpenSSL）

---

## 八、时间估算

| 阶段 | 预计时间 | 累计时间 |
|------|---------|---------|
| Phase 1: CMake 集成 | 30 分钟 | 0.5 小时 |
| Phase 2: Bridge 框架 | 1 小时 | 1.5 小时 |
| Phase 3: 消息转换 | 1.5 小时 | 3 小时 |
| Phase 4: main.cpp 集成 | 1 小时 | 4 小时 |
| Phase 5: 测试优化 | 1 小时 | 5 小时 |
| **总计** | **5 小时** | - |

---

## 九、后续扩展（可选）

1. **支持完整 world_tick**：扩展 proto 定义，支持 `chassis`, `map`, `dynamic`
2. **多房间支持**：命令行参数指定 `room_id`
3. **录播回放**：保存 world_tick 序列到文件
4. **性能监控**：集成 Prometheus metrics
5. **Docker 部署**：提供 Dockerfile 和 docker-compose

---

## 十、待确认问题（已根据反馈修订）

### ✅ 已确认并修正的问题

1. **Topic 命名** → 已改为 `room/<room_id>/world_tick` 和 `room/<room_id>/plan`
2. **消息名称** → 只发送 `plan`，暂不发送 `ego_cmd`
3. **字段映射** → `theta` 替代 `yaw`，`v/kappa` 替代 `vx/vy/omega`
4. **连接 URL** → `ws://host:port/ws?room=<room_id>`
5. **队列容量** → SPSC 队列容量 4-8，统计 `dropped_ticks`
6. **延迟补偿** → 收到 world_tick 时计算并补偿延迟
7. **心跳机制** → Topic: `room/<room_id>/control/heartbeat`

### 🔍 仍需确认的问题

1. **是否先实现最小可用版本**？
   - 暂时忽略 `obstacles`、`map`、`chassis` 字段
   - 只支持基础的 `ego` 和 `goal`
   - 后续再扩展完整字段

2. **命令行参数格式**：
   ```bash
   ./navsim_algo ws://127.0.0.1:8080/ws demo
   # 或
   ./navsim_algo --url ws://127.0.0.1:8080/ws --room demo
   ```

3. **日志级别**：
   - 默认：INFO（连接状态、tick_id、延迟）
   - 环境变量 `NAVSIM_LOG_LEVEL=DEBUG` 启用详细日志
   - 是否需要？

4. **SPSC 队列容量**：
   - 建议 8，是否需要调整？

5. **心跳发送频率**：
   - 建议 5 秒，是否需要调整？

---

## 附录

### A. 关键修订对照表

| 项目 | 初版（错误） | 修订版（正确） | 原因 |
|------|------------|--------------|------|
| Topic 格式 | `/room/demo/world_tick` | `room/demo/world_tick` | 与服务端一致，不带前导 `/` |
| 发送消息名 | `plan_update` | `plan` | 服务端只订阅 `plan` |
| ego_cmd | 立即发送 | 暂不启用 | 服务端未订阅 |
| 字段命名 | `yaw` | `theta` | 与现有协议一致 |
| 速度表示 | `vx, vy, omega` | `v, kappa` | 简化为标量速度和曲率 |
| Schema 字段 | `schema` | `schema_ver` | 版本化管理 |
| 队列容量 | 1（单槽） | 8（SPSC） | 减少频繁覆盖 |
| 延迟补偿 | 无 | 有（前滚预测） | 提高实时性 |
| 心跳 Topic | 未定义 | `room/<id>/control/heartbeat` | 统一命名规范 |

### B. 参考资料

- [ixwebsocket 文档](https://github.com/machinezone/IXWebSocket)
- [nlohmann/json 文档](https://github.com/nlohmann/json)
- [任务书](../../任务书.md)
- [现有服务端实现](../../navsim-online/server/main.py)

### C. 相关文件

- `proto/world_tick.proto` - WorldTick 消息定义
- `proto/plan_update.proto` - PlanUpdate 消息定义
- `proto/ego_cmd.proto` - EgoCmd 消息定义
- `navsim-online/server/main.py` - 服务器实现
- `navsim-online/web/index.html` - 前端实现

### D. 测试检查清单

在 Phase 5 测试时，务必验证：

- [ ] **Topic 名称**：完全匹配（`room/demo/world_tick` vs `room/demo/plan`，不带前导 `/`）
- [ ] **schema_ver**：字段存在且为 `"1.0.0"`（在 data 内）
- [ ] **字段映射**：`theta` 正确映射（不是 `yaw`），单位为弧度
- [ ] **速度和曲率**：`v` 和 `kappa` 正确计算，单位为 m/s 和 1/m
- [ ] **plan 必须字段**：`x, y, theta, kappa, s, t, v` 全部存在
- [ ] **n_points 和 compute_ms**：字段存在且正确
- [ ] **心跳机制**：每 5s 发送一次，包含 `ws_rx, ws_tx, dropped_ticks, loop_hz, compute_ms_p50`
- [ ] **前端显示**：轨迹在 Three.js 场景中正确渲染
- [ ] **延迟**：< 120ms（p95）
- [ ] **稳定性**：长跑 30 分钟无崩溃
- [ ] **断线重连**：正常，指数回退 0.5s → 5s
- [ ] **日志规范**：WARN/ERROR 级别区分清晰
- [ ] **JSON 精度**：无科学计数法，使用 std::fixed

---

**文档结束**


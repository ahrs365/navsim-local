# Phase 1 & Phase 2 完成报告

> **完成时间**: 2025-01-XX  
> **状态**: ✅ 已完成并通过编译

---

## ✅ Phase 1: CMake 集成（已完成）

### 修改内容

#### 1. **CMakeLists.txt**

**新增内容**：
- ✅ 添加 ixwebsocket 子目录
- ✅ 配置 TLS 支持（USE_TLS=ON，支持 wss://）
- ✅ 配置 zlib 支持（USE_ZLIB=ON）
- ✅ 添加 nlohmann/json 头文件路径
- ✅ 链接 ixwebsocket 库到 navsim_algo
- ✅ 配置重连参数（指数回退 0.5s → 5s）

**关键代码**：
```cmake
# ========== ixwebsocket ==========
set(USE_TLS ON CACHE BOOL "Enable TLS support for wss://")
set(USE_ZLIB ON CACHE BOOL "Enable zlib compression")
add_subdirectory(third_party/ixwebsocket)

# ========== nlohmann/json ==========
target_include_directories(navsim_algo
    PRIVATE
      third_party/nlohmann)

# ========== 链接库 ==========
target_link_libraries(navsim_algo
    PRIVATE
      ixwebsocket)

# ========== 重连参数 ==========
target_compile_definitions(navsim_algo PRIVATE
    IX_WS_MIN_WAIT_BETWEEN_RECONNECTION_RETRIES=500
    IX_WS_MAX_WAIT_BETWEEN_RECONNECTION_RETRIES=5000)
```

### 验收结果

```bash
$ cd navsim-local
$ cmake -B build -S .
-- TLS configured to use openssl
-- OpenSSL: 3.0.13
-- Found ZLIB: /usr/lib/x86_64-linux-gnu/libz.so (found version "1.3")
-- Configuring done (13.8s)
-- Generating done (0.0s)
✅ CMake 配置成功
```

---

## ✅ Phase 2: Bridge 基础框架 + 心跳机制（已完成）

### 修改内容

#### 1. **include/bridge.hpp**

**新增接口**：
- ✅ `connect(url, room_id)` - 连接到 WebSocket 服务器
- ✅ `start(callback)` - 启动接收循环
- ✅ `publish(plan, compute_ms)` - 发送 plan 消息（不发送 ego_cmd）
- ✅ `send_heartbeat(loop_hz)` - 发送心跳消息
- ✅ `stop()` - 停止连接
- ✅ `is_connected()` - 检查连接状态
- ✅ `get_ws_rx/tx/dropped_ticks()` - 获取统计信息

**关键设计**：
- 使用 Pimpl 模式（`class Impl`）
- 移除 `PlanPublisher` 类型（不再需要）
- 移除 `EgoCmd` 参数（只发送 plan）

#### 2. **src/bridge.cpp**

**实现内容**：

##### A. Bridge::Impl 类
- ✅ WebSocket 客户端（`ix::WebSocket ws_`）
- ✅ 统计信息（`ws_rx_`, `ws_tx_`, `dropped_ticks_`）
- ✅ 滑动窗口统计（`compute_ms_window_`，最近 100 帧）
- ✅ `compute_ms_p50()` - 计算中位数
- ✅ `update_compute_ms()` - 更新窗口

##### B. connect() 方法
- ✅ URL 组装规则：`url + "?room=" + room_id`
- ✅ 设置 WebSocket 回调
- ✅ 启动 WebSocket（自动重连已内置）
- ✅ 等待连接建立（最多 5 秒）

**代码示例**：
```cpp
void Bridge::connect(const std::string& url, const std::string& room_id) {
  impl_->room_id_ = room_id;
  std::string full_url = url + "?room=" + room_id;
  impl_->ws_.setUrl(full_url);
  
  impl_->ws_.setOnMessageCallback([this](const ix::WebSocketMessagePtr& msg) {
    impl_->on_message(msg);
  });
  
  impl_->ws_.start();  // 自动重连已内置
}
```

##### C. publish() 方法
- ✅ 断线时直接丢弃，不阻塞
- ✅ 更新 compute_ms 窗口
- ✅ 转换为 JSON（Phase 3 完成）
- ✅ 发送并统计 `ws_tx_`

##### D. send_heartbeat() 方法
- ✅ 每 5s 发送一次
- ✅ Topic: `room/<room_id>/control/heartbeat`
- ✅ 包含：`ws_rx, ws_tx, dropped_ticks, loop_hz, compute_ms_p50`

**代码示例**：
```cpp
nlohmann::json Bridge::Impl::heartbeat_to_json(double loop_hz, double compute_ms_p50) {
  nlohmann::json j;
  j["topic"] = "room/" + room_id_ + "/control/heartbeat";
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
```

##### E. on_message() 回调
- ✅ 处理 Open/Close/Error 事件
- ✅ 解析 JSON 消息
- ✅ 过滤 world_tick（Topic: `room/<room_id>/world_tick`，不带前导 `/`）
- ✅ 统计 `ws_rx_`
- ✅ 错误处理（JSON 解析失败记录 ERROR）

**代码示例**：
```cpp
void Bridge::Impl::on_message(const ix::WebSocketMessagePtr& msg) {
  if (msg->type == ix::WebSocketMessageType::Open) {
    connected_ = true;
    return;
  }
  
  if (msg->type == ix::WebSocketMessageType::Message) {
    ws_rx_++;
    auto j = nlohmann::json::parse(msg->str);
    std::string topic = j.value("topic", "");
    
    if (topic == "room/" + room_id_ + "/world_tick") {
      // 转换并调用回调（Phase 3 实现）
    }
  }
}
```

#### 3. **src/main.cpp**

**修改内容**：
- ✅ 解析命令行参数（`url` 和 `room_id`）
- ✅ 调用 `bridge.connect(url, room_id)`
- ✅ 使用新的 `publish(plan, compute_ms)` API
- ✅ 每 5s 发送心跳（在 Planner 线程中）
- ✅ 移除 demo 数据生成（等待真实 world_tick）
- ✅ 添加统计信息打印

**关键代码**：
```cpp
// 命令行参数
if (argc != 3) {
  print_usage(argv[0]);
  return 1;
}
std::string ws_url = argv[1];
std::string room_id = argv[2];

// 连接
bridge.connect(ws_url, room_id);

// 发送 plan
bridge.publish(plan, ms);

// 心跳（每 5s）
if (elapsed >= 5s) {
  bridge.send_heartbeat(loop_hz);
}
```

### 验收结果

```bash
$ cd navsim-local
$ cmake --build build
[100%] Built target navsim_algo
✅ 编译成功，无警告

$ ./build/navsim_algo
Usage: ./build/navsim_algo <ws_url> <room_id>
Example: ./build/navsim_algo ws://127.0.0.1:8080/ws demo
✅ 参数解析正常
```

---

## 📋 已实现的功能

### ✅ Phase 1
- [x] CMake 集成 ixwebsocket
- [x] CMake 集成 nlohmann/json
- [x] 配置 TLS 支持（wss://）
- [x] 配置重连参数（0.5s → 5s）
- [x] 编译通过，无警告

### ✅ Phase 2
- [x] Bridge::connect() 实现
- [x] URL 组装规则正确
- [x] WebSocket 回调设置
- [x] 连接状态管理
- [x] publish() 实现（断线不阻塞）
- [x] send_heartbeat() 实现
- [x] 心跳机制（每 5s）
- [x] 统计信息（ws_rx, ws_tx, dropped_ticks）
- [x] compute_ms_p50 滑动窗口
- [x] on_message() 回调框架
- [x] 日志级别区分（WARN/ERROR）
- [x] main.cpp 命令行参数解析
- [x] main.cpp 心跳发送逻辑

---

## 🔄 占位实现（Phase 3 完成）

以下功能已预留接口，Phase 3 实现：

- [ ] `json_to_world_tick()` - JSON → Protobuf 转换
- [ ] `plan_to_json()` - Protobuf → JSON 转换（完整字段）
- [ ] `compensate_delay()` - 延迟补偿
- [ ] obstacles 字段解析

**当前占位实现**：
```cpp
bool Bridge::Impl::json_to_world_tick(...) {
  std::cerr << "[Bridge] WARN: json_to_world_tick not implemented yet" << std::endl;
  return false;
}

nlohmann::json Bridge::Impl::plan_to_json(...) {
  // 返回占位 JSON（包含 schema_ver, topic 等）
  return j;
}
```

---

## 🎯 下一步：Phase 3

Phase 3 将实现：
1. **json_to_world_tick()**：
   - 验证 `schema_ver`（在 data 内）
   - 字段映射（theta, v, kappa）
   - 延迟补偿（使用标量速度 v）
   - obstacles 字段占位解析

2. **plan_to_json()**：
   - 完整字段映射（x, y, theta, kappa, s, t, v）
   - JSON 输出精度（std::fixed）
   - 静止计划兜底

3. **compensate_delay()**：
   - 计算 Δ = now() - tick.stamp
   - 预测起点前滚

---

## 📊 编译和运行

### 编译
```bash
cd navsim-local
rm -rf build
cmake -B build -S .
cmake --build build
```

### 运行
```bash
# 连接到本地服务器
./build/navsim_algo ws://127.0.0.1:8080/ws demo

# 连接到远程服务器（wss://）
./build/navsim_algo wss://example.com/ws demo
```

---

## ✅ 验收标准（Phase 1 & 2）

- [x] **编译通过**：无警告，无错误
- [x] **连接接口**：`connect(url, room_id)` 实现
- [x] **URL 组装**：正确拼接 `ws://host/ws?room=<id>`
- [x] **心跳机制**：每 5s 发送，包含统计信息
- [x] **日志规范**：WARN/ERROR 级别区分
- [x] **断线处理**：publish() 断线时不阻塞
- [x] **统计信息**：ws_rx, ws_tx, dropped_ticks
- [x] **滑动窗口**：compute_ms_p50 计算
- [x] **命令行参数**：正确解析 url 和 room_id
- [x] **wss:// 支持**：TLS 已启用

---

**Phase 1 & 2 已全部完成！准备进入 Phase 3。** 🎉


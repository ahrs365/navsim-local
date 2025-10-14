# 仿真开始信号实现总结

## 🎯 实现目标

解决静态障碍物不显示的问题，通过实现两阶段启动机制：
1. **连接阶段**：navsim-local 连接但不执行算法
2. **运行阶段**：用户点击"开始"后才执行算法并接收静态地图

---

## ✅ 已完成的修改

### 1. navsim-online 端（Python）

#### 文件：`navsim-online/server/main.py`

**修改 1**：移除连接时发送静态地图（第 138-150 行）
```python
async def register(self, websocket: WebSocket) -> None:
    await websocket.accept()
    self.connections.add(websocket)
    self.active = True
    
    # 注意：不在连接时发送静态地图，而是等用户点击"开始"按钮
    print(f"[Room {self.room_id}] New client connected")
    # ... 其余代码
```

**修改 2**：在开始仿真时发送静态地图（第 475-487 行）
```python
if command == "resume" or command == "start":
    self.sim_running = True
    # 🔧 修复：开始仿真时，发送静态地图
    self.include_static_next_tick = True
    print(f"[Room {self.room_id}] 仿真已开始 (sim_running=True), will send static map in next tick")
```

---

### 2. navsim-local 端（C++）

#### 文件 1：`navsim-local/include/core/bridge.hpp`

**新增**：仿真状态回调类型和方法
```cpp
class Bridge {
 public:
  using SimulationStateCallback = std::function<void(bool)>;

  // 设置仿真状态回调（监听开始/暂停事件）
  void set_simulation_state_callback(const SimulationStateCallback& callback);

  // 获取当前仿真状态
  bool is_simulation_running() const;
};
```

#### 文件 2：`navsim-local/src/core/bridge.cpp`

**新增 1**：仿真状态成员变量（第 22-39 行）
```cpp
class Bridge::Impl {
 public:
  // ... 其他成员 ...
  SimulationStateCallback sim_state_callback_;
  std::atomic<bool> simulation_running_{false};
};
```

**新增 2**：仿真状态回调方法（第 122-134 行）
```cpp
void Bridge::set_simulation_state_callback(const SimulationStateCallback& callback) {
  impl_->sim_state_callback_ = callback;
}

bool Bridge::is_simulation_running() const {
  return impl_->simulation_running_.load();
}
```

**新增 3**：处理 `/sim_ctrl` 消息（第 334-373 行）
```cpp
else if (topic.find("/sim_ctrl") != std::string::npos) {
  try {
    if (j.contains("data") && j["data"].is_object()) {
      std::string command = j["data"].value("command", "");
      if (command == "start" || command == "resume") {
        simulation_running_ = true;
        std::cout << "[Bridge] ✅ Simulation STARTED" << std::endl;
        if (sim_state_callback_) {
          sim_state_callback_(true);
        }
      } else if (command == "pause") {
        simulation_running_ = false;
        std::cout << "[Bridge] ⏸️  Simulation PAUSED" << std::endl;
        if (sim_state_callback_) {
          sim_state_callback_(false);
        }
      } else if (command == "reset") {
        simulation_running_ = false;
        std::cout << "[Bridge] 🔄 Simulation RESET" << std::endl;
        if (sim_state_callback_) {
          sim_state_callback_(false);
        }
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "[Bridge] Error processing sim_ctrl: " << e.what() << std::endl;
  }
}
```

#### 文件 3：`navsim-local/include/core/algorithm_manager.hpp`

**新增 1**：仿真状态成员变量（第 133 行）
```cpp
private:
  // ... 其他成员 ...
  std::atomic<bool> simulation_started_{false};
```

**新增 2**：仿真状态访问方法（第 107-120 行）
```cpp
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
```

#### 文件 4：`navsim-local/src/core/algorithm_manager.cpp`

**修改**：在 `process()` 开始时检查仿真状态（第 125-156 行）
```cpp
bool AlgorithmManager::process(const proto::WorldTick& world_tick,
                              std::chrono::milliseconds deadline,
                              proto::PlanUpdate& plan_update,
                              proto::EgoCmd& ego_cmd) {
  stats_.total_processed++;

  // 🔧 检查仿真是否已开始
  if (!simulation_started_.load()) {
    // 仿真未开始，只更新可视化，不执行算法
    if (visualizer_) {
      visualizer_->beginFrame();
      
      viz::IVisualizer::ConnectionStatus connection_status;
      connection_status.connected = bridge_ && bridge_->is_connected();
      connection_status.label = connection_label_;
      connection_status.message = "⏸️ Waiting for simulation to start...";
      visualizer_->updateConnectionStatus(connection_status);
      visualizer_->showDebugInfo("Status", "⏸️ Waiting for START button");
      // ... 显示其他调试信息 ...
      
      visualizer_->endFrame();
    }
    
    // 返回空的 PlanUpdate（不执行算法）
    plan_update.set_tick_id(world_tick.tick_id());
    plan_update.set_stamp(world_tick.stamp());
    return false;  // 返回 false 表示未处理
  }

  // ... 正常的算法处理逻辑 ...
}
```

#### 文件 5：`navsim-local/src/core/main.cpp`

**新增**：设置仿真状态回调（第 238-247 行）
```cpp
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
```

**修改**：更新主循环提示信息（第 258-260 行）
```cpp
std::cout << "[Main] ⏸️  Waiting for simulation to start..." << std::endl;
std::cout << "[Main] Please click the 'Start' button in the Web interface" << std::endl;
```

---

## 📊 修改统计

| 组件 | 文件数 | 新增行数 | 修改行数 |
|------|--------|----------|----------|
| navsim-online | 1 | 3 | 10 |
| navsim-local | 5 | 95 | 35 |
| **总计** | **6** | **98** | **45** |

---

## 🔄 数据流

### 启动流程

```
1. 用户启动 navsim-online
   ↓
2. 用户启动 navsim-local
   ↓
3. Bridge 连接到 navsim-online
   ↓
4. AlgorithmManager 处于等待状态（simulation_started_ = false）
   ↓
5. 可视化窗口显示 "⏸️ Waiting for START button"
   ↓
6. 用户在 Web 界面放置障碍物
   ↓
7. 用户点击"开始"按钮
   ↓
8. navsim-online 发送 /sim_ctrl {"command": "start"}
   ↓
9. navsim-online 设置 include_static_next_tick = True
   ↓
10. Bridge 收到 /sim_ctrl 消息
    ↓
11. Bridge 调用 sim_state_callback_(true)
    ↓
12. AlgorithmManager 设置 simulation_started_ = true
    ↓
13. navsim-online 发送包含静态地图的 WorldTick
    ↓
14. BEVExtractor 提取静态障碍物
    ↓
15. AlgorithmManager 执行算法并可视化
    ↓
16. 静态障碍物正确显示在可视化窗口中 ✅
```

### 消息流

```
navsim-online                    navsim-local
     |                                |
     |  WebSocket 连接建立             |
     |<------------------------------>|
     |                                |
     |  WorldTick (无静态地图)          |
     |------------------------------->|
     |                                | (不执行算法，只更新可视化)
     |                                |
     |  用户点击"开始"                   |
     |                                |
     |  /sim_ctrl {"command":"start"} |
     |------------------------------->|
     |                                | Bridge 收到消息
     |                                | simulation_running_ = true
     |                                | 调用 sim_state_callback_(true)
     |                                | AlgorithmManager.simulation_started_ = true
     |                                |
     |  WorldTick (包含静态地图)         |
     |------------------------------->|
     |                                | BEVExtractor 提取障碍物
     |                                | AlgorithmManager 执行算法
     |                                | 可视化显示障碍物 ✅
     |                                |
```

---

## 🧪 测试验证

### 自动化测试脚本

- `test_simulation_start.sh` - 交互式测试脚本

### 手动测试步骤

1. **启动 navsim-online**
   ```bash
   cd navsim-online
   python3 -m server.main
   ```

2. **启动 navsim-local**
   ```bash
   cd navsim-local
   ./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json
   ```

3. **验证等待状态**
   - 可视化窗口显示 "⏸️ Waiting for START button"
   - 日志输出 "[Main] ⏸️  Waiting for simulation to start..."

4. **在 Web 界面放置障碍物**
   - 打开 `http://localhost:8080`
   - 勾选"静态圆形"
   - 点击"放置"按钮
   - 在场景中点击几个位置
   - 点击"提交地图"按钮

5. **点击"开始"按钮**
   - 在 Web 界面点击"开始"按钮

6. **验证运行状态**
   - navsim-online 日志：`仿真已开始 (sim_running=True), will send static map in next tick`
   - navsim-local 日志：`[Bridge] ✅ Simulation STARTED`
   - navsim-local 日志：`[BEVExtractor] Has static_map: 1`
   - navsim-local 日志：`[BEVExtractor] Extracted circles: X`
   - 可视化窗口显示橙色圆形（静态障碍物）✅

---

## 📝 相关文档

- `SIMULATION_START_FIX.md` - 详细的修复报告
- `STATIC_OBSTACLES_FIX.md` - 静态障碍物问题分析
- `DEBUGGING_SUMMARY.md` - 调试过程总结
- `test_simulation_start.sh` - 测试脚本

---

## 🎉 预期效果

### 修复前

- ❌ navsim-local 连接时立即收到静态地图（可能为空）
- ❌ 用户无法在算法运行前放置障碍物
- ❌ 静态障碍物不显示

### 修复后

- ✅ navsim-local 连接后等待用户点击"开始"
- ✅ 用户可以先放置障碍物，再开始仿真
- ✅ 点击"开始"后，算法收到完整的静态地图
- ✅ 静态障碍物正确显示在可视化窗口中

---

**实现完成时间**：2025-10-14  
**编译状态**：✅ 成功  
**测试状态**：⏳ 待用户验证


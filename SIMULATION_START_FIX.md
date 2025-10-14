# 仿真开始信号修复报告

## 📋 问题描述

**原始问题**：静态障碍物在可视化窗口中不显示

**根本原因**：
1. `navsim-online` 只在特定时机发送静态地图（初始化、地图更新、重置）
2. 如果 `navsim-local` 在 `navsim-online` 启动后才连接，会错过第一个包含静态地图的 tick
3. 即使在连接时发送静态地图，用户可能还没有在 Web 界面放置障碍物，导致收到空地图

**时序问题**：
```
用户操作顺序：
1. 启动 navsim-online
2. 启动 navsim-local（连接成功）
3. 在 Web 界面放置障碍物
4. 点击"开始"按钮

问题：
- 如果在步骤 2 发送静态地图 → 用户还没放置障碍物（空地图）
- 如果在步骤 4 发送静态地图 → 但算法已经在运行，可能错过
```

---

## ✅ 解决方案

### 核心思路

**两阶段启动**：
1. **连接阶段**：navsim-local 连接到 navsim-online，但**不执行算法**
2. **运行阶段**：用户点击"开始"按钮后，navsim-online 发送静态地图，navsim-local 开始执行算法

### 修改内容

#### 1. navsim-online 端修改

**文件**：`navsim-online/server/main.py`

**修改 1**：移除连接时发送静态地图的逻辑
```python
# 第 138-150 行
async def register(self, websocket: WebSocket) -> None:
    await websocket.accept()
    self.connections.add(websocket)
    self.active = True
    
    # 注意：不在连接时发送静态地图，而是等用户点击"开始"按钮
    # 这样可以确保用户先放置好障碍物，再开始仿真
    print(f"[Room {self.room_id}] New client connected")
    
    if not self.generator_task or self.generator_task.done():
        self.generator_task = asyncio.create_task(self._run_generator())
    if not self.broadcaster_task or self.broadcaster_task.done():
        self.broadcaster_task = asyncio.create_task(self._run_broadcaster())
```

**修改 2**：在开始仿真时发送静态地图
```python
# 第 475-487 行
elif topic.endswith("/sim_ctrl"):
    # Handle simulation control commands
    if isinstance(data, dict):
        command = data.get("command")
        if command == "resume" or command == "start":
            self.sim_running = True
            # 🔧 修复：开始仿真时，发送静态地图
            # 这样可以确保算法模块收到用户设置好的完整地图
            self.include_static_next_tick = True
            print(f"[Room {self.room_id}] 仿真已开始 (sim_running=True), will send static map in next tick")
        elif command == "pause":
            self.sim_running = False
            print(f"[Room {self.room_id}] 仿真已暂停 (sim_running=False)")
```

#### 2. navsim-local 端修改

**文件 1**：`navsim-local/include/core/bridge.hpp`

添加仿真状态管理：
```cpp
class Bridge {
 public:
  using WorldTickCallback = std::function<void(const proto::WorldTick&)>;
  using SimulationStateCallback = std::function<void(bool)>;  // 仿真状态回调：true=运行，false=暂停

  // ... 其他方法 ...

  // 设置仿真状态回调（监听开始/暂停事件）
  void set_simulation_state_callback(const SimulationStateCallback& callback);

  // 获取当前仿真状态
  bool is_simulation_running() const;
```

**文件 2**：`navsim-local/src/core/bridge.cpp`

添加 `/sim_ctrl` 消息处理：
```cpp
// 第 334-373 行
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
```

**文件 3**：`navsim-local/include/core/algorithm_manager.hpp`

添加仿真状态标志和访问方法：
```cpp
class AlgorithmManager {
 public:
  // ... 其他方法 ...

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

 private:
  // ... 其他成员 ...

  // 仿真状态
  std::atomic<bool> simulation_started_{false};
};
```

**文件 4**：`navsim-local/src/core/algorithm_manager.cpp`

在 `process()` 函数开始时检查仿真状态：
```cpp
// 第 125-156 行
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
      visualizer_->showDebugInfo("Tick ID", std::to_string(world_tick.tick_id()));
      {
        std::ostringstream stamp_stream;
        stamp_stream << std::fixed << std::setprecision(3) << world_tick.stamp();
        visualizer_->showDebugInfo("Stamp", stamp_stream.str());
      }
      
      // 结束可视化帧
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

**文件 5**：`navsim-local/src/core/main.cpp`

设置仿真状态回调：
```cpp
// 第 238-247 行
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

---

## 🧪 测试步骤

### 1. 启动 navsim-online

```bash
cd navsim-online
python -m server.main
```

**预期输出**：
```
[Room demo] Room created
WebSocket server started on ws://0.0.0.0:8080/ws
```

### 2. 启动 navsim-local

```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json
```

**预期输出**：
```
[Bridge] Connecting to ws://127.0.0.1:8080/ws?room=demo
[Bridge] WebSocket connection opened
[Main] ⏸️  Waiting for simulation to start...
[Main] Please click the 'Start' button in the Web interface
```

**可视化窗口**：
- 状态栏显示：`⏸️ Waiting for START button`
- 窗口保持响应，但不执行算法

### 3. 在 Web 界面放置障碍物

1. 打开浏览器访问 `http://localhost:8080`
2. 勾选"静态圆形"
3. 点击"放置"按钮
4. 在场景中点击几个位置添加障碍物
5. 点击"提交地图"按钮

**预期**：障碍物显示在 Web 界面中

### 4. 点击"开始"按钮

在 Web 界面点击"开始"按钮

**navsim-online 预期输出**：
```
[Room demo] 仿真已开始 (sim_running=True), will send static map in next tick
```

**navsim-local 预期输出**：
```
[Bridge] ✅ Simulation STARTED - algorithm will now process ticks
[Main] ✅ Simulation STARTED - algorithm will now process ticks
[BEVExtractor] Has static_map: 1
[BEVExtractor] StaticMap circles: 5
[BEVExtractor] Extracted circles: 5
[AlgorithmManager] BEV obstacles in perception_input:
[AlgorithmManager]   Circles: 5
```

**可视化窗口**：
- 状态栏显示：`✅ Processing`
- 🟠 **橙色圆形** - 静态障碍物（正确显示！）
- 🟢 **绿色圆形 + 箭头** - 自车
- 🔴 **红色圆形** - 目标点
- 🔵 **青色线条** - 规划轨迹

---

## 📊 数据流验证

### 正确的数据流

```
用户操作：
1. 启动 navsim-online
2. 启动 navsim-local（连接成功，但不执行算法）
3. 在 Web 界面放置障碍物
4. 点击"开始"按钮

数据流：
1. navsim-online 收到 /sim_ctrl {"command": "start"}
2. navsim-online 设置 include_static_next_tick = True
3. navsim-online 在下一个 tick 中包含静态地图
4. navsim-local Bridge 收到 /sim_ctrl 消息
5. Bridge 设置 simulation_running_ = true
6. Bridge 调用 sim_state_callback_
7. AlgorithmManager 设置 simulation_started_ = true
8. navsim-local 收到包含静态地图的 WorldTick
9. BEVExtractor 提取静态障碍物
10. AlgorithmManager 执行算法并可视化
```

---

## 🎯 关键改进

### 1. 时序控制

- ✅ 用户可以先放置障碍物，再开始仿真
- ✅ 算法模块只在收到"开始"信号后才执行
- ✅ 静态地图在开始时发送，确保包含用户设置的障碍物

### 2. 用户体验

- ✅ 可视化窗口在等待时保持响应（显示"⏸️ Waiting for START button"）
- ✅ 清晰的日志输出，告知用户当前状态
- ✅ 符合直觉的操作流程

### 3. 系统稳定性

- ✅ 避免空地图被缓存的问题
- ✅ 支持暂停/恢复/重置操作
- ✅ 状态同步可靠（通过 WebSocket 消息）

---

## 📝 相关文件

- `navsim-online/server/main.py` - 仿真控制逻辑
- `navsim-local/include/core/bridge.hpp` - Bridge 接口
- `navsim-local/src/core/bridge.cpp` - Bridge 实现
- `navsim-local/include/core/algorithm_manager.hpp` - AlgorithmManager 接口
- `navsim-local/src/core/algorithm_manager.cpp` - AlgorithmManager 实现
- `navsim-local/src/core/main.cpp` - 主程序

---

## 🚀 下一步

1. **测试完整流程**：按照上述测试步骤验证修复
2. **提交代码**：如果测试通过，提交所有修改
3. **更新文档**：更新用户指南，说明正确的使用流程

---

**修复完成时间**：2025-10-14
**修复状态**：✅ 已实现，待测试


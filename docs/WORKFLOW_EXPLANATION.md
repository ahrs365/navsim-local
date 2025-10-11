# NavSim 系统工作流程详解

## 📋 目录

1. [系统组成](#系统组成)
2. [工作流程](#工作流程)
3. [前端"连接 WebSocket"按钮的作用](#前端连接-websocket按钮的作用)
4. [详细时序说明](#详细时序说明)
5. [消息流向](#消息流向)
6. [常见问题](#常见问题)

---

## 系统组成

NavSim 系统由 **4 个主要组件** 组成：

### 1. 浏览器前端（navsim-online/web/index.html）

**作用**: 可视化界面，用于观察仿真和规划结果

**技术栈**:
- Three.js - 3D 场景渲染
- Chart.js - 性能图表
- WebSocket API - 实时通信

**功能**:
- 显示自车位置、目标点、障碍物
- 显示规划轨迹（绿色曲线）
- 提供交互工具（设置起点/终点、放置障碍物）
- 显示话题控制台（查看所有消息）
- 显示性能图表（速度、偏差等）

### 2. FastAPI 服务器（navsim-online/server/main.py）

**作用**: WebSocket 服务器，负责消息广播和房间管理

**端口**:
- `:8080` - WebSocket 服务（/ws）
- `:8000` - 静态文件服务（前端页面）

**功能**:
- 管理多个房间（room）
- 生成 world_tick 消息（20Hz）
- 广播消息到所有连接的客户端
- 接收并转发 plan 消息
- 处理起点/终点/地图更新

### 3. 本地算法（navsim-local/navsim_algo）

**作用**: C++ 规划算法，接收世界状态并生成轨迹

**技术栈**:
- C++17
- ixwebsocket - WebSocket 客户端
- nlohmann/json - JSON 解析
- Protobuf - 内部数据结构

**功能**:
- 连接到 WebSocket 服务器
- 接收 world_tick 消息
- 运行规划算法
- 发送 plan 消息
- 发送心跳消息

### 4. WebSocket 连接

**作用**: 实时双向通信通道

**协议**: WebSocket (ws:// 或 wss://)

**消息格式**:
```json
{
  "topic": "room/<room_id>/world_tick",
  "data": { ... }
}
```

---

## 工作流程

### 阶段 1: 启动系统

#### 步骤 1: 启动服务器

```bash
cd navsim-online
bash run_navsim.sh
```

**发生了什么**:
1. FastAPI 服务器启动在 `:8080`
2. 静态文件服务器启动在 `:8000`
3. 服务器等待 WebSocket 连接
4. **此时还没有生成 world_tick**（因为没有客户端连接）

#### 步骤 2: 打开前端页面

```
浏览器访问: http://127.0.0.1:8000/index.html
```

**发生了什么**:
1. 浏览器发送 HTTP GET 请求到 `:8000/index.html`
2. 服务器返回 HTML/CSS/JavaScript 文件
3. 浏览器加载并渲染页面
4. Three.js 初始化 3D 场景
5. **此时还没有连接 WebSocket**（需要用户点击"连接"按钮）

#### 步骤 3: 用户点击"连接 WebSocket"按钮

**这是关键步骤！**

**发生了什么**:
1. 前端读取输入框的值：
   - WebSocket URL: `ws://127.0.0.1:8080/ws`
   - Room ID: `demo`

2. 前端创建 WebSocket 连接：
   ```javascript
   const socket = new WebSocket('ws://127.0.0.1:8080/ws?room=demo');
   ```

3. 服务器接收连接：
   - 将连接注册到房间 `demo`
   - **如果这是第一个连接，启动 world_tick 生成器**
   - **如果这是第一个连接，启动广播任务（20Hz）**

4. 前端状态更新：
   - 连接状态从"未连接"变为"已连接"（绿色）
   - 开始接收 world_tick 消息
   - 开始渲染 3D 场景

**重要**: 服务器采用"懒启动"策略，只有在有客户端连接时才开始生成和广播 world_tick。

#### 步骤 4: 启动本地算法

```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

**发生了什么**:
1. C++ 程序启动
2. 创建 WebSocket 连接到 `ws://127.0.0.1:8080/ws?room=demo`
3. 服务器将连接注册到房间 `demo`（现在房间有 2 个连接）
4. C++ 程序开始接收 world_tick 消息
5. C++ 程序开始发送 plan 消息

---

### 阶段 2: 实时通信循环（20Hz）

**这是系统的核心工作循环！**

#### 每 50ms（20Hz）发生的事情：

```
时间轴: 0ms -----> 50ms -----> 100ms -----> 150ms ...
        |          |           |            |
        tick #1    tick #2     tick #3      tick #4
```

#### 单个 tick 的完整流程：

**T = 0ms**: 服务器生成 world_tick

```python
# 服务器端（Python）
tick = {
  "schema": "navsim.v1",
  "tick_id": 12345,
  "stamp": 1727654321.123,  # Unix 时间戳
  "ego": {
    "pose": {"x": 5.2, "y": 3.1, "yaw": 0.5},
    "twist": {"vx": 2.0, "vy": 0.0, "omega": 0.1}
  },
  "goal": {
    "pose": {"x": 18.0, "y": 6.0, "yaw": 0.0},
    "tol": {"pos": 0.3, "yaw": 0.2}
  },
  "map": { ... },
  "dynamic": [ ... ]
}
```

**T = 1ms**: 服务器广播到所有客户端

```python
# 服务器端
message = {
  "topic": "/room/demo/world_tick",
  "data": tick
}
# 广播到前端和本地算法
```

**T = 2ms**: 前端接收并渲染

```javascript
// 前端（JavaScript）
socket.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  if (msg.topic === '/room/demo/world_tick') {
    // 更新自车位置
    egoMesh.position.set(msg.data.ego.pose.x, 0, msg.data.ego.pose.y);
    egoMesh.rotation.y = msg.data.ego.pose.yaw;
    
    // 更新目标点
    goalMesh.position.set(msg.data.goal.pose.x, 0, msg.data.goal.pose.y);
    
    // 渲染场景
    renderer.render(scene, camera);
  }
};
```

**T = 3ms**: 本地算法接收并处理

```cpp
// 本地算法（C++）
void on_message(const WebSocketMessagePtr& msg) {
  // 1. 解析 JSON
  auto j = nlohmann::json::parse(msg->str);
  
  // 2. 转换为 Protobuf
  proto::WorldTick tick;
  json_to_world_tick(j["data"], &tick, &delay_ms);
  
  // 3. 延迟补偿
  compensate_delay(&tick, delay_sec);
  
  // 4. 调用回调
  callback_(tick);
}
```

**T = 5ms**: 规划器运行

```cpp
// Planner 线程
proto::PlanUpdate plan;
bool ok = planner.solve(world, last_plan, deadline, &plan, &cmd);

// 设置 tick_id 和 stamp
plan.set_tick_id(world.tick_id());
plan.set_stamp(now());
```

**T = 5.1ms**: 发送 plan

```cpp
// 转换为 JSON
nlohmann::json plan_json = plan_to_json(plan, compute_ms);

// 发送到服务器
socket.send(plan_json.dump());
```

**T = 6ms**: 服务器接收并广播 plan

```python
# 服务器端
# 接收到 plan 消息
message = {
  "topic": "/room/demo/plan",
  "data": {
    "schema_ver": "1.0.0",
    "tick_id": 12345,
    "stamp": 1727654321.129,
    "n_points": 190,
    "compute_ms": 0.1,
    "points": [
      {"x": 5.2, "y": 3.1, "theta": 0.5, "kappa": 0.0, "s": 0.0, "t": 0.0, "v": 0.8},
      {"x": 5.3, "y": 3.2, "theta": 0.5, "kappa": 0.0, "s": 0.1, "t": 0.1, "v": 0.8},
      ...
    ]
  }
}

# 广播到所有客户端（包括前端和本地算法）
```

**T = 7ms**: 前端接收并渲染轨迹

```javascript
// 前端
socket.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  if (msg.topic === '/room/demo/plan') {
    // 清除旧轨迹
    scene.remove(planLine);
    
    // 创建新轨迹（绿色曲线）
    const points = msg.data.points.map(p => 
      new THREE.Vector3(p.x, 0.1, p.y)
    );
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({ color: 0x00ff00 });
    planLine = new THREE.Line(geometry, material);
    scene.add(planLine);
    
    // 渲染
    renderer.render(scene, camera);
  }
};
```

**T = 50ms**: 下一个 tick 开始

---

### 阶段 3: 心跳机制（每 5 秒）

```cpp
// 本地算法
if (elapsed >= 5s) {
  nlohmann::json heartbeat = {
    "topic": "room/demo/control/heartbeat",
    "data": {
      "schema_ver": "1.0.0",
      "stamp": now(),
      "ws_rx": 2406,        // 接收消息数
      "ws_tx": 1201,        // 发送消息数
      "dropped_ticks": 0,   // 丢弃消息数
      "loop_hz": 19.9,      // 循环频率
      "compute_ms_p50": 0.1 // 计算时间中位数
    }
  };
  
  socket.send(heartbeat.dump());
}
```

前端接收心跳并更新统计面板。

---

## 前端"连接 WebSocket"按钮的作用

### 为什么需要这个按钮？

**重要说明**: 前端的"连接"按钮**不是必需的**！即使前端不连接，只要有其他客户端（如本地算法）连接，服务器就会开始生成 world_tick。

**原因 1: 前端可视化控制**
- 前端需要连接才能**看到**仿真场景
- 前端需要连接才能**接收** world_tick 和 plan 消息
- 前端需要连接才能**发送**起点/终点/障碍物设置

**原因 2: 用户控制**
- 用户可以选择何时开始观察仿真
- 用户可以修改 URL 和 Room ID
- 用户可以断开并重新连接

**原因 3: 多房间支持**
- 不同用户可以连接到不同的房间
- 例如：`room=demo`, `room=test`, `room=production`
- 每个房间有独立的世界状态

**原因 4: 调试方便**
- 可以先打开页面，检查界面
- 然后再连接到服务器
- 可以随时断开连接

**原因 5: 可选观察**
- 本地算法可以独立运行，不需要前端
- 前端只是一个**可选的观察工具**
- 适合无头（headless）运行场景

### 点击按钮后发生了什么？

#### 1. 前端代码执行

```javascript
// 用户点击"连接"按钮
elements.connectionForm.addEventListener('submit', (event) => {
  event.preventDefault();
  
  // 读取输入框的值
  const url = elements.wsUrl.value.trim();      // "ws://127.0.0.1:8080/ws"
  const roomId = elements.roomId.value.trim();  // "demo"
  
  if (!url || !roomId) return;
  
  // 调用连接函数
  connectSocket(url, roomId);
});

function connectSocket(url, roomId) {
  // 如果已连接，先关闭旧连接
  if (state.connected && state.socket) {
    state.socket.close();
  }
  
  try {
    // 创建 WebSocket 连接
    const socket = new WebSocket(`${url}?room=${encodeURIComponent(roomId)}`);
    // 例如: ws://127.0.0.1:8080/ws?room=demo
    
    // 连接成功
    socket.onopen = () => {
      state.socket = socket;
      updateConnectionStatus(true);  // 状态变为"已连接"（绿色）
      logMessage('system', { message: `已连接 ${socket.url}` }, null, true);
    };
    
    // 连接关闭
    socket.onclose = () => {
      updateConnectionStatus(false);  // 状态变为"未连接"（红色）
      state.socket = null;
      logMessage('system', { message: '连接关闭' }, null, false);
    };
    
    // 接收消息
    socket.onmessage = (event) => {
      const parsed = JSON.parse(event.data);
      interpretMessage(parsed);  // 处理消息（world_tick, plan, heartbeat 等）
    };
    
  } catch (error) {
    logMessage('system', { message: '连接失败', error: error.message }, null, false);
  }
}
```

#### 2. 服务器端处理

```python
# FastAPI 服务器
@app.websocket("/ws")
async def websocket_endpoint(
    websocket: WebSocket, 
    room_state: RoomState = Depends(get_room)
):
    # 接受连接
    await room_state.register(websocket)
    
    try:
        while True:
            # 接收客户端消息
            message = await websocket.receive_text()
            payload = json.loads(message)
            
            # 处理消息
            topic = payload.get("topic")
            data = payload.get("data")
            
            # 验证 topic 是否属于当前房间
            if not topic.startswith(f"/room/{room_state.room_id}/"):
                await websocket.send_json({
                    "topic": f"/room/{room_state.room_id}/system/error",
                    "data": {"reason": "topic_out_of_scope"}
                })
                continue
            
            # 处理并回显消息
            await room_state.handle_client_payload(topic, data)
            await room_state.echo(topic, data)
            
    except WebSocketDisconnect:
        pass
    finally:
        # 注销连接
        await room_state.unregister(websocket)
        await room_manager.cleanup_if_empty(room_state)
```

#### 3. 房间管理器启动广播

**关键点**: **任何客户端**（前端或本地算法）连接时，都会触发生成器和广播器的启动。

```python
class RoomState:
    async def register(self, websocket: WebSocket):
        await websocket.accept()
        self.connections.add(websocket)
        self.active = True

        # 如果生成器未运行，启动它（无论是前端还是本地算法连接）
        if not self.generator_task or self.generator_task.done():
            self.generator_task = asyncio.create_task(self._run_generator())

        # 如果广播器未运行，启动它
        if not self.broadcaster_task or self.broadcaster_task.done():
            self.broadcaster_task = asyncio.create_task(self._run_broadcaster())

    async def _run_generator(self):
        """生成 world_tick（33Hz 内部频率）"""
        while True:
            # 如果没有连接，等待
            if not self.connections:
                if not self.active:
                    break
                await asyncio.sleep(0.1)
                continue

            # 有连接时，生成 world_tick
            await self._generate_tick()
            await asyncio.sleep(0.03)  # ~33 Hz

    async def _run_broadcaster(self):
        """广播 world_tick（20Hz）"""
        while True:
            # 如果没有连接，等待
            if not self.connections:
                if not self.active:
                    break
                await asyncio.sleep(0.1)
                continue

            # 有连接时，广播最新的 world_tick
            await self._broadcast_latest()
            await asyncio.sleep(0.05)  # 20 Hz
```

**重要**: 这意味着：
- ✅ 只启动本地算法，不打开前端 → 服务器正常工作
- ✅ 只打开前端并连接，不启动本地算法 → 服务器正常工作
- ✅ 两者都启动 → 服务器正常工作
- ❌ 两者都不启动 → 服务器空闲，不生成 world_tick

---

## 消息流向

### 1. world_tick 流向

```
服务器生成器 (33Hz)
    ↓
服务器广播器 (20Hz)
    ↓
    ├─→ 前端 WebSocket → Three.js 渲染
    └─→ 本地算法 WebSocket → Planner 规划
```

### 2. plan 流向

```
本地算法 Planner
    ↓
本地算法 WebSocket
    ↓
服务器接收
    ↓
服务器广播
    ↓
    ├─→ 前端 WebSocket → Three.js 渲染轨迹
    └─→ 本地算法 WebSocket（回显）
```

### 3. 用户交互流向

```
前端用户操作（设置起点/终点/障碍物）
    ↓
前端 WebSocket 发送
    ↓
服务器接收并更新世界状态
    ↓
服务器广播 world_tick（包含新的起点/终点/障碍物）
    ↓
    ├─→ 前端 WebSocket → Three.js 渲染
    └─→ 本地算法 WebSocket → Planner 规划（考虑新的起点/终点/障碍物）
```

---

## 常见问题

### Q1: 为什么前端需要点击"连接"按钮，而本地算法不需要？

**答**:
- **前端**: 是交互式应用，用户需要控制何时连接
- **本地算法**: 是命令行程序，启动时自动连接
- **关键**: 两者都是客户端，都会触发服务器启动 world_tick 生成

### Q2: 如果只启动本地算法，不打开前端，会发生什么？

**答**:
- ✅ 服务器正常生成 world_tick（因为本地算法已连接）
- ✅ 本地算法正常接收 world_tick 并发送 plan
- ❌ 无法可视化（因为没有前端渲染）
- ✅ 可以通过日志查看运行情况
- **这是您遇到的情况！**

### Q3: 如果只打开前端并连接，不启动本地算法，会发生什么？

**答**:
- ✅ 服务器正常生成 world_tick（因为前端已连接）
- ✅ 前端可以正常显示自车移动
- ❌ 没有规划轨迹（因为没有本地算法发送 plan）
- ✅ 自车会按照服务器的默认运动模型移动

### Q4: 如果两者都不启动（或都不连接），会发生什么？

**答**:
- ❌ 服务器空闲，不生成 world_tick
- ❌ 没有任何仿真运行
- ✅ 服务器等待客户端连接

### Q5: 多个前端可以同时连接到同一个房间吗？

**答**:
- ✅ 可以！所有连接到同一个房间的客户端都会接收相同的 world_tick
- ✅ 所有客户端都会看到相同的场景
- ✅ 这对于多人协作或观察很有用

### Q6: 本地算法断开后会自动重连吗？

**答**:
- ✅ 会！本地算法使用 ixwebsocket 的自动重连功能
- ✅ 重连策略：指数回退（0.5s → 5s）
- ❌ 前端需要用户手动重新点击"连接"按钮

### Q7: 前端"连接"按钮是否必需？

**答**:
- ❌ **不是必需的**！
- ✅ 只要有任何客户端（本地算法或其他前端）连接，服务器就会工作
- ✅ 前端"连接"按钮只是让**这个前端**能够观察仿真
- ✅ 适合场景：本地算法独立运行，前端可选观察

---

**文档结束**


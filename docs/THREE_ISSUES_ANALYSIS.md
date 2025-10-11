# 三个问题的详细分析和解决方案

## 📋 问题总结

用户在完整运行场景下发现了三个问题：

1. **前端页面没有看到轨迹**
2. **不知道哪个按钮是开始仿真的**
3. **自车位置更新是否正常**

---

## 🔍 问题 1: 前端页面没有看到轨迹

### 问题分析

**根本原因**: Topic 名称不匹配！

**本地算法发送的 topic**:
```cpp
// navsim-local/src/bridge.cpp
j["topic"] = "room/" + room_id_ + "/plan";
// 例如: "room/demo/plan"
```

**前端期望的 topic**:
```javascript
// navsim-online/web/index.html
} else if (topic.endsWith('/plan_update')) {
    handlePlanUpdate(data, topic);
```

**结果**: 
- ❌ 前端收到 `room/demo/plan` 消息
- ❌ 不匹配 `/plan_update` 后缀
- ❌ 消息被记录到话题控制台，但不会渲染轨迹
- ❌ 只会调用 `logMessage(topic, data, null, true)`

### 数据格式对比

**本地算法发送的格式**:
```json
{
  "topic": "room/demo/plan",
  "data": {
    "schema_ver": "1.0.0",
    "tick_id": 123,
    "stamp": 1727654321.123,
    "n_points": 190,
    "compute_ms": 0.1,
    "points": [
      {"x": 0.0, "y": 0.0, "theta": 0.0, "kappa": 0.0, "s": 0.0, "t": 0.0, "v": 0.8},
      ...
    ]
  }
}
```

**前端期望的格式**:
```javascript
// 前端期望 topic 以 /plan_update 结尾
// 前端期望 data.trajectory 数组（不是 data.points）
{
  "topic": "room/demo/plan_update",
  "data": {
    "trajectory": [
      {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0},
      ...
    ]
  }
}
```

### 解决方案

**方案 1: 修改本地算法的 topic 名称（推荐）**

修改 `navsim-local/src/bridge.cpp`:

```cpp
nlohmann::json Bridge::Impl::plan_to_json(const proto::PlanUpdate& plan, double compute_ms) {
  nlohmann::json j;
  // 修改这一行
  j["topic"] = "room/" + room_id_ + "/plan_update";  // 改为 plan_update
  
  // 修改 data 结构
  nlohmann::json data;
  data["schema_ver"] = "1.0.0";
  data["tick_id"] = plan.tick_id();
  data["stamp"] = plan.stamp();
  data["n_points"] = plan.trajectory_size();
  data["compute_ms"] = compute_ms;
  
  // 使用 trajectory 而不是 points
  nlohmann::json trajectory = nlohmann::json::array();
  
  for (int i = 0; i < plan.trajectory_size(); ++i) {
    const auto& pt = plan.trajectory(i);
    
    nlohmann::json point;
    point["x"] = pt.x();
    point["y"] = pt.y();
    point["yaw"] = pt.yaw();  // 使用 yaw 而不是 theta
    point["t"] = pt.t();
    
    trajectory.push_back(point);
  }
  
  data["trajectory"] = trajectory;  // 使用 trajectory 字段
  
  j["data"] = data;
  return j;
}
```

**方案 2: 修改前端代码（不推荐）**

修改 `navsim-online/web/index.html`:

```javascript
function interpretMessage(message) {
  // ...
  if (topic.endsWith('/plan_update') || topic.endsWith('/plan')) {
    handlePlanUpdate(data, topic);
  }
  // ...
}

function handlePlanUpdate(data, topic) {
  // 兼容两种格式
  const trajectory = data?.trajectory || data?.points;
  if (Array.isArray(trajectory) && trajectory.length) {
    const sanitized = trajectory.map((pt, idx) => ({
      x: Number(pt.x ?? 0),
      y: Number(pt.y ?? 0),
      yaw: Number(pt.yaw ?? pt.theta ?? 0),  // 兼容 yaw 和 theta
      t: Number(pt.t ?? idx * 0.1),
    }));
    updateTrajectory(sanitized);
    // ...
  }
}
```

---

## 🔍 问题 2: 哪个按钮是开始仿真的

### 答案：没有"开始仿真"按钮！

**关键理解**: 仿真是**自动运行**的，不需要手动开始。

### 仿真的运行机制

**服务器端**:
```python
# 服务器在有客户端连接时自动启动
async def _run_generator(self):
    while True:
        if not self.connections:
            await asyncio.sleep(0.1)
            continue
        
        # 自动生成 world_tick（33Hz）
        await self._generate_tick()
        await asyncio.sleep(0.03)

async def _run_broadcaster(self):
    while True:
        if not self.connections:
            await asyncio.sleep(0.1)
            continue
        
        # 自动广播 world_tick（20Hz）
        await self._broadcast_latest()
        await asyncio.sleep(0.05)
```

**前端渲染循环**:
```javascript
function renderLoop() {
    requestAnimationFrame(renderLoop);
    applyDisplayTick();           // 应用最新的 world_tick
    advanceEgoAlongTrajectory();  // 沿轨迹移动自车
    updateDynamicDraftAnimation();
    controls.update();
    renderer.render(scene, camera);
}
renderLoop();  // 自动启动
```

### 自车运动的两种模式

#### 模式 1: 服务器驱动（默认）

**当没有 plan_update 时**:
```
服务器生成 world_tick
    ↓
world_tick.ego.pose 由服务器计算
    ↓
前端接收 world_tick
    ↓
前端更新自车位置（直接使用 world_tick.ego.pose）
    ↓
自车按照服务器的运动模型移动
```

**服务器的运动模型**:
```python
# navsim-online/server/main.py
def _integrate_dynamic(self, dt: float) -> None:
    # 简单的匀速运动模型
    self.ego_pose["x"] += self.ego_twist["vx"] * dt
    self.ego_pose["y"] += self.ego_twist["vy"] * dt
    self.ego_pose["yaw"] += self.ego_twist["omega"] * dt
```

#### 模式 2: 轨迹驱动（有 plan_update 时）

**当收到 plan_update 时**:
```
前端接收 plan_update
    ↓
前端提取 trajectory
    ↓
前端启动轨迹回放
    ↓
advanceEgoAlongTrajectory() 沿轨迹移动自车
    ↓
自车按照规划轨迹移动
```

**前端的轨迹回放**:
```javascript
function handlePlanUpdate(data, topic) {
  if (Array.isArray(data?.trajectory) && data.trajectory.length) {
    const sanitized = data.trajectory.map((pt, idx) => ({
      x: Number(pt.x ?? 0),
      y: Number(pt.y ?? 0),
      yaw: Number(pt.yaw ?? 0),
      t: Number(pt.t ?? idx * 0.1),
    }));
    
    updateTrajectory(sanitized);  // 显示绿色轨迹线
    
    // 启动轨迹回放
    state.trajectoryPlayback = {
      trajectory: sanitized,
      startTime: performance.now(),
    };
    
    // 立即跳到轨迹起点
    state.egoPose = { ...sanitized[0] };
    updateEgoFromPose();
  }
}

function advanceEgoAlongTrajectory() {
  const playback = state.trajectoryPlayback;
  if (!playback) return;
  
  const now = performance.now();
  const elapsed = (now - playback.startTime) / 1000;
  
  // 根据时间 t 插值计算当前位置
  // ...
  state.egoPose = { x, y, yaw };
  updateEgoFromPose();
}
```

### 前端的控制按钮

**播放控制面板**:
```html
<section class="panel" aria-label="播放控制">
  <h2>播放控制</h2>
  <div class="inline-actions">
    <button type="button" id="playBtn">播放</button>
    <button type="button" id="pauseBtn" class="secondary">暂停</button>
  </div>
  <button type="button" id="stepBtn" class="secondary">单步</button>
</section>
```

**这些按钮的作用**:
- **播放**: 恢复 world_tick 的应用（从队列中取出并显示）
- **暂停**: 暂停 world_tick 的应用（不从队列中取出）
- **单步**: 应用队列中的下一个 world_tick

**注意**: 这些按钮**不控制仿真的运行**，只控制前端的显示！

---

## 🔍 问题 3: 自车位置更新是否正常

### 答案：取决于实现方式

### 当前的实现（有问题）

**服务器端**:
```python
# navsim-online/server/main.py
class RoomState:
    ego_pose: Dict[str, float] = field(
        default_factory=lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0}
    )
    
    def _integrate_dynamic(self, dt: float) -> None:
        # 服务器自己计算自车位置
        self.ego_pose["x"] += self.ego_twist["vx"] * dt
        self.ego_pose["y"] += self.ego_twist["vy"] * dt
        self.ego_pose["yaw"] += self.ego_twist["omega"] * dt
```

**问题**:
- ❌ 服务器不知道本地算法的规划结果
- ❌ 服务器按照自己的运动模型更新自车位置
- ❌ 本地算法接收到的起点不是上一次规划的终点

### 正确的实现方式

**方案 1: 服务器接收 ego_cmd 并更新自车状态**

**本地算法发送 ego_cmd**:
```cpp
// 在 bridge.cpp 中添加
void Bridge::publish_ego_cmd(const proto::EgoCmd& cmd) {
  nlohmann::json j;
  j["topic"] = "room/" + room_id_ + "/ego_cmd";
  
  nlohmann::json data;
  data["schema_ver"] = "1.0.0";
  data["tick_id"] = cmd.tick_id();
  data["stamp"] = cmd.stamp();
  data["v"] = cmd.v();
  data["steer"] = cmd.steer();
  data["a"] = cmd.a();
  
  j["data"] = data;
  
  ws_.send(j.dump());
}
```

**服务器接收并应用 ego_cmd**:
```python
# navsim-online/server/main.py
async def handle_client_payload(self, topic: str, data: Any) -> None:
    if topic.endswith("/ego_cmd"):
        if isinstance(data, dict):
            self.pending_ego_cmd = data
    # ...

def _apply_pending_events(self) -> None:
    if self.pending_ego_cmd:
        # 应用控制命令，更新自车状态
        cmd = self.pending_ego_cmd
        v = cmd.get("v", 0.0)
        steer = cmd.get("steer", 0.0)
        # 使用自行车模型更新
        # ...
        self.pending_ego_cmd = None
```

**方案 2: 服务器接收 plan 并跟踪执行**

**服务器接收 plan**:
```python
async def handle_client_payload(self, topic: str, data: Any) -> None:
    if topic.endswith("/plan") or topic.endswith("/plan_update"):
        if isinstance(data, dict) and "trajectory" in data:
            self.current_plan = data["trajectory"]
    # ...

def _integrate_dynamic(self, dt: float) -> None:
    if self.current_plan and len(self.current_plan) > 0:
        # 沿着 plan 移动自车
        # 根据当前时间找到对应的轨迹点
        # ...
    else:
        # 使用默认运动模型
        self.ego_pose["x"] += self.ego_twist["vx"] * dt
        # ...
```

### 当前的行为

**实际情况**:
1. 本地算法接收 world_tick #1: ego at (0, 0)
2. 本地算法规划轨迹: (0,0) → (18,6)
3. 本地算法发送 plan
4. **服务器不更新 ego_pose**（问题所在）
5. 本地算法接收 world_tick #2: ego still at (0, 0) + 微小移动
6. 本地算法再次规划: (0,0) → (18,6)
7. 循环...

**结果**: ❌ 自车位置不会按照规划轨迹更新

---

## 🛠️ 完整的解决方案

### 步骤 1: 修改本地算法的 topic 和数据格式

修改 `navsim-local/src/bridge.cpp`:

```cpp
nlohmann::json Bridge::Impl::plan_to_json(const proto::PlanUpdate& plan, double compute_ms) {
  nlohmann::json j;
  j["topic"] = "room/" + room_id_ + "/plan_update";  // 改为 plan_update
  
  nlohmann::json data;
  data["schema_ver"] = "1.0.0";
  data["tick_id"] = plan.tick_id();
  data["stamp"] = plan.stamp();
  data["n_points"] = plan.trajectory_size();
  data["compute_ms"] = compute_ms;
  
  // 使用 trajectory 字段
  nlohmann::json trajectory = nlohmann::json::array();
  
  for (int i = 0; i < plan.trajectory_size(); ++i) {
    const auto& pt = plan.trajectory(i);
    
    nlohmann::json point;
    point["x"] = pt.x();
    point["y"] = pt.y();
    point["yaw"] = pt.yaw();  // 使用 yaw
    point["t"] = pt.t();
    
    trajectory.push_back(point);
  }
  
  data["trajectory"] = trajectory;
  
  j["data"] = data;
  return j;
}
```

### 步骤 2: 重新编译

```bash
cd navsim-local
cmake --build build
```

### 步骤 3: 测试

```bash
# 终端 1
cd navsim-online && bash run_navsim.sh

# 终端 2
cd navsim-local && ./build/navsim_algo ws://127.0.0.1:8080/ws demo

# 浏览器
# 打开 http://127.0.0.1:8000/index.html
# 点击"连接 WebSocket"按钮
```

**预期结果**:
- ✅ 前端显示绿色轨迹线
- ✅ 自车沿着轨迹移动
- ✅ 话题控制台显示 `room/demo/plan_update` 消息

---

## 📝 总结

### 问题 1: 前端没有看到轨迹

**原因**: Topic 名称不匹配（`/plan` vs `/plan_update`）和数据格式不匹配（`points` vs `trajectory`）

**解决**: 修改本地算法发送 `plan_update` topic 和 `trajectory` 字段

### 问题 2: 哪个按钮开始仿真

**答案**: 没有"开始仿真"按钮，仿真自动运行

**说明**: 
- 服务器在有客户端连接时自动生成 world_tick
- 前端自动渲染接收到的 world_tick
- "播放/暂停"按钮只控制前端显示，不控制仿真

### 问题 3: 自车位置更新

**当前状态**: ❌ 服务器不会根据 plan 更新自车位置

**临时方案**: 前端会根据 plan_update 显示自车沿轨迹移动（仅前端显示）

**完整方案**: 需要修改服务器，让服务器接收并应用 plan 或 ego_cmd

---

**文档结束**


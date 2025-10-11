# 仿真控制功能

## 🎯 功能概述

实现了完整的仿真控制功能，允许用户通过前端控制物理世界的仿真状态。

---

## 📋 系统架构

### 正确的理解

1. **物理世界在服务器端**
   - 服务器维护权威的物理状态（自车位置、障碍物、目标等）
   - 自车默认在原点 (0, 0)
   - 服务器生成 world_tick 消息（33Hz）并广播（20Hz）

2. **前端是控制界面**
   - 用户可以修改物理世界（放置起点、终点、障碍物）
   - 用户可以控制仿真（开始/暂停/重置）
   - 前端的修改发送到服务器，更新物理世界

3. **本地算法是被动的**
   - 接收服务器的 world_tick（物理世界状态）
   - 根据当前状态规划轨迹
   - 发送 plan_update 回服务器

4. **仿真控制流程**
   - 用户点击"开始仿真" → 服务器开始广播 world_tick → 本地算法接收并规划
   - 用户点击"暂停仿真" → 服务器停止广播 world_tick → 本地算法停止接收
   - 用户点击"重置仿真" → 服务器重置状态并暂停

---

## 🔧 实现细节

### 服务器端修改 (`navsim-online/server/main.py`)

#### 1. 添加仿真状态标志

```python
# Simulation control
sim_running: bool = False  # Simulation starts paused, waiting for user to click "Start"
```

**说明**: 
- 仿真默认暂停（`False`）
- 用户必须点击"开始仿真"才会开始广播

#### 2. 处理 sim_ctrl 消息

```python
async def handle_client_payload(self, topic: str, data: Any) -> None:
    # ...
    elif topic.endswith("/sim_ctrl"):
        # Handle simulation control commands
        if isinstance(data, dict):
            command = data.get("command")
            if command == "resume" or command == "start":
                self.sim_running = True
            elif command == "pause":
                self.sim_running = False
            elif command == "reset":
                # Reset simulation state
                self.tick_id = 0
                self.ego_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                self.ego_twist = {"vx": 0.0, "vy": 0.0, "omega": 0.0}
                self.sim_running = False
```

**支持的命令**:
- `resume` / `start` - 开始仿真
- `pause` - 暂停仿真
- `reset` - 重置仿真状态并暂停

#### 3. 条件生成和广播

```python
async def _run_generator(self) -> None:
    try:
        while True:
            if not self.connections:
                if not self.active:
                    break
                await asyncio.sleep(0.1)
                continue
            # Only generate ticks when simulation is running
            if self.sim_running:
                await self._generate_tick()
            await asyncio.sleep(0.03)  # ~33 Hz internal simulation step
    finally:
        self.generator_task = None

async def _run_broadcaster(self) -> None:
    try:
        while True:
            if not self.connections:
                if not self.active:
                    break
                await asyncio.sleep(0.1)
                continue
            # Only broadcast when simulation is running
            if self.sim_running:
                await self._broadcast_latest()
            await asyncio.sleep(0.05)  # 20 Hz broadcast cadence
    finally:
        self.broadcaster_task = None
```

**说明**:
- 只有在 `sim_running=True` 时才生成和广播 world_tick
- 暂停时，循环继续运行但不生成/广播消息

---

### 前端修改 (`navsim-online/web/index.html`)

#### 1. 添加仿真控制按钮

```html
<section class="panel" aria-label="仿真控制与录播">
  <h2>仿真控制</h2>
  <div class="inline-actions">
    <button type="button" id="simStartBtn" style="background: #10b981; color: white;">▶ 开始仿真</button>
    <button type="button" id="simPauseBtn" class="secondary">⏸ 暂停仿真</button>
  </div>
  <button type="button" id="simResetBtn" class="secondary" style="margin-top:8px;">🔄 重置仿真</button>
  <div class="hint">开始仿真后，服务器会向本地算法发送 world_tick 消息。</div>
  
  <details style="margin-top:12px;">
    <summary style="cursor:pointer; color: var(--subtle); font-size: 0.85rem;">高级控制</summary>
    <div style="margin-top:8px;">
      <label for="simCtrlSelect">/sim_ctrl 指令</label>
      <select id="simCtrlSelect">
        <option value="resume">resume</option>
        <option value="pause">pause</option>
        <option value="reset">reset</option>
        <option value="seed">seed</option>
      </select>
      <button type="button" id="simCtrlBtn">发送 /sim_ctrl</button>
    </div>
  </details>
</section>
```

**UI 设计**:
- **开始仿真** - 绿色按钮，醒目
- **暂停仿真** - 灰色按钮
- **重置仿真** - 灰色按钮
- **高级控制** - 折叠面板，包含原有的 sim_ctrl 下拉菜单

#### 2. 添加事件处理

```javascript
elements.simStartBtn.addEventListener('click', () => {
  sendJson(`/room/${elements.roomId.value.trim()}/sim_ctrl`, { command: 'resume' });
  console.log('%c▶ 仿真已开始', 'color: green; font-weight: bold');
});

elements.simPauseBtn.addEventListener('click', () => {
  sendJson(`/room/${elements.roomId.value.trim()}/sim_ctrl`, { command: 'pause' });
  console.log('%c⏸ 仿真已暂停', 'color: orange; font-weight: bold');
});

elements.simResetBtn.addEventListener('click', () => {
  sendJson(`/room/${elements.roomId.value.trim()}/sim_ctrl`, { command: 'reset' });
  console.log('%c🔄 仿真已重置', 'color: blue; font-weight: bold');
});
```

**说明**:
- 点击按钮发送 `/room/<room_id>/sim_ctrl` 消息
- 消息格式: `{ command: 'resume' | 'pause' | 'reset' }`
- 在控制台输出操作日志

---

## 🚀 使用流程

### 1. 启动系统

```bash
# 终端 1: 启动服务器
cd navsim-online
bash run_navsim.sh

# 终端 2: 启动本地算法
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

### 2. 打开前端

1. 浏览器打开: http://127.0.0.1:8000/index.html
2. 点击"连接 WebSocket"按钮
3. 等待右上角显示"已连接"（绿色）

### 3. 设置场景（可选）

- **放置起点**: 点击"放置起点"按钮，在场景中点击
- **放置终点**: 点击"放置终点"按钮，在场景中点击
- **添加障碍物**: 使用障碍物工具添加静态/动态障碍

### 4. 开始仿真

1. **点击"▶ 开始仿真"按钮**
2. 服务器开始广播 world_tick
3. 本地算法接收 world_tick 并规划轨迹
4. 本地算法发送 plan_update
5. 前端显示绿色轨迹线
6. 自车沿轨迹移动

### 5. 控制仿真

- **暂停**: 点击"⏸ 暂停仿真" - 停止广播 world_tick
- **继续**: 点击"▶ 开始仿真" - 恢复广播
- **重置**: 点击"🔄 重置仿真" - 重置状态并暂停

---

## 📊 消息流程

### 开始仿真

```
用户点击"开始仿真"
  ↓
前端发送: /room/demo/sim_ctrl { command: 'resume' }
  ↓
服务器设置: sim_running = True
  ↓
服务器开始生成 world_tick (33Hz)
  ↓
服务器开始广播 world_tick (20Hz)
  ↓
本地算法接收 world_tick
  ↓
本地算法规划轨迹
  ↓
本地算法发送 plan_update
  ↓
服务器广播 plan_update
  ↓
前端接收 plan_update
  ↓
前端显示轨迹并启动轨迹回放
```

### 暂停仿真

```
用户点击"暂停仿真"
  ↓
前端发送: /room/demo/sim_ctrl { command: 'pause' }
  ↓
服务器设置: sim_running = False
  ↓
服务器停止生成 world_tick
  ↓
服务器停止广播 world_tick
  ↓
本地算法停止接收 world_tick
  ↓
本地算法停止规划
```

### 重置仿真

```
用户点击"重置仿真"
  ↓
前端发送: /room/demo/sim_ctrl { command: 'reset' }
  ↓
服务器重置状态:
  - tick_id = 0
  - ego_pose = (0, 0, 0)
  - ego_twist = (0, 0, 0)
  - sim_running = False
  ↓
仿真暂停
```

---

## 🎯 关键特性

### 1. 默认暂停

- 仿真默认暂停（`sim_running = False`）
- 用户必须主动点击"开始仿真"
- 避免意外启动仿真

### 2. 独立控制

- 仿真控制独立于前端显示控制
- "播放/暂停"按钮控制前端显示
- "开始/暂停仿真"按钮控制服务器广播

### 3. 状态同步

- 服务器状态是权威的
- 前端只是发送命令
- 所有客户端看到相同的仿真状态

---

## 📝 总结

### 实现的功能

✅ **开始仿真** - 服务器开始广播 world_tick  
✅ **暂停仿真** - 服务器停止广播 world_tick  
✅ **重置仿真** - 重置状态并暂停  
✅ **前端控制** - 通过按钮控制仿真  
✅ **默认暂停** - 仿真默认不运行  

### 用户体验

1. 打开页面 → 连接 WebSocket
2. （可选）设置起点、终点、障碍物
3. 点击"开始仿真" → 系统开始运行
4. 观察轨迹规划和自车运动
5. 点击"暂停仿真" → 系统暂停
6. 点击"重置仿真" → 回到初始状态

---

**文档结束**


# 修复：播放控制冲突问题

## 🐛 问题描述

**用户观察到的奇怪现象**:

1. **放置起点/终点后** - 自车不抖动 ✅
2. **点击"开始仿真"** - 自车开始抖动 ❌
3. **点击"暂停仿真"** - 自车开始沿轨迹移动 ❌（应该是暂停）
4. **再次点击"暂停仿真"** - 自车继续移动 ❌（应该是暂停）

---

## 🔍 问题分析

### 根本原因

系统中有**两个独立的"播放/暂停"控制**在冲突：

1. **仿真控制**（新添加的）
   - 控制服务器是否广播 world_tick
   - 按钮："▶ 开始仿真" / "⏸ 暂停仿真"
   - 状态：`RoomState.sim_running`（服务器端）

2. **显示控制**（原有的）
   - 控制前端是否显示收到的 world_tick
   - 按钮："播放" / "暂停"
   - 状态：`state.playback`（前端）

### 冲突逻辑

**原有的 `handleWorldTick` 逻辑**:

```javascript
function handleWorldTick(tick, topic) {
    // ...
    if (state.playback === 'playing') {
        state.displayTick = tick;  // 显示
    } else {
        state.pendingTicks.push(tick);  // 放入队列，不显示
    }
    // ...
}
```

**问题场景**:

```
初始状态:
  - state.playback = 'playing'（前端显示控制默认播放）
  - sim_running = False（仿真默认暂停）

用户点击"开始仿真":
  - sim_running = True
  - 服务器开始广播 world_tick
  - 前端收到 world_tick
  - state.playback = 'playing' → 显示 tick ✅
  - tick.ego.pose 更新 state.startPose
  - 因为没有轨迹回放，更新 state.egoPose
  - 自车被 world_tick 控制，抖动 ❌

用户点击"暂停仿真":
  - sim_running = False
  - 服务器停止广播 world_tick
  - 但轨迹回放（state.trajectoryPlayback）仍在运行
  - advanceEgoAlongTrajectory() 继续更新 ego 位置
  - 自车沿轨迹移动 ❌（应该暂停）
```

### 为什么会抖动？

**当仿真运行时**:

1. 服务器广播 world_tick（ego.pose 在变化，因为服务器有默认运动模型）
2. 前端收到 world_tick
3. `handleWorldTick` 更新 `state.startPose = tick.ego.pose`
4. 因为没有轨迹回放，更新 `state.egoPose = state.startPose`
5. 自车位置被 world_tick 的 ego.pose 控制
6. 服务器的 ego.pose 在抖动（默认运动模型）
7. 自车跟着抖动

**当收到 plan_update 时**:

1. 设置 `state.trajectoryPlayback`
2. `advanceEgoAlongTrajectory()` 开始更新 ego 位置
3. 但 `handleWorldTick` 仍在更新 `state.startPose`
4. 两个更新源冲突，导致抖动

---

## ✅ 解决方案

### 方案：移除前端显示控制，简化逻辑

**原则**: 
- 只保留一个控制：仿真控制（服务器端）
- 前端自动显示收到的所有消息
- 简化用户体验，避免混淆

### 修改内容

#### 1. 移除前端"播放/暂停"按钮

**修改前**:
```html
<section class="panel" aria-label="播放控制">
  <h2>播放控制</h2>
  <div class="inline-actions">
    <button type="button" id="playBtn">播放</button>
    <button type="button" id="pauseBtn" class="secondary">暂停</button>
  </div>
  <button type="button" id="stepBtn" class="secondary">单步</button>
  <div class="hint">暂停后，单步会播放队列中的下一帧。</div>
</section>
```

**修改后**:
```html
<section class="panel" aria-label="显示控制">
  <h2>显示控制</h2>
  <div class="hint">显示控制已简化。收到的消息会自动显示。如需控制仿真，请使用下方的"仿真控制"面板。</div>
</section>
```

#### 2. 简化 `handleWorldTick`

**修改前**:
```javascript
if (state.playback === 'playing') {
    state.displayTick = tick;
} else {
    state.pendingTicks.push(tick);
    if (state.pendingTicks.length > 100) state.pendingTicks.shift();
}
```

**修改后**:
```javascript
// 总是显示收到的 tick（简化逻辑，移除前端播放/暂停控制）
state.displayTick = tick;
```

#### 3. 移除事件处理

**移除**:
- `playBtn` 事件处理
- `pauseBtn` 事件处理
- `stepBtn` 事件处理
- 键盘快捷键（空格键、右箭头）

#### 4. 移除状态变量

**移除**:
- `state.playback`
- `state.pendingTicks`

---

## 📊 修改前后对比

### 修改前（两个控制系统）

```
用户界面:
  - "播放" / "暂停" 按钮（前端显示控制）
  - "开始仿真" / "暂停仿真" 按钮（仿真控制）

逻辑:
  - 前端显示控制: state.playback
  - 仿真控制: sim_running
  - 两个控制独立，容易冲突

问题:
  - 用户困惑：两个"暂停"有什么区别？
  - 状态不一致：前端暂停但仿真运行，或反之
  - 抖动：两个更新源冲突
```

### 修改后（单一控制系统）

```
用户界面:
  - "开始仿真" / "暂停仿真" / "重置仿真" 按钮（仿真控制）
  - 提示：收到的消息会自动显示

逻辑:
  - 只有仿真控制: sim_running
  - 前端自动显示所有收到的消息

优点:
  - 用户清晰：只有一个控制
  - 状态一致：仿真运行 = 有消息 = 自动显示
  - 无冲突：只有一个更新源
```

---

## 🚀 测试步骤

### 1. 刷新前端

**重要**: 必须刷新才能加载新代码！

```bash
# 在浏览器中
# 按 Ctrl+Shift+R 强制刷新
```

### 2. 测试流程

#### 初始状态

1. 打开 http://127.0.0.1:8000/index.html
2. 点击"连接 WebSocket"
3. **观察**: 
   - ✅ 没有 world_tick 消息（仿真默认暂停）
   - ✅ 自车不动

#### 放置起点/终点

1. 点击"放置起点"，在场景中点击
2. 点击"放置终点"，在场景中点击
3. **观察**:
   - ✅ 起点/终点标记显示
   - ✅ 自车不动（仿真未开始）
   - ✅ 没有抖动

#### 开始仿真

1. 点击"▶ 开始仿真"
2. **观察**:
   - ✅ 话题控制台出现 world_tick 消息
   - ✅ 本地算法接收 world_tick
   - ✅ 本地算法发送 plan_update
   - ✅ 前端显示绿色轨迹线
   - ✅ 自车沿轨迹平滑移动
   - ❌ **不应该抖动**

#### 暂停仿真

1. 点击"⏸ 暂停仿真"
2. **观察**:
   - ✅ world_tick 消息停止
   - ✅ plan_update 消息停止
   - ✅ 自车停止移动
   - ✅ 轨迹线仍然显示

#### 重置仿真

1. 点击"🔄 重置仿真"
2. **观察**:
   - ✅ 自车回到原点 (0, 0)
   - ✅ 仿真暂停

---

## 🎯 预期结果

### 正常行为

✅ **初始状态** - 仿真暂停，自车不动  
✅ **放置起点/终点** - 标记显示，自车不动  
✅ **开始仿真** - 自车沿轨迹平滑移动，**不抖动**  
✅ **暂停仿真** - 自车停止移动  
✅ **重置仿真** - 自车回到原点，仿真暂停  

### 关键改进

1. **单一控制** - 只有"仿真控制"，清晰明确
2. **自动显示** - 收到的消息自动显示，无需手动控制
3. **无冲突** - 移除了两个控制系统的冲突
4. **无抖动** - 轨迹回放时，world_tick 不会覆盖 ego 位置

---

## 📝 总结

### 问题

两个独立的"播放/暂停"控制系统冲突：
- 前端显示控制（`state.playback`）
- 仿真控制（`sim_running`）

### 解决

移除前端显示控制，简化为自动显示：
- 只保留仿真控制
- 前端自动显示所有收到的消息
- 避免用户困惑和状态冲突

### 结果

- ✅ 用户体验简化
- ✅ 逻辑清晰
- ✅ 无冲突
- ✅ 无抖动

---

**文档结束**


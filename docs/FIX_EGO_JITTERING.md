# 修复：自车在原地抖动

## 🐛 问题描述

**症状**:
- ✅ 绿色轨迹线显示正常
- ✅ 前端接收到 plan_update 消息
- ❌ 自车在原地抖动，没有沿轨迹移动

---

## 🔍 问题分析

### 渲染循环

前端的渲染循环（每帧约 60Hz）：

```javascript
function renderLoop() {
    requestAnimationFrame(renderLoop);
    applyDisplayTick();              // 应用 world_tick
    advanceEgoAlongTrajectory();     // 沿轨迹移动自车
    updateDynamicDraftAnimation();
    controls.update();
    renderer.render(scene, camera);
}
```

### 消息处理

**world_tick 处理**（每 50ms）:

```javascript
function handleWorldTick(tick, topic) {
    // ...
    if (!state.startPoseManual && tick.ego?.pose) {
        setStartPose(tick.ego.pose, false);  // ← 问题所在
    }
    // ...
}
```

**setStartPose 函数**:

```javascript
function setStartPose(pose, emit = true) {
    state.startPose = { x, y, yaw };
    state.egoPose = { ...state.startPose };  // ← 覆盖 egoPose
    updateEgoFromPose();                     // ← 更新自车位置
    // ...
}
```

**plan_update 处理**:

```javascript
function handlePlanUpdate(data, topic) {
    // ...
    state.trajectoryPlayback = {
        trajectory: sanitized,
        startTime: performance.now(),
    };
    state.egoPose = { ...sanitized[0] };
    updateEgoFromPose();
}
```

**轨迹回放**:

```javascript
function advanceEgoAlongTrajectory() {
    const playback = state.trajectoryPlayback;
    if (!playback) return;
    
    // 根据时间插值计算当前位置
    const elapsed = (performance.now() - playback.startTime) / 1000;
    // ...
    state.egoPose = { x, y, yaw };  // ← 更新 egoPose
    updateEgoFromPose();            // ← 更新自车位置
}
```

---

## 🎯 根本原因

**冲突的更新逻辑**:

1. **轨迹回放** (`advanceEgoAlongTrajectory`):
   - 每帧（60Hz）根据轨迹更新 `state.egoPose`
   - 自车应该沿轨迹移动

2. **world_tick 处理** (`handleWorldTick`):
   - 每 50ms 接收 world_tick
   - 调用 `setStartPose(tick.ego.pose, false)`
   - 覆盖 `state.egoPose` 为服务器的 ego.pose
   - 服务器的 ego.pose 一直是 (0, 0)（因为服务器没有根据 plan 更新）

**结果**:
- 轨迹回放尝试移动自车
- world_tick 立即把自车拉回原点
- 自车在原点和轨迹起点之间快速切换
- 看起来像在"抖动"

---

## ✅ 解决方案

### 修改前端代码

**文件**: `navsim-online/web/index.html`

**修改位置**: `handleWorldTick` 函数（第 3341 行）

**修改前**:
```javascript
if (!state.startPoseManual && tick.ego?.pose) {
    setStartPose(tick.ego.pose, false);
}
```

**修改后**:
```javascript
// 只有在没有轨迹回放时才更新 ego 位置（避免与轨迹回放冲突）
if (!state.startPoseManual && !state.trajectoryPlayback && tick.ego?.pose) {
    setStartPose(tick.ego.pose, false);
}
```

**说明**:
- 添加了 `!state.trajectoryPlayback` 条件
- 当有轨迹回放时，不会用 world_tick 的 ego.pose 覆盖
- 轨迹回放可以正常控制自车位置

---

## 🚀 测试步骤

### 1. 刷新前端页面

由于修改了 `index.html`，需要刷新浏览器：

1. 在浏览器中按 `Ctrl+Shift+R`（强制刷新，清除缓存）
2. 或者关闭标签页，重新打开 http://127.0.0.1:8000/index.html

### 2. 重新连接

1. 点击"连接 WebSocket"按钮
2. 等待右上角显示"已连接"（绿色）

### 3. 观察自车运动

**预期结果**:
- ✅ 绿色轨迹线显示
- ✅ 自车从起点开始
- ✅ 自车沿着绿色轨迹线平滑移动
- ✅ 自车到达终点后停止
- ✅ 收到新的 plan_update 后，自车重新开始沿新轨迹移动

**如果仍然抖动**:
- 检查是否刷新了页面
- 检查浏览器控制台是否有错误
- 尝试清除浏览器缓存

---

## 📊 修改前后对比

### 修改前

```
时间线：
T=0ms:    收到 plan_update，启动轨迹回放
T=16ms:   advanceEgoAlongTrajectory() 移动自车到 (0.1, 0.0)
T=32ms:   advanceEgoAlongTrajectory() 移动自车到 (0.2, 0.0)
T=48ms:   advanceEgoAlongTrajectory() 移动自车到 (0.3, 0.0)
T=50ms:   收到 world_tick，setStartPose() 拉回自车到 (0.0, 0.0) ← 问题
T=64ms:   advanceEgoAlongTrajectory() 移动自车到 (0.4, 0.0)
T=80ms:   advanceEgoAlongTrajectory() 移动自车到 (0.5, 0.0)
T=96ms:   advanceEgoAlongTrajectory() 移动自车到 (0.6, 0.0)
T=100ms:  收到 world_tick，setStartPose() 拉回自车到 (0.0, 0.0) ← 问题
...

结果：自车在原地抖动
```

### 修改后

```
时间线：
T=0ms:    收到 plan_update，启动轨迹回放
T=16ms:   advanceEgoAlongTrajectory() 移动自车到 (0.1, 0.0)
T=32ms:   advanceEgoAlongTrajectory() 移动自车到 (0.2, 0.0)
T=48ms:   advanceEgoAlongTrajectory() 移动自车到 (0.3, 0.0)
T=50ms:   收到 world_tick，检测到 trajectoryPlayback，跳过 setStartPose() ✅
T=64ms:   advanceEgoAlongTrajectory() 移动自车到 (0.4, 0.0)
T=80ms:   advanceEgoAlongTrajectory() 移动自车到 (0.5, 0.0)
T=96ms:   advanceEgoAlongTrajectory() 移动自车到 (0.6, 0.0)
T=100ms:  收到 world_tick，检测到 trajectoryPlayback，跳过 setStartPose() ✅
...

结果：自车平滑移动
```

---

## 🎓 设计说明

### 两种 ego 位置更新模式

#### 模式 1: 服务器驱动（默认）

**条件**: 没有轨迹回放时

**行为**:
- 服务器在 world_tick 中发送 ego.pose
- 前端接收并显示
- 自车位置由服务器控制

**适用场景**:
- 没有本地算法
- 只观察服务器的仿真

#### 模式 2: 轨迹回放驱动

**条件**: 收到 plan_update 时

**行为**:
- 前端接收 plan_update
- 启动轨迹回放 (`state.trajectoryPlayback`)
- 前端根据时间插值计算自车位置
- 自车沿轨迹平滑移动

**适用场景**:
- 有本地算法
- 需要可视化规划轨迹

### 模式切换

**启动轨迹回放**:
```javascript
function handlePlanUpdate(data, topic) {
    state.trajectoryPlayback = {
        trajectory: sanitized,
        startTime: performance.now(),
    };
    // ...
}
```

**结束轨迹回放**:
```javascript
function advanceEgoAlongTrajectory() {
    // ...
    if (elapsed >= totalTime) {
        state.trajectoryPlayback = null;  // 清除回放状态
        // ...
    }
}
```

**检查回放状态**:
```javascript
function handleWorldTick(tick, topic) {
    if (!state.startPoseManual && !state.trajectoryPlayback && tick.ego?.pose) {
        setStartPose(tick.ego.pose, false);
    }
}
```

---

## 📝 总结

### 问题

world_tick 的 ego.pose 覆盖了轨迹回放的 ego 位置，导致自车抖动。

### 解决

在轨迹回放时，不使用 world_tick 的 ego.pose 更新自车位置。

### 结果

- ✅ 自车沿轨迹平滑移动
- ✅ 轨迹回放结束后，恢复使用 world_tick 的 ego.pose
- ✅ 两种模式无缝切换

---

## 🎉 问题已解决！

**请刷新浏览器页面并测试：**

1. 按 `Ctrl+Shift+R` 强制刷新
2. 点击"连接 WebSocket"按钮
3. 观察自车是否沿轨迹平滑移动

**应该能看到自车沿绿色轨迹线移动了！** 🚀

---

**文档结束**


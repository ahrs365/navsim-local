# 诊断：自车不可见 + 轨迹起点漂移

## 🔍 问题描述

### 问题 1: 自车不可见
**症状**: 点击开始仿真后，看不到蓝色自车模型（箭头）

### 问题 2: 轨迹起点漂移
**症状**: 绿色轨迹终点未变，但起点有稳定持续的漂移

---

## 🎯 问题分析

### 问题 1 的可能原因

#### 原因 A: 自车模型被隐藏
```javascript
egoGroup.visible = false
```

**检查方法**: 在浏览器控制台运行
```javascript
console.log(egoGroup.visible);
```

---

#### 原因 B: 自车位置在相机视野外
```javascript
// 自车在 (100, 0, 0)，但相机在 (12, 18, 12)
egoGroup.position.x = 100;  // 太远了！
```

**检查方法**: 在浏览器控制台运行
```javascript
console.log(egoGroup.position);
console.log(camera.position);
```

---

#### 原因 C: state.egoPose 为空
```javascript
if (!pose) {
    egoGroup.visible = false;  // ← 这里会隐藏自车
    return;
}
```

**检查方法**: 在浏览器控制台运行
```javascript
console.log(state.egoPose);
```

---

#### 原因 D: updateEgoFromPose() 未被调用
```javascript
// handleWorldTick() 中应该调用
updateEgoFromPose();
```

**检查方法**: 在浏览器控制台运行
```javascript
// 手动调用
updateEgoFromPose();
```

---

### 问题 2 的根本原因：平滑切换算法的 Bug

#### 问题流程：

```
时刻 t=0:
  服务器: ego=(0, 0)
  发送: world_tick: ego=(0, 0)
  
时刻 t=0.05:
  本地算法收到 world_tick: ego=(0, 0)
  规划轨迹: [(0, 0, t=0), (1, 0, t=1), (2, 0, t=2), ...]
  发送: plan_update
  
时刻 t=0.1:
  服务器收到 plan_update
  设置: trajectory_start_time = now (t=0.1)
  开始跟踪轨迹
  
时刻 t=1.1:
  服务器插值: elapsed = 1.1 - 0.1 = 1.0s
  查找轨迹点: t=1.0 → ego=(1, 0)
  发送: world_tick: ego=(1, 0)
  
时刻 t=1.15:
  本地算法收到 world_tick: ego=(1, 0)  ← ⚠️ 有延迟！
  规划新轨迹: [(1, 0, t=0), (2, 0, t=1), (3, 0, t=2), ...]
  发送: plan_update
  
时刻 t=1.2:
  服务器收到新 plan_update
  当前 ego=(1.1, 0)  ← 服务器已经前进了
  
  平滑切换算法：
    在新轨迹上找最近点:
      点 0: (1.0, 0) 距离=0.1
      点 1: (2.0, 0) 距离=0.9
      点 2: (3.0, 0) 距离=1.9
      ...
    
    最近点: 点 0 (距离=0.1)
    offset_time = 0.0  ← ⚠️ 问题！
    trajectory_start_time = now - 0.0 = now
  
时刻 t=1.21:
  服务器插值: elapsed = 1.21 - 1.2 = 0.01s
  查找轨迹点: t=0.01 → ego=(1.001, 0)
  
  ❌ 问题：自车从 (1.1, 0) 跳回 (1.001, 0)
```

---

#### 根本原因：

**平滑切换算法找到的"最近点"总是新轨迹的第一个点**，因为：
1. 新轨迹的第一个点是本地算法收到的 `ego_pose`
2. 但服务器的 `ego_pose` 已经前进了（因为有延迟）
3. 所以新轨迹的第一个点总是落后于服务器的当前位置
4. 设置 `offset_time = 0` 相当于重置时间，导致自车跳回

---

#### 可视化：

```
服务器时间线:
  t=1.0  t=1.1  t=1.2  t=1.3
    |      |      |      |
    ●------●------●------●
   (0,0)  (1,0) (1.1,0)(1.2,0)
                   ↑
                 当前位置

本地算法收到的 ego_pose:
  t=1.15: ego=(1.0, 0)  ← 延迟了 0.15s

新轨迹:
  [(1.0, 0, t=0), (2.0, 0, t=1), ...]
     ↑
   第一个点

平滑切换:
  当前位置: (1.1, 0)
  最近点: (1.0, 0) 距离=0.1
  offset_time = 0.0
  
  下一帧: elapsed=0.01s → ego=(1.001, 0)
  
  ❌ 从 (1.1, 0) 跳回 (1.001, 0)
```

---

## ✅ 解决方案

### 解决方案 1: 自车不可见

#### 步骤 1: 运行诊断脚本

```bash
cd navsim-local
bash check_ego_visibility.sh
```

按照脚本提示，在浏览器控制台运行诊断命令。

---

#### 步骤 2: 检查关键变量

在浏览器控制台 (F12) 运行：

```javascript
// 检查自车可见性
console.log("egoGroup.visible:", egoGroup.visible);
console.log("egoGroup.position:", egoGroup.position);
console.log("state.egoPose:", state.egoPose);

// 强制显示自车
egoGroup.visible = true;
if (state.egoPose) {
    const pos = navToScenePosition(state.egoPose.x, state.egoPose.y);
    egoGroup.position.copy(pos);
    egoGroup.rotation.y = -(state.egoPose.yaw ?? 0);
}
```

---

#### 步骤 3: 调整相机视角

如果自车在视野外，在浏览器控制台运行：

```javascript
// 移动相机到原点上方
camera.position.set(0, 20, 20);
controls.target.set(0, 0, 0);
controls.update();
```

或者在前端界面：
- 按住鼠标右键拖动：平移视角
- 滚轮：缩放
- 按住鼠标左键拖动：旋转视角

---

### 解决方案 2: 轨迹起点漂移

#### 方案 A: 改进平滑切换算法（推荐）

**核心思想**: 不要找"最近点"，而是找"最前面的点"

```python
# 当前方法（有问题）
best_idx = argmin(distance)  # 总是找到第一个点

# 改进方法
# 找到第一个在当前位置"前面"的点
best_idx = 0
for idx, pt in enumerate(sanitized):
    if pt["x"] >= current_x:  # 假设沿 x 轴前进
        best_idx = idx
        break
```

---

#### 方案 B: 使用预测位置（更好）

**核心思想**: 考虑通信延迟，预测未来位置

```python
# 估计延迟
latency = 0.1  # 100ms

# 预测未来位置
predicted_x = current_x + ego_vx * latency
predicted_y = current_y + ego_vy * latency

# 在新轨迹上找最接近预测位置的点
for idx, pt in enumerate(sanitized):
    dx = pt["x"] - predicted_x
    dy = pt["y"] - predicted_y
    dist = sqrt(dx² + dy²)
    # ...
```

---

#### 方案 C: 降低规划频率（临时方案）

**核心思想**: 不要每帧都规划，减少轨迹切换次数

```cpp
// 在 planner.cpp 中
static auto last_plan_time = std::chrono::steady_clock::now();
auto now = std::chrono::steady_clock::now();
auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_plan_time);

// 只有距离上次规划超过 500ms 才重新规划
if (elapsed.count() < 500) {
    return false;  // 跳过规划
}

last_plan_time = now;
// 继续规划...
```

---

#### 方案 D: 禁用平滑切换（最简单）

**核心思想**: 直接从新轨迹的第一个点开始，接受小的跳跃

```python
# 简化版本
self.current_trajectory = sanitized
self.trajectory_start_time = now  # 总是从 t=0 开始
```

**优点**: 简单，不会累积误差  
**缺点**: 会有小的跳跃（但可能不明显）

---

## 🔧 立即修复

### 修复 1: 禁用平滑切换（快速测试）

修改 `navsim-online/server/main.py`:

```python
def _handle_plan_update(self, data: Dict[str, Any]) -> None:
    # ... 清洗轨迹点 ...
    
    if len(sanitized) > 0:
        now = time.time()
        
        # ❌ 禁用平滑切换（临时）
        self.current_trajectory = sanitized
        self.trajectory_start_time = now
        self.trajectory_received_time = now
        
        print(f"[Room {self.room_id}] Received trajectory with {len(sanitized)} points, "
              f"duration: {sanitized[-1]['t']:.2f}s")
```

---

### 修复 2: 降低规划频率

修改 `navsim-local/src/planner.cpp`:

```cpp
bool Planner::solve(...) {
    // 添加频率限制
    static auto last_plan_time = std::chrono::steady_clock::now();
    auto now_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now_time - last_plan_time);
    
    if (elapsed.count() < 200) {  // 最多 5Hz
        return false;
    }
    
    last_plan_time = now_time;
    
    // 原有规划逻辑...
}
```

---

## 📊 验证方法

### 验证 1: 检查服务器日志

启动系统后，查看服务器终端：

```
✅ 正常日志:
[Room demo] Received trajectory with 50 points, duration: 5.00s
[Room demo] Current ego: (1.100, 0.000)
[Room demo] New trajectory first 5 points:
  [0] (1.000, 0.000, t=0.00) dist=0.100m
  [1] (2.000, 0.000, t=1.00) dist=0.900m
  [2] (3.000, 0.000, t=2.00) dist=1.900m
  ...
[Room demo] Smooth transition: starting from point 0/50, offset=0.00s, dist=0.100m
[Room demo] Tracking: tick=20, ego=(1.00, 0.00, 0.00), v=1.00

❌ 问题日志:
[Room demo] Smooth transition: starting from point 0/50, offset=0.00s
  ↑ offset=0.00 说明总是从第一个点开始，会导致跳跃
```

---

### 验证 2: 检查自车位置变化

在浏览器控制台运行：

```javascript
// 监控 10 秒
let lastX = 0;
setInterval(() => {
    if (state.egoPose) {
        const dx = state.egoPose.x - lastX;
        console.log(`ego.x=${state.egoPose.x.toFixed(3)}, dx=${dx.toFixed(3)}`);
        lastX = state.egoPose.x;
    }
}, 1000);
```

**预期输出**:
```
✅ 正常（持续前进）:
ego.x=0.000, dx=0.000
ego.x=1.000, dx=1.000
ego.x=2.000, dx=1.000
ego.x=3.000, dx=1.000

❌ 异常（跳跃）:
ego.x=0.000, dx=0.000
ego.x=1.000, dx=1.000
ego.x=0.950, dx=-0.050  ← 跳回！
ego.x=1.950, dx=1.000
ego.x=1.900, dx=-0.050  ← 又跳回！
```

---

## 📝 总结

### 问题 1: 自车不可见

**可能原因**:
1. `egoGroup.visible = false`
2. 自车位置在相机视野外
3. `state.egoPose` 为空
4. `updateEgoFromPose()` 未被调用

**解决方法**:
- 运行 `check_ego_visibility.sh` 诊断
- 在浏览器控制台手动检查和修复

---

### 问题 2: 轨迹起点漂移

**根本原因**:
- 平滑切换算法总是找到新轨迹的第一个点
- 设置 `offset_time = 0` 导致时间重置
- 自车跳回轨迹起点

**解决方法**:
1. **临时方案**: 禁用平滑切换
2. **推荐方案**: 改进算法，找"最前面的点"而非"最近点"
3. **长期方案**: 使用预测位置 + 降低规划频率

---

**更新时间**: 2025-09-30  
**状态**: 🔍 诊断中，待修复


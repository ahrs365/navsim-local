# 运动模型详解

## 📋 概述

`_integrate_ego(dt)` 函数负责更新自车的状态（位置、姿态、速度）。它采用**双模式**设计：
1. **轨迹跟踪模式**（有规划轨迹时）
2. **简单运动模型**（无规划轨迹时，降级策略）

---

## 🔄 双模式架构

```python
def _integrate_ego(self, dt: float) -> None:
    """Update ego state using trajectory tracking or default motion model."""
    
    if self.current_trajectory and len(self.current_trajectory) > 0:
        self._track_trajectory(dt)  # 模式 1: 轨迹跟踪
    else:
        # 模式 2: 简单运动模型（降级）
        # ...
```

---

## 🎯 模式 1: 轨迹跟踪模式（主要模式）

### 核心思想

**不使用传统的运动学模型积分**，而是直接在规划轨迹上进行**时间插值**。

### 为什么这样设计？

传统方法的问题：
```python
# ❌ 传统方法：使用运动学模型积分
x_new = x_old + v * cos(yaw) * dt
y_new = y_old + v * sin(yaw) * dt
yaw_new = yaw_old + omega * dt

# 问题：
# 1. 累积误差：每次积分都有误差，长时间后偏离轨迹
# 2. 数值不稳定：dt 变化时精度下降
# 3. 需要控制器：需要额外的跟踪控制器来修正误差
```

我们的方法：
```python
# ✅ 我们的方法：时间插值
# 直接在轨迹上查找当前时刻应该在的位置
elapsed = now - trajectory_start_time
position = interpolate(trajectory, elapsed)

# 优势：
# 1. 无累积误差：每次都基于原始轨迹计算
# 2. 数值稳定：与 dt 无关
# 3. 完美跟踪：自车精确沿轨迹移动
```

---

### 算法详解

#### 步骤 1: 计算经过时间

```python
now = time.time()
elapsed = now - self.trajectory_start_time
```

**说明**:
- `trajectory_start_time`: 收到轨迹的时刻
- `elapsed`: 从轨迹开始到现在经过的时间

---

#### 步骤 2: 查找当前时间段

```python
# 轨迹点格式: [
#   {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0},
#   {"x": 1.0, "y": 0.0, "yaw": 0.0, "t": 1.0},
#   {"x": 2.0, "y": 0.0, "yaw": 0.0, "t": 2.0},
#   ...
# ]

for i in range(1, len(traj)):
    prev_point = traj[i-1]
    curr_point = traj[i]
    prev_t = prev_point["t"]  # 例如: 1.0
    curr_t = curr_point["t"]  # 例如: 2.0
    
    if elapsed <= curr_t:
        # 找到了！elapsed 在 [prev_t, curr_t] 之间
        break
```

**示例**:
- 轨迹点: `t=0, t=1, t=2, t=3, ...`
- 当前时间: `elapsed = 1.5s`
- 找到区间: `[t=1, t=2]`

---

#### 步骤 3: 线性插值

```python
# 计算插值比例
span = curr_t - prev_t  # 例如: 2.0 - 1.0 = 1.0
ratio = (elapsed - prev_t) / span  # 例如: (1.5 - 1.0) / 1.0 = 0.5

# 插值位置
x = prev_point["x"] + (curr_point["x"] - prev_point["x"]) * ratio
y = prev_point["y"] + (curr_point["y"] - prev_point["y"]) * ratio

# 插值角度（考虑角度环绕）
yaw_diff = wrap_yaw(curr_point["yaw"] - prev_point["yaw"])
yaw = wrap_yaw(prev_point["yaw"] + yaw_diff * ratio)
```

**数学公式**:
```
x(t) = x₀ + (x₁ - x₀) * α
y(t) = y₀ + (y₁ - y₀) * α
yaw(t) = yaw₀ + Δyaw * α

其中:
α = (t - t₀) / (t₁ - t₀)  ∈ [0, 1]
Δyaw = wrap(yaw₁ - yaw₀)  # 处理角度环绕
```

**示例**:
```
已知:
  点 A: (x=0, y=0, yaw=0, t=1.0)
  点 B: (x=2, y=0, yaw=0, t=2.0)
  当前时间: elapsed=1.5

计算:
  ratio = (1.5 - 1.0) / (2.0 - 1.0) = 0.5
  x = 0 + (2 - 0) * 0.5 = 1.0
  y = 0 + (0 - 0) * 0.5 = 0.0
  yaw = 0 + (0 - 0) * 0.5 = 0.0

结果: 自车在 (1.0, 0.0, 0.0)
```

---

#### 步骤 4: 计算速度

```python
# 计算位移
dx = curr_point["x"] - prev_point["x"]
dy = curr_point["y"] - prev_point["y"]

# 计算标量速度
v = sqrt(dx² + dy²) / span

# 计算角速度
omega = yaw_diff / span
```

**说明**:
- 速度是**平均速度**（两点之间的平均值）
- 不是瞬时速度（因为轨迹点是离散的）
- 假设 `vy = 0`（无侧向速度）

**示例**:
```
已知:
  点 A: (x=0, y=0, yaw=0, t=1.0)
  点 B: (x=2, y=0, yaw=0.1, t=2.0)

计算:
  dx = 2 - 0 = 2
  dy = 0 - 0 = 0
  span = 2.0 - 1.0 = 1.0
  
  v = sqrt(2² + 0²) / 1.0 = 2.0 m/s
  omega = 0.1 / 1.0 = 0.1 rad/s

结果: vx=2.0, vy=0.0, omega=0.1
```

---

#### 步骤 5: 更新状态

```python
# 更新位姿
self.ego_pose["x"] = x
self.ego_pose["y"] = y
self.ego_pose["yaw"] = yaw

# 更新速度（车体坐标系）
self.ego_twist["vx"] = v
self.ego_twist["vy"] = 0.0
self.ego_twist["omega"] = omega
```

---

### 边界情况处理

#### 情况 1: 轨迹未开始

```python
if elapsed <= traj[0]["t"]:
    # 停在起点
    self.ego_pose = traj[0]
    self.ego_twist = {"vx": 0, "vy": 0, "omega": 0}
```

#### 情况 2: 轨迹已完成

```python
if elapsed >= traj[-1]["t"]:
    # 停在终点
    self.ego_pose = traj[-1]
    self.ego_twist = {"vx": 0, "vy": 0, "omega": 0}
    # 清空轨迹
    self.current_trajectory = []
```

---

### 角度环绕处理

```python
def wrap_yaw(angle: float) -> float:
    """Normalize yaw into [-π, π]"""
    return (angle + math.pi) % (2 * math.pi) - math.pi
```

**为什么需要？**

```
错误示例:
  yaw₀ = 3.0 rad  (≈ 172°)
  yaw₁ = -3.0 rad (≈ -172°)
  
  直接插值: yaw_diff = -3.0 - 3.0 = -6.0 rad (错误！)
  正确插值: yaw_diff = wrap(-3.0 - 3.0) = 0.28 rad (正确)
```

---

## 🔧 模式 2: 简单运动模型（降级策略）

### 何时使用？

- 没有收到 `plan_update`
- 轨迹已完成且被清空
- 用于演示/测试

### 运动学模型

采用**2D 刚体运动学模型**（非完整约束）：

```python
# 车体坐标系速度
vx_body = self.ego_twist["vx"]
vy_body = self.ego_twist["vy"]
omega = self.ego_twist["omega"]

# 转换到世界坐标系
vx_world = vx_body * cos(yaw) - vy_body * sin(yaw)
vy_world = vx_body * sin(yaw) + vy_body * cos(yaw)

# 积分更新位置
x_new = x_old + vx_world * dt
y_new = y_old + vy_world * dt
yaw_new = wrap_yaw(yaw_old + omega * dt)
```

### 数学推导

**坐标系转换**:
```
世界坐标系 (X, Y) → 车体坐标系 (x, y)

旋转矩阵:
R = [cos(yaw)  -sin(yaw)]
    [sin(yaw)   cos(yaw)]

速度转换:
[vx_world] = R * [vx_body]
[vy_world]       [vy_body]
```

**欧拉积分**:
```
x(t+dt) = x(t) + vx_world * dt
y(t+dt) = y(t) + vy_world * dt
yaw(t+dt) = yaw(t) + omega * dt
```

### 演示运动

为了让演示更生动，添加了正弦波漫游：

```python
# 速度漫游
wander = sin(tick_id / 40.0) * 0.02
vx = max(0.2, min(v_max, vx_old + wander))

# 角速度漫游
omega = 0.05 + sin(tick_id / 60.0) * 0.04
```

**效果**: 自车会缓慢地蛇形移动

---

## 📊 两种模式对比

| 特性 | 轨迹跟踪模式 | 简单运动模型 |
|------|------------|------------|
| **使用场景** | 有规划轨迹 | 无规划轨迹 |
| **方法** | 时间插值 | 运动学积分 |
| **精度** | ✅ 完美跟踪 | ⚠️ 累积误差 |
| **稳定性** | ✅ 数值稳定 | ⚠️ 依赖 dt |
| **速度来源** | 轨迹计算 | 手动设置 |
| **适用性** | 闭环仿真 | 演示/测试 |

---

## 🎯 优势分析

### 轨迹跟踪模式的优势

1. **无累积误差**
   - 每次都基于原始轨迹计算
   - 不会随时间偏离

2. **数值稳定**
   - 与时间步长 `dt` 无关
   - 不受仿真频率影响

3. **完美跟踪**
   - 自车精确沿轨迹移动
   - 不需要额外的跟踪控制器

4. **简单高效**
   - 只需线性插值
   - 计算量小

---

## 🔍 局限性

### 当前实现的局限

1. **线性插值**
   - 轨迹点之间是直线
   - 不够平滑

2. **平均速度**
   - 速度是两点之间的平均值
   - 不是瞬时速度

3. **无侧向速度**
   - 假设 `vy = 0`
   - 不适用于漂移等场景

4. **无动力学约束**
   - 不考虑加速度限制
   - 不考虑转向限制

---

## 🚀 未来改进方向

### 1. 样条插值

```python
# 使用三次样条代替线性插值
from scipy.interpolate import CubicSpline

spline_x = CubicSpline(t_points, x_points)
spline_y = CubicSpline(t_points, y_points)

x = spline_x(elapsed)
y = spline_y(elapsed)
```

**优势**: 更平滑的轨迹

---

### 2. 瞬时速度计算

```python
# 使用样条的导数
vx = spline_x.derivative()(elapsed)
vy = spline_y.derivative()(elapsed)
```

**优势**: 更准确的速度

---

### 3. 动力学约束

```python
# 检查加速度限制
a = (v_new - v_old) / dt
if abs(a) > a_max:
    v_new = v_old + sign(a) * a_max * dt
```

**优势**: 更真实的车辆行为

---

### 4. 自行车模型

```python
# 使用自行车运动学模型
x_new = x_old + v * cos(yaw) * dt
y_new = y_old + v * sin(yaw) * dt
yaw_new = yaw_old + (v / wheelbase) * tan(steer) * dt
```

**优势**: 考虑转向约束

---

## 📝 总结

### 核心设计思想

**轨迹跟踪模式**采用**时间插值**而非**运动学积分**，这是一个巧妙的设计：

✅ **优点**:
- 无累积误差
- 数值稳定
- 完美跟踪
- 简单高效

⚠️ **缺点**:
- 线性插值不够平滑
- 速度是平均值
- 无动力学约束

### 适用场景

- ✅ **轨迹规划算法开发**: 完美
- ✅ **控制算法测试**: 适合
- ✅ **仿真验证**: 良好
- ⚠️ **高保真仿真**: 需要改进

---

**更新时间**: 2025-09-30  
**状态**: ✅ 已实现并测试


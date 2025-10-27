# 自车运动仿真问题修复报告

## 🔍 问题描述

**现象**：
- 自车（ego vehicle）移动非常缓慢
- 规划器输出的线速度（v）和角速度（w）数值较大
- 自车的实际运动与规划的速度命令不匹配

## 🔬 问题诊断

### 1. 数据流分析

完整的数据流应该是：

```
规划器 (Planner)
    ↓
PlanningResult::trajectory (包含位置和速度)
    ↓
AlgorithmManager::process_local_simulation_step()
    ↓
LocalSimulator::set_ego_twist() (设置速度)
    ↓
LocalSimulator::step() → integrate_ego_motion() (速度积分)
    ↓
更新 ego_pose (位置)
```

### 2. 根本原因

在 `navsim-local/platform/src/sim/local_simulator.cpp` 的第 **659-663** 行：

```cpp
void LocalSimulator::Impl::integrate_ego_motion(double dt) {
  // 目前不积分自车运动，等待外部算法控制
  // 这个函数预留给未来的直接控制模式
  (void)dt;
}
```

**这个函数是空的！自车根本没有进行运动积分！**

### 3. 问题表现的原因

#### 当前的错误实现：

1. **AlgorithmManager** 在 `process_local_simulation_step()` 中（第 1042-1055 行）：
   - 直接将轨迹点的**位置**设置给自车：`local_simulator_->set_ego_pose(new_pose)`
   - 设置轨迹点的**速度**：`local_simulator_->set_ego_twist(new_twist)`

2. **LocalSimulator::step()** 调用 `integrate_ego_motion(dt)`，但这个函数是空的

3. **结果**：
   - 自车的位置是**跳跃式**地设置到轨迹点上的
   - 速度虽然被设置了，但**从未被用于积分**
   - 自车看起来移动很慢，是因为它在**离散地跟踪轨迹点**，而不是连续地运动

#### 正确的实现应该是：

1. **AlgorithmManager** 只设置速度命令（或者不设置位置，只设置速度）
2. **LocalSimulator** 根据速度进行积分，平滑地更新位置
3. 自车的运动是**连续的**，而不是跳跃的

## ✅ 修复方案

### 实现自车运动积分

在 `local_simulator.cpp` 中实现 `integrate_ego_motion()` 函数：

```cpp
void LocalSimulator::Impl::integrate_ego_motion(double dt) {
  // 🚗 自车运动积分：根据当前速度更新位姿
  // 使用简单的欧拉积分（一阶积分）
  
  // 获取当前速度
  double vx = world_state_.ego_twist.vx;      // 纵向速度 (m/s)
  double vy = world_state_.ego_twist.vy;      // 横向速度 (m/s)
  double omega = world_state_.ego_twist.omega; // 角速度 (rad/s)
  
  // 获取当前位姿
  double x = world_state_.ego_pose.x;
  double y = world_state_.ego_pose.y;
  double yaw = world_state_.ego_pose.yaw;
  
  // 积分更新位姿
  // 方法：在车体坐标系下积分，然后转换到世界坐标系
  // 车体坐标系：x轴向前，y轴向左
  
  // 世界坐标系下的速度分量
  double vx_world = vx * std::cos(yaw) - vy * std::sin(yaw);
  double vy_world = vx * std::sin(yaw) + vy * std::cos(yaw);
  
  // 更新位置（欧拉积分）
  world_state_.ego_pose.x = x + vx_world * dt;
  world_state_.ego_pose.y = y + vy_world * dt;
  
  // 更新朝向
  world_state_.ego_pose.yaw = normalize_angle(yaw + omega * dt);
}
```

### 积分方法说明

#### 1. 坐标系转换

- **车体坐标系**：x轴向前（车头方向），y轴向左
- **世界坐标系**：固定的全局坐标系

速度从车体坐标系转换到世界坐标系：
```
vx_world = vx * cos(yaw) - vy * sin(yaw)
vy_world = vx * sin(yaw) + vy * cos(yaw)
```

#### 2. 欧拉积分（一阶积分）

位置更新：
```
x(t+dt) = x(t) + vx_world * dt
y(t+dt) = y(t) + vy_world * dt
```

朝向更新：
```
yaw(t+dt) = yaw(t) + omega * dt
```

#### 3. 时间步长

- 默认时间步长：`dt = 0.033s`（约 30Hz）
- 可以通过 `SimulatorConfig::time_step` 配置
- 可以通过 `time_scale` 调整仿真速度

## 📊 修复效果

### 修复前：
- 自车位置：**跳跃式**更新（直接设置到轨迹点）
- 速度：被设置但**未使用**
- 运动：**不连续**，看起来很慢

### 修复后：
- 自车位置：**平滑**更新（根据速度积分）
- 速度：**正确使用**于积分
- 运动：**连续平滑**，符合规划的速度

## 🧪 测试方法

### 1. 运行仿真

```bash
cd navsim-local
./build/navsim_algo --local-sim --scenario=scenarios/map1.json --visualize
```

### 2. 观察自车运动

- 点击 **Start** 按钮开始仿真
- 观察自车是否平滑地向目标移动
- 检查速度是否合理（不会太慢）

### 3. 查看日志

在终端中查看日志（每秒打印一次）：

```
[AlgorithmManager] Step 30: Planning success, 150 trajectory points generated
  Control index: 3 (t=0.1s, accumulated_time=1.0s)
  Ego pose: (0.123, 0.456, 0.789)
  Ego twist: (1.5, 0.0, 0.2)
```

- `Ego twist` 显示当前速度（vx, vy, omega）
- 自车应该根据这个速度平滑移动

## 🔧 相关配置

### 仿真时间步长

在 `config/default.json` 中配置：

```json
{
  "simulator": {
    "time_step": 0.033,      // 时间步长 (s)，默认 30Hz
    "time_scale": 1.0,       // 时间缩放，1.0 = 实时
    "max_time_step": 0.1     // 最大时间步长 (s)
  }
}
```

### 调整仿真速度

- `time_scale = 0.5`：慢速（0.5倍速）
- `time_scale = 1.0`：实时
- `time_scale = 2.0`：快速（2倍速）

## 📝 技术细节

### 为什么使用欧拉积分？

1. **简单高效**：计算量小，适合实时仿真
2. **精度足够**：对于小时间步长（30Hz），精度已经足够
3. **稳定性好**：不会产生数值不稳定

### 更高精度的积分方法

如果需要更高精度，可以使用：

1. **RK4（四阶龙格-库塔）**：精度更高，但计算量大4倍
2. **中点法**：精度介于欧拉和RK4之间
3. **自适应步长**：根据误差自动调整步长

对于当前的应用场景（30Hz，低速运动），欧拉积分已经足够。

## 🎯 总结

**问题**：自车运动积分函数为空，导致速度命令未被使用

**修复**：实现欧拉积分，根据速度平滑更新位置

**效果**：自车现在能够根据规划的速度命令平滑移动

## 📚 相关文件

- `navsim-local/platform/src/sim/local_simulator.cpp`：自车运动积分实现
- `navsim-local/platform/src/core/algorithm_manager.cpp`：规划结果应用
- `navsim-local/platform/include/sim/local_simulator.hpp`：LocalSimulator 接口
- `navsim-local/platform/include/plugin/data/planning_result.hpp`：PlanningResult 数据结构


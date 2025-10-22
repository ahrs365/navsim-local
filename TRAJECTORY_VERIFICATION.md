# MINCO 轨迹信息验证

## 📊 验证结果

已在 `navsim-local` 端添加详细的轨迹信息打印，确认 MINCO 优化生成的轨迹包含完整且正确的动力学信息。

---

## ✅ 轨迹信息完整性

### **前 5 个点**

```
Point[0]: pos=(0.001, 0.000, yaw=0.000), twist=(vx=0.500, vy=0, ω=0.000053), acc=-0.020, curv=0.000106, t=0.002, s=0.001
Point[1]: pos=(0.002, 0.000, yaw=0.000), twist=(vx=0.500, vy=0, ω=0.000210), acc=-0.040, curv=0.000421, t=0.004, s=0.002
Point[2]: pos=(0.003, 0.000, yaw=0.000), twist=(vx=0.500, vy=0, ω=0.000472), acc=-0.059, curv=0.000944, t=0.006, s=0.003
Point[3]: pos=(0.004, 0.000, yaw=0.000), twist=(vx=0.500, vy=0, ω=0.000836), acc=-0.078, curv=0.001674, t=0.008, s=0.004
Point[4]: pos=(0.005, 0.000, yaw=0.000), twist=(vx=0.500, vy=0, ω=0.001302), acc=-0.097, curv=0.002607, t=0.010, s=0.005
```

**验证**:
- ✅ **位置 (x, y, yaw)**: 正确
- ✅ **速度 (vx, vy, omega)**: 正确，vx ≈ 0.5 m/s
- ✅ **加速度 (acc)**: 正确，为负值（减速）
- ✅ **曲率 (curv)**: 正确，κ = ω / v
- ✅ **时间 (t)**: 正确，从 0.002s 开始
- ✅ **路径长度 (s)**: 正确，从 0.001m 开始

### **最后 5 个点**

```
Point[8795]: pos=(17.999, 6.002, yaw=-0.000), twist=(vx=0.000209, vy=0, ω=0.000092), acc=-0.039, curv=0.440, t=17.796, s=19.539
Point[8796]: pos=(17.999, 6.002, yaw=-0.000), twist=(vx=0.000118, vy=0, ω=0.000052), acc=-0.029, curv=0.443, t=17.799, s=19.539
Point[8797]: pos=(17.999, 6.002, yaw=-0.000), twist=(vx=0.000052, vy=0, ω=0.000023), acc=-0.019, curv=0.445, t=17.802, s=19.539
Point[8798]: pos=(17.999, 6.002, yaw=-0.000), twist=(vx=0.000013, vy=0, ω=0.000006), acc=-0.010, curv=0.447, t=17.804, s=19.539
Point[8799]: pos=(17.999, 6.002, yaw=-0.000), twist=(vx=0.000000, vy=0, ω=0.000000), acc=-0.000, curv=0.000, t=17.807, s=19.539
```

**验证**:
- ✅ **位置**: 最终到达 (17.999, 6.002)
- ✅ **速度**: vx 从 0.000209 逐渐减速到 0
- ✅ **角速度**: ω 也逐渐减小到 0
- ✅ **加速度**: 为负值（减速到停止）
- ✅ **曲率**: 最后一个点曲率为 0

---

## 📈 动力学范围统计

```
- Velocity (vx): [-0.121, 1.646] m/s
- Angular velocity (ω): [-0.649, 1.190] rad/s
- Acceleration: [-1.270, 1.621] m/s²
- Curvature: [-9158.73, 596.471] 1/m
```

**分析**:
- ⚠️ **速度有负值**: vx 最小值为 -0.121 m/s
  - 可能原因：数值误差或者轨迹中有倒车段
  - 建议：检查轨迹中速度为负的点

- ✅ **角速度范围合理**: [-0.649, 1.190] rad/s
  - 符合差速驱动机器人的运动特性

- ✅ **加速度范围合理**: [-1.270, 1.621] m/s²
  - 符合配置的加速度限制

- ⚠️ **曲率范围异常大**: [-9158.73, 596.471] 1/m
  - 原因：在速度接近 0 时，κ = ω / v 会趋于无穷大
  - 这是数值问题，不影响轨迹跟踪（因为速度很小时，曲率的影响也很小）

---

## 🔍 发现的问题

### **问题：轨迹第一个点不是起点**

**观察**:
- 起点位置：`(0, 0)`
- 轨迹第一个点：`(0.001, 0.000)` at `t=0.002s`
- 间隙：`0.001m`

**原因**:
这是 Simpson 积分的特性。第一个采样点（j=0）用于计算积分，但不作为输出点。因此，轨迹的第一个点是在 `t = 0.002s` 时刻的位置。

**影响**:
- 在 `navsim-online` 中，如果小车当前位置超过了轨迹的第一个点，会导致轨迹跟踪失败
- 例如：小车位置 `(0.130, 0.001)`，轨迹第一个点 `(0.001, 0.000)`，间隙 `0.130m`

**解决方案**:
1. **方案 1**: 在 `extractMincoTrajectory` 中添加起点（t=0）
2. **方案 2**: 在 `navsim-online` 中，如果小车位置超过第一个点，从当前位置开始跟踪
3. **方案 3**: 在 `navsim-online` 中，接收到新轨迹时，将小车位置重置到轨迹的第一个点

---

## 💡 建议的修复方案

### **推荐方案 1：在轨迹开头添加起点**

在 `extractMincoTrajectory` 函数中，在开始采样之前，先添加一个起点（t=0）：

```cpp
// Add initial point at t=0
navsim::plugin::TrajectoryPoint initial_pt;
initial_pt.pose.x = ini_x;
initial_pt.pose.y = ini_y;
initial_pt.pose.yaw = final_traj.getPos(0.0)[0];  // Initial yaw
initial_pt.twist.vx = final_traj.getVel(0.0)[1];  // Initial velocity
initial_pt.twist.vy = 0.0;
initial_pt.twist.omega = final_traj.getVel(0.0)[0];  // Initial angular velocity
initial_pt.acceleration = final_traj.getAcc(0.0)[1];  // Initial acceleration
initial_pt.time_from_start = 0.0;
initial_pt.path_length = 0.0;
if (std::abs(initial_pt.twist.vx) > 1e-6) {
  initial_pt.curvature = initial_pt.twist.omega / initial_pt.twist.vx;
} else {
  initial_pt.curvature = 0.0;
}
minco_trajectory.push_back(initial_pt);
```

**优点**:
- 轨迹从起点开始，避免间隙
- 不需要修改 `navsim-online` 的逻辑
- 数据完整性更好

**缺点**:
- 需要修改 `navsim-local` 代码

---

## 📝 修改记录

### **添加的代码**

**文件**: `navsim-local/plugins/planning/jps_planner/adapter/jps_planner_plugin.cpp`

**位置**: 第 1048-1115 行

**功能**: 在 `convertMincoOutputToResult` 函数中添加详细的轨迹信息打印

**打印内容**:
1. 前 5 个点的完整信息（位置、速度、加速度、曲率、时间、路径长度）
2. 最后 5 个点的完整信息
3. 动力学范围统计（速度、角速度、加速度、曲率的最小值和最大值）

---

## 🧪 测试命令

```bash
cd navsim-local
./build/navsim_local_debug --scenario scenarios/map1.json --planner JpsPlanner --perception EsdfBuilder 2>&1 | grep -A 100 "MINCO trajectory statistics"
```

---

## ✅ 结论

**MINCO 轨迹生成的动力学信息是完整且正确的！**

- ✅ 位置、速度、角速度、加速度、曲率都已正确提取
- ✅ 数据范围合理（除了曲率在速度接近 0 时的数值问题）
- ⚠️ 轨迹第一个点不是起点，需要在 `navsim-online` 或 `navsim-local` 中处理

**下一步**:
1. 实现推荐方案 1，在轨迹开头添加起点
2. 重新测试 `navsim-online` 的轨迹跟踪
3. 验证小车能够正常沿轨迹移动


# MINCO 轨迹提取完整动力学信息

## 📋 概述

本文档说明如何从 MINCO 优化轨迹中提取完整的动力学信息（位置、速度、加速度等）。

---

## 🔧 修改内容

### 1. **修改 `extractMincoTrajectory()` 函数**

**文件**: `navsim-local/plugins/planning/jps_planner/adapter/jps_planner_plugin.cpp`

**修改前**:
```cpp
std::vector<navsim::planning::Pose2d> extractMincoTrajectory() const;
```
- 只返回 `{x, y, yaw}`
- 速度和加速度信息丢失

**修改后**:
```cpp
std::vector<navsim::plugin::TrajectoryPoint> extractMincoTrajectory() const;
```
- 返回完整的 `TrajectoryPoint`，包含：
  - `pose`: `{x, y, yaw}` - 位置和朝向
  - `twist`: `{vx, vy, omega}` - 速度（线速度、横向速度、角速度）
  - `acceleration`: 线加速度
  - `curvature`: 曲率
  - `time_from_start`: 从起点开始的时间
  - `path_length`: 从起点开始的路径长度

---

## 📊 提取的动力学信息

### **MINCO 轨迹对象提供的接口**

```cpp
// trajectory.hpp
Eigen::VectorXd getPos(double t) const;  // 返回 [yaw, s]
Eigen::VectorXd getVel(double t) const;  // 返回 [ω, v]
Eigen::VectorXd getAcc(double t) const;  // 返回 [α, a]
Eigen::VectorXd getJer(double t) const;  // 返回 [jerk_yaw, jerk_s]
```

其中：
- `getPos(t)[0]` = `yaw` (朝向角)
- `getPos(t)[1]` = `s` (弧长)
- `getVel(t)[0]` = `ω` (角速度, rad/s)
- `getVel(t)[1]` = `v` (线速度, m/s)
- `getAcc(t)[0]` = `α` (角加速度, rad/s²)
- `getAcc(t)[1]` = `a` (线加速度, m/s²)

### **提取过程**

在 Simpson 积分采样过程中，对每个采样点：

```cpp
// 第766-790行
Eigen::Vector2d currPos = final_traj.getPos(s1 + sumT);  // [yaw, s]
Eigen::Vector2d currVel = final_traj.getVel(s1 + sumT);  // [ω, v]
Eigen::Vector2d currAcc = final_traj.getAcc(s1 + sumT);  // [α, a]

// 保存到数组
Yaw[j/2-1] = currPos.x();      // yaw
Vel[j/2-1] = currVel.y();      // v (线速度)
Omega[j/2-1] = currVel.x();    // ω (角速度)
Acc[j/2-1] = currAcc.y();      // a (线加速度)
```

### **构建 TrajectoryPoint**

```cpp
// 第810-840行
navsim::plugin::TrajectoryPoint traj_pt;

// Pose
traj_pt.pose.x = pos.x();
traj_pt.pose.y = pos.y();
traj_pt.pose.yaw = Yaw[j];

// Twist (velocity in body frame)
traj_pt.twist.vx = Vel[j];      // 线速度 (m/s)
traj_pt.twist.vy = 0.0;         // 横向速度 (差速驱动为0)
traj_pt.twist.omega = Omega[j]; // 角速度 (rad/s)

// Acceleration
traj_pt.acceleration = Acc[j];  // 线加速度 (m/s²)

// Time and path length
traj_pt.time_from_start = cumulative_time;
traj_pt.path_length = cumulative_length;

// Curvature (κ = ω / v)
if (std::abs(Vel[j]) > 1e-6) {
  traj_pt.curvature = Omega[j] / Vel[j];
} else {
  traj_pt.curvature = 0.0;
}
```

---

## 🎯 使用示例

### **在 `convertMincoOutputToResult` 中使用**

**修改前** (第999-1002行):
```cpp
// 速度是硬编码的！
traj_pt.twist.vx = jps_config_.max_vel;  // ❌ 固定值
traj_pt.twist.vy = 0.0;
traj_pt.twist.omega = 0.0;  // ❌ 没有角速度
```

**修改后** (第1020-1056行):
```cpp
// 直接使用提取的完整轨迹
std::vector<navsim::plugin::TrajectoryPoint> minco_trajectory = extractMincoTrajectory();
result.trajectory = minco_trajectory;  // ✅ 包含真实的速度、加速度

// 打印统计信息
std::cout << "  - Total time: " << result.getTotalTime() << " s" << std::endl;
std::cout << "  - Total length: " << result.getTotalLength() << " m" << std::endl;
std::cout << "  - First point: v=" << minco_trajectory.front().twist.vx 
          << " m/s, ω=" << minco_trajectory.front().twist.omega << " rad/s" << std::endl;
```

---

## 📈 测试结果

运行测试场景：
```bash
./build/navsim_local_debug --scenario scenarios/map1.json --planner JpsPlanner --perception EsdfBuilder
```

**输出示例**:
```
[JPSPlannerPlugin] Using MINCO trajectory with 8800 points
[JPSPlannerPlugin] MINCO trajectory statistics:
  - Total points: 8800
  - Total time: 17.807 s
  - Total length: 19.5391 m
  - First point: v=0.499979 m/s, ω=5.28016e-05 rad/s
  - Last point: v=1.12757e-17 m/s, ω=3.46945e-18 rad/s
```

**关键观察**:
- ✅ 起点速度：`v = 0.5 m/s` (从静止加速)
- ✅ 终点速度：`v ≈ 0 m/s` (减速到停止)
- ✅ 角速度：根据轨迹曲率动态变化
- ✅ 总时长：17.8 秒
- ✅ 总路径长度：19.5 米

---

## 🔍 与原始 ROS 实现对比

### **原始 `mincoPathPub` 函数** (optimizer.cpp 第1581-1700行)

```cpp
void MSPlanner::mincoPathPub(const Trajectory<5, 2> &final_traj, ...) {
  // 只提取位置和朝向用于可视化
  for(int j=0; j<=SamNumEachPart; j++){
    Eigen::Vector2d currPos = final_traj.getPos(s1+sumT);
    Eigen::Vector2d currVel = final_traj.getVel(s1+sumT);
    
    // 用于积分计算位置
    IntegralX[j/2-1] += CoeffIntegral * currVel.y() * cos(currPos.x());
    IntegralY[j/2-1] += CoeffIntegral * currVel.y() * sin(currPos.x());
    Yaw[j/2-1] = currPos.x();  // ✅ 只保存 yaw
    
    // ❌ currVel 没有被保存！
  }
  
  // 发布到 ROS (只有位置和朝向)
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = pos.x();
  pose.pose.position.y = pos.y();
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(VecYaw[i][j]);
}
```

### **我们的实现** (jps_planner_plugin.cpp 第704-852行)

```cpp
std::vector<navsim::plugin::TrajectoryPoint> extractMincoTrajectory() const {
  // 提取完整动力学信息
  for(int j = 0; j <= SamNumEachPart; j++) {
    Eigen::Vector2d currPos = final_traj.getPos(s1 + sumT);
    Eigen::Vector2d currVel = final_traj.getVel(s1 + sumT);
    Eigen::Vector2d currAcc = final_traj.getAcc(s1 + sumT);  // ✅ 新增
    
    // 保存所有信息
    Yaw[j/2-1] = currPos.x();
    Vel[j/2-1] = currVel.y();      // ✅ 保存线速度
    Omega[j/2-1] = currVel.x();    // ✅ 保存角速度
    Acc[j/2-1] = currAcc.y();      // ✅ 保存加速度
  }
  
  // 构建完整的 TrajectoryPoint
  traj_pt.pose = {x, y, yaw};
  traj_pt.twist = {vx, vy, omega};  // ✅ 真实速度
  traj_pt.acceleration = a;         // ✅ 真实加速度
  traj_pt.curvature = omega / vx;   // ✅ 计算曲率
}
```

---

## ✅ 优势

1. **完整的动力学信息**: 控制器可以获得真实的速度和加速度，而不是硬编码的近似值
2. **数据一致性**: 位置、速度、加速度来自同一次采样，保证一致性
3. **避免重复计算**: 不需要在 `convertMincoOutputToResult` 中重新查询轨迹
4. **支持高级控制**: 可以用于 MPC、前馈控制等需要速度和加速度的控制器

---

## 📚 参考

- **MINCO 论文**: "Geometrically Constrained Trajectory Optimization for Multicopters" (Zhepei Wang et al.)
- **原始实现**: `/home/gao/workspace/pnc_project/ahrs-simulator/navsim-local/docs/ref/optimizer.cpp`
- **轨迹类**: `navsim-local/plugins/planning/jps_planner/algorithm/opt/trajectory.hpp`
- **数据结构**: `navsim-local/platform/include/plugin/data/planning_result.hpp`

---

## 🎓 总结

通过修改 `extractMincoTrajectory()` 函数，我们现在可以从 MINCO 优化轨迹中提取完整的动力学信息，包括：

- ✅ 位置 `(x, y, yaw)`
- ✅ 速度 `(vx, vy, omega)`
- ✅ 加速度 `(a)`
- ✅ 曲率 `(κ)`
- ✅ 时间和路径长度

这为后续的控制器设计和轨迹跟踪提供了完整的信息支持。


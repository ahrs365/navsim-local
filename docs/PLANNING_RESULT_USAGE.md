# PlanningResult 使用指南

## 📋 概述

`PlanningResult` 包含了规划器输出的完整轨迹信息，包括位置、速度、加速度、曲率等动力学信息。

---

## 📊 数据结构

### **PlanningResult**

```cpp
struct PlanningResult {
  std::vector<TrajectoryPoint> trajectory;  // 轨迹点序列
  bool success = false;                     // 规划是否成功
  std::string failure_reason;               // 失败原因
  std::string planner_name;                 // 规划器名称
  std::map<std::string, double> metadata;   // 元数据
  // ...
};
```

### **TrajectoryPoint**

```cpp
struct TrajectoryPoint {
  // 位置和朝向
  planning::Pose2d pose;           // {x, y, yaw}
  
  // 速度
  planning::Twist2d twist;         // {vx, vy, omega}
  
  // 加速度
  double acceleration = 0.0;       // 纵向加速度 (m/s²)
  
  // 转向
  double steering_angle = 0.0;     // 转向角 (rad)
  
  // 曲率
  double curvature = 0.0;          // 曲率 (1/m)
  
  // 时间
  double time_from_start = 0.0;    // 从起点开始的时间 (s)
  
  // 路径长度
  double path_length = 0.0;        // 从起点开始的路径长度 (m)
};
```

### **Pose2d**

```cpp
struct Pose2d {
  double x = 0.0;    // X 坐标 (m)
  double y = 0.0;    // Y 坐标 (m)
  double yaw = 0.0;  // 朝向角 (rad)
};
```

### **Twist2d**

```cpp
struct Twist2d {
  double vx = 0.0;    // 纵向速度 (m/s)
  double vy = 0.0;    // 横向速度 (m/s)
  double omega = 0.0; // 角速度 (rad/s)
};
```

---

## 🔍 访问轨迹信息

### **1. 基本访问**

```cpp
bool JpsPlannerPlugin::plan(const navsim::planning::PlanningContext& context,
                             std::chrono::milliseconds deadline,
                             navsim::plugin::PlanningResult& result) {
  // ... 规划过程 ...
  
  // 检查规划是否成功
  if (!result.success) {
    std::cerr << "Planning failed: " << result.failure_reason << std::endl;
    return false;
  }
  
  // 获取轨迹点数量
  size_t num_points = result.trajectory.size();
  std::cout << "Trajectory has " << num_points << " points" << std::endl;
  
  // 访问第一个点
  const auto& first_point = result.trajectory.front();
  std::cout << "Start position: (" << first_point.pose.x << ", " 
            << first_point.pose.y << ")" << std::endl;
  std::cout << "Start velocity: " << first_point.twist.vx << " m/s" << std::endl;
  
  // 访问最后一个点
  const auto& last_point = result.trajectory.back();
  std::cout << "End position: (" << last_point.pose.x << ", " 
            << last_point.pose.y << ")" << std::endl;
  std::cout << "End velocity: " << last_point.twist.vx << " m/s" << std::endl;
  
  return true;
}
```

### **2. 遍历轨迹**

```cpp
void processTrajectory(const navsim::plugin::PlanningResult& result) {
  for (size_t i = 0; i < result.trajectory.size(); ++i) {
    const auto& pt = result.trajectory[i];
    
    // 位置
    double x = pt.pose.x;
    double y = pt.pose.y;
    double yaw = pt.pose.yaw;
    
    // 速度
    double vx = pt.twist.vx;      // 线速度
    double vy = pt.twist.vy;      // 横向速度 (差速驱动通常为0)
    double omega = pt.twist.omega; // 角速度
    
    // 加速度
    double acc = pt.acceleration;
    
    // 曲率
    double kappa = pt.curvature;
    
    // 时间和路径长度
    double t = pt.time_from_start;
    double s = pt.path_length;
    
    std::cout << "Point " << i << ": "
              << "pos=(" << x << ", " << y << ", " << yaw << "), "
              << "vel=(" << vx << ", " << omega << "), "
              << "acc=" << acc << ", "
              << "κ=" << kappa << ", "
              << "t=" << t << ", "
              << "s=" << s << std::endl;
  }
}
```

### **3. 查找特定时间的轨迹点**

```cpp
const navsim::plugin::TrajectoryPoint* findPointAtTime(
    const navsim::plugin::PlanningResult& result, 
    double target_time) {
  
  for (const auto& pt : result.trajectory) {
    if (pt.time_from_start >= target_time) {
      return &pt;
    }
  }
  
  // 如果没找到，返回最后一个点
  if (!result.trajectory.empty()) {
    return &result.trajectory.back();
  }
  
  return nullptr;
}

// 使用示例
void example() {
  navsim::plugin::PlanningResult result;
  // ... 规划 ...
  
  // 查找 5 秒时的轨迹点
  const auto* pt = findPointAtTime(result, 5.0);
  if (pt) {
    std::cout << "At t=5s: pos=(" << pt->pose.x << ", " << pt->pose.y << "), "
              << "v=" << pt->twist.vx << " m/s" << std::endl;
  }
}
```

### **4. 计算轨迹统计信息**

```cpp
struct TrajectoryStats {
  double max_velocity = 0.0;
  double max_omega = 0.0;
  double max_acceleration = 0.0;
  double max_curvature = 0.0;
  double total_time = 0.0;
  double total_length = 0.0;
};

TrajectoryStats computeStats(const navsim::plugin::PlanningResult& result) {
  TrajectoryStats stats;
  
  if (result.trajectory.empty()) {
    return stats;
  }
  
  for (const auto& pt : result.trajectory) {
    stats.max_velocity = std::max(stats.max_velocity, std::abs(pt.twist.vx));
    stats.max_omega = std::max(stats.max_omega, std::abs(pt.twist.omega));
    stats.max_acceleration = std::max(stats.max_acceleration, std::abs(pt.acceleration));
    stats.max_curvature = std::max(stats.max_curvature, std::abs(pt.curvature));
  }
  
  stats.total_time = result.trajectory.back().time_from_start;
  stats.total_length = result.trajectory.back().path_length;
  
  return stats;
}

// 使用示例
void printStats(const navsim::plugin::PlanningResult& result) {
  auto stats = computeStats(result);
  
  std::cout << "Trajectory Statistics:" << std::endl;
  std::cout << "  Max velocity: " << stats.max_velocity << " m/s" << std::endl;
  std::cout << "  Max omega: " << stats.max_omega << " rad/s" << std::endl;
  std::cout << "  Max acceleration: " << stats.max_acceleration << " m/s²" << std::endl;
  std::cout << "  Max curvature: " << stats.max_curvature << " 1/m" << std::endl;
  std::cout << "  Total time: " << stats.total_time << " s" << std::endl;
  std::cout << "  Total length: " << stats.total_length << " m" << std::endl;
}
```

---

## 🎯 实际应用场景

### **1. 控制器输入**

```cpp
void Controller::followTrajectory(const navsim::plugin::PlanningResult& result) {
  double current_time = 0.0;
  
  for (const auto& pt : result.trajectory) {
    // 等待到达目标时间
    waitUntil(pt.time_from_start);
    
    // 使用速度和加速度进行前馈控制
    double feedforward_vel = pt.twist.vx;
    double feedforward_omega = pt.twist.omega;
    double feedforward_acc = pt.acceleration;
    
    // 计算控制输入
    double control_v = feedforward_vel + pid_vel.compute(error_v);
    double control_omega = feedforward_omega + pid_omega.compute(error_omega);
    
    // 发送控制命令
    sendCommand(control_v, control_omega);
  }
}
```

### **2. 碰撞检测**

```cpp
bool checkCollision(const navsim::plugin::PlanningResult& result,
                    const ObstacleMap& obstacles) {
  for (const auto& pt : result.trajectory) {
    // 使用位置和朝向检查碰撞
    if (obstacles.isCollision(pt.pose.x, pt.pose.y, pt.pose.yaw)) {
      std::cerr << "Collision detected at t=" << pt.time_from_start << std::endl;
      return true;
    }
  }
  return false;
}
```

### **3. 可视化**

```cpp
void visualizeTrajectory(const navsim::plugin::PlanningResult& result) {
  // 绘制路径
  for (const auto& pt : result.trajectory) {
    drawPoint(pt.pose.x, pt.pose.y);
  }
  
  // 绘制速度箭头
  for (size_t i = 0; i < result.trajectory.size(); i += 10) {
    const auto& pt = result.trajectory[i];
    double arrow_length = pt.twist.vx * 0.5;  // 速度越大箭头越长
    drawArrow(pt.pose.x, pt.pose.y, pt.pose.yaw, arrow_length);
  }
  
  // 绘制曲率热图
  for (const auto& pt : result.trajectory) {
    double color = std::abs(pt.curvature) / max_curvature;
    drawPointWithColor(pt.pose.x, pt.pose.y, color);
  }
}
```

---

## ✅ 数据来源

| 字段 | 来源 | 说明 |
|------|------|------|
| `pose.x, pose.y` | Simpson 积分 | 从速度积分得到位置 |
| `pose.yaw` | `final_traj.getPos(t)[0]` | MINCO 优化的朝向角 |
| `twist.vx` | `final_traj.getVel(t)[1]` | MINCO 优化的线速度 |
| `twist.omega` | `final_traj.getVel(t)[0]` | MINCO 优化的角速度 |
| `acceleration` | `final_traj.getAcc(t)[1]` | MINCO 优化的线加速度 |
| `curvature` | `omega / vx` | 从速度计算得到 |
| `time_from_start` | 累积时间 | 从起点开始的累积时间 |
| `path_length` | 累积长度 | 从起点开始的累积路径长度 |

---

## 📚 参考

- **数据结构定义**: `navsim-local/platform/include/plugin/data/planning_result.hpp`
- **提取实现**: `navsim-local/plugins/planning/jps_planner/adapter/jps_planner_plugin.cpp`
- **MINCO 轨迹**: `navsim-local/plugins/planning/jps_planner/algorithm/opt/trajectory.hpp`

---

## 🎓 总结

`PlanningResult` 现在包含了完整的动力学信息：

- ✅ **位置**: `pose.x, pose.y, pose.yaw`
- ✅ **速度**: `twist.vx, twist.vy, twist.omega`
- ✅ **加速度**: `acceleration`
- ✅ **曲率**: `curvature`
- ✅ **时间**: `time_from_start`
- ✅ **路径长度**: `path_length`

这些信息可以直接用于控制器、碰撞检测、可视化等应用场景。


# 前置处理层修复报告

**日期**: 2025-10-13  
**状态**: ✅ 完成  
**分支**: `feature/plugin-architecture-v2`

---

## 📋 任务概述

修复前置处理层的数据结构不匹配问题，使其与实际的 protobuf 定义和 `planning_context.hpp` 保持一致。

---

## ⚠️ 原始问题

### 问题描述

前置处理层的初始实现假设的数据结构与实际定义不匹配：

1. **BEVExtractor 问题**:
   - ❌ 假设 `proto::WorldTick` 有 `obstacles_size()` 和 `obstacles()` 方法
   - ✅ 实际使用 `dynamic_obstacles()` 字段
   - ❌ 假设 `BEVObstacles` 有 `positions`, `sizes`, `orientations` 等向量
   - ✅ 实际使用 `circles`, `rectangles`, `polygons` 向量

2. **DynamicObstaclePredictor 问题**:
   - ❌ 假设 `Pose2d` 有 `theta` 字段
   - ✅ 实际使用 `yaw` 字段
   - ❌ 假设 `DynamicObstacle` 有 `trajectory` 字段
   - ✅ 实际使用 `predicted_trajectories` 向量

3. **BasicDataConverter 问题**:
   - ❌ 假设 `EgoVehicle` 有 `velocity`, `acceleration` 字段
   - ✅ 实际使用 `twist`, `kinematics` 嵌套结构
   - ❌ 假设 `PlanningTask` 有 `goal`, `goal_heading` 字段
   - ✅ 实际使用 `goal_pose`, `tolerance` 结构

4. **命名空间问题**:
   - ❌ `PerceptionInput` 中使用 `proto::WorldTick*`
   - ✅ 应该使用 `navsim::proto::WorldTick*`

### 影响

- 前置处理层文件已创建但无法编译
- 暂时从 CMakeLists.txt 中移除
- 不影响核心插件系统框架的使用

---

## ✅ 解决方案

### 策略

参考原项目的 `perception_processor.cpp` 和 `perception_processor.hpp` 实现，确保与实际数据结构完全一致。

### 修复内容

#### 1. BEVExtractor 修复

**参考文件**: `src/perception_processor.cpp` (第 98-224 行)

**修复内容**:
- ✅ 使用 `world_tick.dynamic_obstacles()` 访问动态障碍物
- ✅ 使用 `world_tick.static_map()` 访问静态地图
- ✅ 输出到 `BEVObstacles` 的 `circles`, `rectangles`, `polygons` 向量
- ✅ 使用 `pose.yaw` 而非 `pose.theta`
- ✅ 添加静态地图缓存机制
- ✅ 添加检测范围过滤

**关键代码**:
```cpp
// 提取静态圆形障碍物
for (const auto& circle : static_map.circles()) {
  planning::BEVObstacles::Circle circle_obs;
  circle_obs.center.x = circle.x();
  circle_obs.center.y = circle.y();
  circle_obs.radius = circle.r();
  circle_obs.confidence = 1.0;
  obstacles.circles.push_back(circle_obs);
}

// 提取动态障碍物
for (const auto& dyn_obs : world_tick.dynamic_obstacles()) {
  if (dyn_obs.shape().has_rectangle()) {
    planning::BEVObstacles::Rectangle rect_obs;
    rect_obs.pose.yaw = dyn_obs.pose().yaw();  // 使用 yaw
    obstacles.rectangles.push_back(rect_obs);
  }
}
```

#### 2. DynamicObstaclePredictor 修复

**参考文件**: `src/perception_processor.cpp` (第 226-307 行)

**修复内容**:
- ✅ 使用 `world_tick.dynamic_obstacles()` 访问动态障碍物
- ✅ 创建 `predicted_trajectories` 向量
- ✅ 使用 `current_pose`, `current_twist` 字段
- ✅ 使用 `Pose2d.yaw` 而非 `theta`
- ✅ 实现恒定速度预测模型

**关键代码**:
```cpp
planning::DynamicObstacle pred_obs;
pred_obs.current_pose.yaw = dyn_obs.pose().yaw();  // 使用 yaw
pred_obs.current_twist.vx = dyn_obs.twist().vx();

// 生成预测轨迹
planning::DynamicObstacle::Trajectory trajectory;
for (int i = 0; i <= num_steps; ++i) {
  planning::Pose2d future_pose;
  future_pose.yaw = pred_obs.current_pose.yaw + pred_obs.current_twist.omega * t;
  trajectory.poses.push_back(future_pose);
  trajectory.timestamps.push_back(t);
}
pred_obs.predicted_trajectories.push_back(trajectory);
```

#### 3. BasicDataConverter 修复

**参考文件**: `src/perception_processor.cpp` (第 351-439 行)

**修复内容**:
- ✅ 使用 `world_tick.ego().pose()` 和 `world_tick.ego().twist()`
- ✅ 使用 `world_tick.goal().pose()` 和 `world_tick.goal().tol()`
- ✅ 使用 `ego.kinematics.wheelbase` 和 `ego.limits` 结构
- ✅ 使用 `task.goal_pose` 和 `task.tolerance` 结构
- ✅ 处理底盘配置（`world_tick.chassis()`）

**关键代码**:
```cpp
// 转换自车状态
ego.pose = {pose.x(), pose.y(), pose.yaw()};
ego.twist = {twist.vx(), twist.vy(), twist.omega()};
ego.kinematics.wheelbase = chassis.wheelbase();
ego.limits.max_velocity = limits.v_max();

// 转换任务
task.goal_pose = {goal.x(), goal.y(), goal.yaw()};
task.tolerance.position = tol.pos();
task.tolerance.yaw = tol.yaw();
```

#### 4. PreprocessingPipeline 修复

**修复内容**:
- ✅ 使用 `BasicDataConverter::convertEgo()` 静态方法
- ✅ 使用 `BasicDataConverter::convertTask()` 静态方法
- ✅ 使用 `world_tick.stamp()` 和 `world_tick.tick_id()`
- ✅ 修复 `raw_world_tick` 指针类型

#### 5. 命名空间修复

**修复内容**:
- ✅ 在 `perception_input.hpp` 中使用 `navsim::proto::WorldTick`
- ✅ 前向声明使用完整命名空间

---

## 📊 修复统计

### 修改的文件

| 文件 | 修改前行数 | 修改后行数 | 变化 |
|------|-----------|-----------|------|
| `include/perception/preprocessing.hpp` | 200 | 196 | -4 |
| `src/perception/bev_extractor.cpp` | 80 | 134 | +54 |
| `src/perception/dynamic_predictor.cpp` | 110 | 80 | -30 |
| `src/perception/basic_converter.cpp` | 150 | 81 | -69 |
| `src/perception/preprocessing_pipeline.cpp` | 70 | 70 | 0 |
| `include/plugin/perception_input.hpp` | 111 | 113 | +2 |
| `CMakeLists.txt` | - | - | 重新启用 |

### 代码质量

- ✅ 完全参考原项目实现
- ✅ 保持与现有代码的一致性
- ✅ 使用实际的 protobuf 字段名
- ✅ 使用实际的数据结构定义
- ✅ 添加详细的注释

---

## 🧪 编译测试

### 编译命令

```bash
cd navsim-local/build
make -j$(nproc)
```

### 编译结果

```
[ 11%] Built target navsim_proto
[ 71%] Built target ixwebsocket
[ 76%] Building CXX object CMakeFiles/navsim_plugin_system.dir/src/perception/bev_extractor.cpp.o
[ 77%] Building CXX object CMakeFiles/navsim_plugin_system.dir/src/perception/dynamic_predictor.cpp.o
[ 80%] Building CXX object CMakeFiles/navsim_plugin_system.dir/src/perception/basic_converter.cpp.o
[ 82%] Building CXX object CMakeFiles/navsim_plugin_system.dir/src/perception/preprocessing_pipeline.cpp.o
[ 84%] Linking CXX static library libnavsim_plugin_system.a
[ 84%] Built target navsim_plugin_system
[100%] Built target navsim_algo
```

**结果**: ✅ 编译成功，无错误，无警告

---

## 📝 关键经验

### 1. 参考原项目代码

- ✅ 原项目的 `perception_processor.cpp` 是可靠的参考
- ✅ 直接复制核心逻辑，确保兼容性
- ✅ 保持数据结构和字段名的一致性

### 2. 理解实际数据结构

- ✅ 查看 `world_tick.proto` 了解 protobuf 定义
- ✅ 查看 `planning_context.hpp` 了解数据结构
- ✅ 不要假设字段名，要实际验证

### 3. 命名空间问题

- ✅ 注意 `proto::WorldTick` vs `navsim::proto::WorldTick`
- ✅ 使用完整命名空间避免歧义
- ✅ 前向声明要包含完整命名空间

### 4. 增量修复

- ✅ 一个文件一个文件修复
- ✅ 每次修复后尝试编译
- ✅ 及时发现和解决问题

---

## ✅ 验证清单

- [x] BEVExtractor 使用正确的 protobuf 字段
- [x] DynamicObstaclePredictor 使用正确的数据结构
- [x] BasicDataConverter 使用正确的转换逻辑
- [x] PreprocessingPipeline 正确整合所有组件
- [x] 命名空间正确
- [x] 编译成功
- [x] 无编译警告
- [x] 代码风格一致
- [x] 注释清晰

---

## 🎯 总结

**前置处理层修复完成！**

- ✅ 所有数据结构与实际定义一致
- ✅ 参考原项目实现，确保兼容性
- ✅ 编译成功，无错误无警告
- ✅ Phase 1 基础架构 100% 完成

**下一步**: 开始 Phase 2 - 实现具体插件

---

**修复完成时间**: 约 30 分钟  
**修复质量**: 高（参考原项目实现）  
**编译状态**: ✅ 成功


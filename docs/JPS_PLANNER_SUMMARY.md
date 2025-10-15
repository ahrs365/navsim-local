# JPS 规划器梳理总结

## 📊 核心发现

### 你的问题很关键！

**问题**：为什么创建 JPSGridAdapter，JPSPlanner 这个类也需要移植吧？

**答案**：完全正确！我之前的分析不够完整。

---

## 🏗️ 完整架构梳理

### 原始 ROS 版本的三层架构

```
┌─────────────────────────────────────────┐
│  JPSPlanner (高层封装)                   │
│  ├─ 坐标转换                             │
│  ├─ 调用 GraphSearch                     │
│  ├─ 路径优化 (removeCornerPts)           │
│  ├─ 轨迹生成 (getSampleTraj)             │
│  ├─ 时间规划 (getTrajsWithTime)          │
│  └─ 碰撞检测 (checkLineCollision)        │
└────────────┬────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────┐
│  GraphSearch (核心算法)                  │
│  ├─ JPS 主循环                           │
│  ├─ jump() 跳点搜索                      │
│  ├─ getJpsSucc() 获取后继                │
│  └─ hasForced() 强制邻居                 │
└────────────┬────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────┐
│  SDFmap (地图接口)                       │
│  ├─ coord2gridIndex()                    │
│  ├─ gridIndex2coordd()                   │
│  ├─ isOccupied()                         │
│  └─ getDistanceReal()                    │
└─────────────────────────────────────────┘
```

### NavSim 适配版本的四层架构

```
┌─────────────────────────────────────────┐
│  JPSPlannerPlugin (插件接口)             │
│  ├─ getMetadata()                        │
│  ├─ initialize()                         │
│  ├─ plan() - 对接 PlanningContext        │
│  └─ convertToTrajectory()                │
└────────────┬────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────┐
│  JPSPlannerCore (核心逻辑 - 移植)        │
│  ├─ search() - 执行搜索                  │
│  ├─ optimizePath() - 路径优化            │
│  ├─ generateSampleTrajectory() - 轨迹    │
│  ├─ addTimeProfile() - 时间规划          │
│  ├─ checkLineCollision() - 碰撞检测      │
│  └─ getGridsBetweenPoints2D() - 工具     │
└────────────┬────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────┐
│  JPSGraphSearch (JPS 算法 - 移植)        │
│  ├─ plan() - JPS 主循环                  │
│  ├─ jump() - 跳点搜索                    │
│  ├─ getJpsSucc() - 获取后继              │
│  └─ hasForced() - 强制邻居               │
└────────────┬────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────┐
│  JPSGridAdapter (地图适配器 - 新建)      │
│  ├─ worldToGrid()                        │
│  ├─ gridToWorld()                        │
│  ├─ isOccupied()                         │
│  └─ isFreeWithSafeDis()                  │
└────────────┬────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────┐
│  OccupancyGrid (NavSim 数据结构)         │
└─────────────────────────────────────────┘
```

---

## 🔍 为什么需要移植 JPSPlanner？

### JPSPlanner 不仅仅是"调用 GraphSearch"

**JPSPlanner 的核心价值**：

| 功能模块 | 代码行数 | 重要性 | 说明 |
|---------|---------|--------|------|
| **路径优化** | ~100 行 | 🔥 高 | `removeCornerPts()` - 去除冗余拐角点 |
| **碰撞检测** | ~50 行 | 🔥 高 | `checkLineCollision()` - 直线碰撞检测 |
| **Bresenham** | ~30 行 | 🔥 高 | `getGridsBetweenPoints2D()` - 直线栅格化 |
| **轨迹生成** | ~150 行 | 🔶 中 | `getSampleTraj()` - 生成 5D 轨迹 |
| **时间规划** | ~200 行 | 🔶 中 | `getTrajsWithTime()` - 梯形速度曲线 |
| **速度曲线** | ~50 行 | 🔶 中 | `evaluateDuration()` 等 |
| **角度归一化** | ~10 行 | 🔷 低 | `normalizeAngle()` |

**总计**：约 **590 行**有价值的代码需要移植！

### 具体例子：路径优化

**原始路径**（JPS 输出）：
```
A → B → C → D → E → F → G
```

**优化后路径**（removeCornerPts）：
```
A → C → E → G
```

**优化逻辑**：
1. 检查 A → C 是否无碰撞
2. 如果是，跳过 B
3. 继续检查 A → D，A → E...
4. 直到遇到碰撞，保留前一个点

**为什么重要**？
- ✅ 减少路径点数量（提高效率）
- ✅ 路径更平滑（减少转向）
- ✅ 减少计算量（后续轨迹生成）

### 具体例子：轨迹生成

**输入**：优化后的路径点 `[(x1, y1), (x2, y2), (x3, y3)]`

**输出**：5D 轨迹 `[(x, y, theta, dtheta, ds)]`

**生成逻辑**：
```
对于每个路径点 i：
  1. 到达点 i（保持前一个航向）
     state = (x_i, y_i, theta_prev, 0, distance)
  
  2. 转向（计算新航向）
     theta_new = atan2(y_{i+1} - y_i, x_{i+1} - x_i)
     state = (x_i, y_i, theta_new, dtheta, 0)
```

**为什么重要**？
- ✅ 考虑车辆航向（不是简单的路径点）
- ✅ 分离平移和旋转（更符合车辆运动学）
- ✅ 为时间规划提供基础

### 具体例子：时间规划

**输入**：5D 轨迹 + 速度/加速度限制

**输出**：带时间戳的轨迹 `[(x, y, theta, t)]`

**规划逻辑**：
1. 计算总路径长度（加权：距离 + 航向变化）
2. 使用梯形速度曲线计算总时间
3. 均匀采样时间点
4. 对每个时间点，插值计算位置

**为什么重要**？
- ✅ 考虑速度/加速度限制
- ✅ 生成可执行的轨迹（不仅仅是路径）
- ✅ 符合车辆动力学约束

---

## 🎯 为什么需要 JPSGridAdapter？

### SDFmap 不仅仅是数据

**SDFmap 提供的功能**：

| 功能 | 说明 | NavSim 对应 |
|------|------|------------|
| `coord2gridIndex()` | 世界坐标 → 栅格坐标 | ❌ 没有 |
| `gridIndex2coordd()` | 栅格坐标 → 世界坐标 | ❌ 没有 |
| `isOccupied()` | 是否占据 | ❌ 没有（只有数据） |
| `isOccWithSafeDis()` | 带安全距离检测 | ❌ 没有 |
| `getDistanceReal()` | 距离场值 | ❌ 没有 |

**NavSim 的 OccupancyGrid**：
```cpp
struct OccupancyGrid {
  int width, height;
  double resolution;
  Eigen::Vector2d origin;
  std::vector<uint8_t> data;  // 仅仅是数据！
};
```

**所以需要 JPSGridAdapter**：
```cpp
class JPSGridAdapter {
  // 封装 OccupancyGrid
  const OccupancyGrid* grid_;
  
  // 提供 SDFmap 的接口
  Eigen::Vector2i worldToGrid(const Eigen::Vector2d& world);
  Eigen::Vector2d gridToWorld(const Eigen::Vector2i& grid);
  bool isOccupied(int x, int y);
  bool isFreeWithSafeDis(int x, int y);
};
```

---

## 📦 完整的移植清单

### 需要创建的类

| 类名 | 对应原始类 | 职责 | 代码量估计 |
|------|-----------|------|-----------|
| `JPSPlannerPlugin` | - | 插件接口 | ~200 行 |
| `JPSPlannerCore` | `JPSPlanner` | 核心逻辑 | ~600 行 |
| `JPSGraphSearch` | `GraphSearch` | JPS 算法 | ~500 行 |
| `JPSGridAdapter` | `SDFmap` | 地图适配 | ~150 行 |
| `JPS2DNeib` | `JPS2DNeib` | 邻居查找 | ~100 行 |
| **总计** | - | - | **~1550 行** |

### 需要移植的功能

#### 高优先级（必须）

- [x] `GraphSearch::plan()` - JPS 主循环
- [x] `GraphSearch::jump()` - 跳点搜索
- [x] `GraphSearch::getJpsSucc()` - 获取后继
- [x] `JPSPlanner::removeCornerPts()` - 路径优化
- [x] `JPSPlanner::checkLineCollision()` - 碰撞检测
- [x] `JPSPlanner::getGridsBetweenPoints2D()` - Bresenham

#### 中优先级（推荐）

- [ ] `JPSPlanner::getSampleTraj()` - 轨迹生成
- [ ] `JPSPlanner::getTrajsWithTime()` - 时间规划
- [ ] `JPSPlanner::evaluateDuration()` - 速度曲线

#### 低优先级（可选）

- [ ] `JPSPlanner::normalizeAngle()` - 角度归一化
- [ ] `JPSPlanner::getKinoNodeWithStartPath()` - 特定应用

### 需要移除的功能

- ❌ `pubPath()` - ROS 发布器
- ❌ `ros::NodeHandle` - ROS 依赖
- ❌ `nav_msgs::Path` - ROS 消息
- ❌ `visualization_msgs::Marker` - ROS 可视化

---

## 🚀 实施建议

### 分阶段实施

**阶段 1：基础搜索**（3-4 天）
- 创建 `JPSGridAdapter`
- 移植 `JPSGraphSearch`
- 实现基本的 `JPSPlannerPlugin`
- 测试基本搜索功能

**阶段 2：路径优化**（2-3 天）
- 移植 `removeCornerPts()`
- 移植 `checkLineCollision()`
- 移植 `getGridsBetweenPoints2D()`
- 测试路径优化效果

**阶段 3：轨迹生成**（2-3 天）
- 移植 `getSampleTraj()`
- 移植 `getTrajsWithTime()`
- 移植速度曲线计算
- 测试完整轨迹生成

**总工作量**：7-10 天

### 测试策略

1. **单元测试**：
   - `JPSGridAdapter` 坐标转换
   - `JPSGraphSearch` 基本搜索
   - `removeCornerPts` 路径优化

2. **集成测试**：
   - 简单场景（无障碍物）
   - 复杂场景（多障碍物）
   - 边界情况

3. **性能测试**：
   - 与 A* 对比速度
   - 大地图测试
   - 路径质量对比

---

## 📚 参考文档

1. **`JPS_PLANNER_ANALYSIS.md`** - 详细分析（300 行）
   - JPS 算法原理
   - 代码结构分析
   - 依赖关系分析

2. **`JPS_COMPLETE_ADAPTATION_PLAN.md`** - 完整方案（300 行）
   - 架构重新梳理
   - 完整的类设计
   - 数据流分析

3. **`JPS_ADAPTATION_GUIDE.md`** - 快速指南（300 行）
   - 代码模板
   - 实现步骤
   - CMake 配置

4. **可视化流程图**
   - JPS 算法核心流程
   - 架构对比图
   - 类关系图

---

## 💡 关键要点

### 1. 这不是简单的"包装"

❌ **错误理解**：
```
只需要调用 GraphSearch，然后包装成插件
```

✅ **正确理解**：
```
需要移植整个 JPSPlanner 的核心逻辑：
- GraphSearch（JPS 算法）
- 路径优化（removeCornerPts）
- 轨迹生成（getSampleTraj）
- 时间规划（getTrajsWithTime）
- 碰撞检测（checkLineCollision）
```

### 2. JPSPlanner 的价值

**JPSPlanner 包含**：
- ✅ 成熟的路径优化算法
- ✅ 完善的轨迹生成逻辑
- ✅ 梯形速度曲线规划
- ✅ 考虑车辆航向的轨迹

**这些都需要移植！**

### 3. 为什么需要 JPSGridAdapter

因为 `SDFmap` 提供的是**接口**，而 `OccupancyGrid` 只是**数据**。

需要创建适配器来提供：
- 坐标转换
- 碰撞检测
- 安全距离检测

---

## 🎯 下一步行动

1. ✅ 阅读 `JPS_COMPLETE_ADAPTATION_PLAN.md` - 理解完整架构
2. ✅ 阅读 `JPS_ADAPTATION_GUIDE.md` - 查看代码模板
3. ⏭️ 创建 `JPSGridAdapter` - 先解决地图适配
4. ⏭️ 移植 `JPSGraphSearch` - 保留核心算法
5. ⏭️ 移植 `JPSPlannerCore` - 移植核心逻辑
6. ⏭️ 实现 `JPSPlannerPlugin` - 完成插件封装

**记住**：这是一个完整的移植项目，不是简单的包装！


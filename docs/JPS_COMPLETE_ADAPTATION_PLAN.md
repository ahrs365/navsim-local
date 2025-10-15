# JPS 规划器完整适配方案

## 📋 架构重新梳理

你说得对！我之前的分析不够完整。让我重新梳理整个架构。

### 原始 ROS 版本的完整架构

```
┌─────────────────────────────────────────────────────────┐
│                    ROS Node                             │
└────────────────────┬────────────────────────────────────┘
                     │
                     ↓
┌─────────────────────────────────────────────────────────┐
│                  JPSPlanner                             │
│  (高层封装 - 负责整体流程)                                │
├─────────────────────────────────────────────────────────┤
│  • plan() - 主规划入口                                   │
│  • removeCornerPts() - 路径优化                          │
│  • getSampleTraj() - 生成采样轨迹                        │
│  • getTrajsWithTime() - 添加时间信息                     │
│  • checkLineCollision() - 碰撞检测                       │
│  • getGridsBetweenPoints2D() - Bresenham 直线算法        │
│  • evaluateDuration() - 梯形速度曲线时间计算              │
│  • normalizeAngle() - 角度归一化                         │
└────────────┬────────────────────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────────────────────┐
│                GraphSearch                              │
│  (核心搜索引擎 - JPS 算法实现)                            │
├─────────────────────────────────────────────────────────┤
│  • plan() - JPS 主循环                                   │
│  • jump() - 跳点搜索                                     │
│  • getJpsSucc() - 获取 JPS 后继节点                      │
│  • hasForced() - 检查强制邻居                            │
│  • getPath() - 返回路径                                  │
└────────────┬────────────────────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────────────────────┐
│                   SDFmap                                │
│  (地图接口 - 提供障碍物查询)                              │
├─────────────────────────────────────────────────────────┤
│  • coord2gridIndex() - 世界坐标 → 栅格坐标                │
│  • gridIndex2coordd() - 栅格坐标 → 世界坐标               │
│  • isOccupied() - 是否占据                               │
│  • isOccWithSafeDis() - 带安全距离的占据检测              │
│  • getDistanceReal() - 获取距离场值                       │
└─────────────────────────────────────────────────────────┘
```

### 关键发现

**JPSPlanner 的职责**：
1. ✅ **坐标转换**：世界坐标 ↔ 栅格坐标
2. ✅ **调用 GraphSearch**：执行 JPS 搜索
3. ✅ **路径优化**：`removeCornerPts()` - 去除冗余拐角点
4. ✅ **轨迹生成**：`getSampleTraj()` - 生成带航向的轨迹
5. ✅ **时间规划**：`getTrajsWithTime()` - 梯形速度曲线
6. ✅ **碰撞检测**：`checkLineCollision()` - 用于路径优化
7. ✅ **工具函数**：Bresenham 直线、角度归一化等

**为什么需要 JPSGridAdapter？**

因为 `SDFmap` 不仅仅是一个简单的栅格地图，它提供了：
- 坐标转换功能
- 距离场查询（SDF）
- 带安全距离的碰撞检测

而 NavSim 的 `OccupancyGrid` 只是一个数据结构，没有这些方法。所以我们需要：

1. **JPSGridAdapter** - 封装 `OccupancyGrid`，提供 `SDFmap` 的接口
2. **JPSPlanner 的核心逻辑** - 移植到插件中
3. **GraphSearch** - 保留核心算法，适配新的地图接口

---

## 🏗️ 完整适配架构

### NavSim 适配版本架构

```
┌─────────────────────────────────────────────────────────┐
│            PlannerPluginManager                         │
│            (NavSim 插件管理器)                           │
└────────────────────┬────────────────────────────────────┘
                     │
                     ↓
┌─────────────────────────────────────────────────────────┐
│              JPSPlannerPlugin                           │
│  (插件接口实现 - 对接 NavSim)                             │
├─────────────────────────────────────────────────────────┤
│  • getMetadata() - 插件元数据                            │
│  • initialize() - 初始化配置                             │
│  • plan() - 主规划入口（对接 PlanningContext）           │
│  • isAvailable() - 检查可用性                            │
└────────────┬────────────────────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────────────────────┐
│                JPSPlannerCore                           │
│  (移植 JPSPlanner 的核心逻辑)                            │
├─────────────────────────────────────────────────────────┤
│  • search() - 执行 JPS 搜索                              │
│  • optimizePath() - 路径优化                             │
│  • generateTrajectory() - 生成轨迹                       │
│  • addTimeProfile() - 添加时间信息                       │
│  • checkLineCollision() - 碰撞检测                       │
│  • getGridsBetweenPoints2D() - Bresenham 算法            │
└────────────┬────────────────────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────────────────────┐
│              JPSGraphSearch                             │
│  (移植 GraphSearch - JPS 核心算法)                       │
├─────────────────────────────────────────────────────────┤
│  • plan() - JPS 主循环                                   │
│  • jump() - 跳点搜索                                     │
│  • getJpsSucc() - 获取后继节点                           │
│  • hasForced() - 检查强制邻居                            │
│  • getPath() - 返回路径                                  │
└────────────┬────────────────────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────────────────────┐
│              JPSGridAdapter                             │
│  (适配器 - 封装 OccupancyGrid)                           │
├─────────────────────────────────────────────────────────┤
│  • worldToGrid() - 世界坐标 → 栅格坐标                    │
│  • gridToWorld() - 栅格坐标 → 世界坐标                    │
│  • isOccupied() - 是否占据                               │
│  • isFreeWithSafeDis() - 带安全距离检测                   │
│  • coordToId() - 坐标转 ID                               │
└────────────┬────────────────────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────────────────────┐
│              OccupancyGrid                              │
│  (NavSim 栅格地图数据结构)                               │
└─────────────────────────────────────────────────────────┘
```

---

## 📦 完整的类设计

### 1. JPSGridAdapter（地图适配器）

**职责**：封装 `OccupancyGrid`，提供与 `SDFmap` 兼容的接口

```cpp
class JPSGridAdapter {
public:
  JPSGridAdapter(const OccupancyGrid* grid, double safe_dis);
  
  // 地图尺寸
  int getWidth() const;
  int getHeight() const;
  double getResolution() const;
  
  // 坐标转换（对应 SDFmap 的接口）
  Eigen::Vector2i worldToGrid(const Eigen::Vector2d& world) const;
  Eigen::Vector2d gridToWorld(const Eigen::Vector2i& grid) const;
  int coordToId(int x, int y) const;
  
  // 碰撞检测（对应 SDFmap 的接口）
  bool isOccupied(int x, int y) const;
  bool isFree(int x, int y) const;
  bool isFreeWithSafeDis(int x, int y) const;  // 对应 isOccWithSafeDis
  bool isInBounds(int x, int y) const;
  
private:
  const OccupancyGrid* grid_;
  int width_, height_;
  double resolution_;
  Eigen::Vector2d origin_;
  double safe_dis_;
  int safe_dis_cells_;
};
```

### 2. JPSGraphSearch（核心搜索算法）

**职责**：实现 JPS 算法，移植自原始 `GraphSearch`

```cpp
class JPSGraphSearch {
public:
  JPSGraphSearch(std::shared_ptr<JPSGridAdapter> grid_adapter);
  
  // 主搜索接口
  bool plan(int xStart, int yStart, int xGoal, int yGoal, 
            bool useJps = true, int maxExpand = -1);
  
  // 获取结果
  std::vector<StatePtr> getPath() const;
  std::vector<StatePtr> getOpenSet() const;
  std::vector<StatePtr> getCloseSet() const;
  
private:
  // 核心算法（保留原始实现）
  bool jump(int x, int y, int dx, int dy, int& new_x, int& new_y);
  void getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, 
                  std::vector<double>& succ_costs);
  bool hasForced(int x, int y, int dx, int dy);
  
  // 辅助方法
  bool isFree(int x, int y) const;
  double getHeur(int x, int y) const;
  int coordToId(int x, int y) const;
  
  std::shared_ptr<JPSGridAdapter> grid_adapter_;
  priorityQueue pq_;
  std::vector<StatePtr> hm_;
  std::vector<bool> seen_;
  std::shared_ptr<JPS2DNeib> jn2d_;
  // ... 其他成员变量
};
```

### 3. JPSPlannerCore（规划器核心逻辑）

**职责**：移植 `JPSPlanner` 的核心功能，去除 ROS 依赖

```cpp
class JPSPlannerCore {
public:
  struct Config {
    double safe_dis = 0.3;
    double max_jps_dis = 100.0;
    double distance_weight = 1.0;
    double yaw_weight = 1.0;
    double traj_cut_length = 10.0;
    double max_vel = 2.0;
    double max_acc = 1.0;
    double max_omega = 1.0;
    double time_step = 0.1;
  };
  
  JPSPlannerCore(const Config& config);
  
  // 主搜索接口
  bool search(const Eigen::Vector3d& start, 
              const Eigen::Vector3d& goal,
              std::shared_ptr<JPSGridAdapter> grid_adapter,
              std::vector<Eigen::Vector2d>& raw_path);
  
  // 路径优化（移植自 removeCornerPts）
  std::vector<Eigen::Vector2d> optimizePath(
      const std::vector<Eigen::Vector2d>& path,
      std::shared_ptr<JPSGridAdapter> grid_adapter);
  
  // 轨迹生成（移植自 getSampleTraj）
  std::vector<Eigen::VectorXd> generateSampleTrajectory(
      const std::vector<Eigen::Vector2d>& path,
      const Eigen::Vector3d& start_state,
      const Eigen::Vector3d& end_state);
  
  // 时间规划（移植自 getTrajsWithTime）
  FlatTrajData addTimeProfile(
      const std::vector<Eigen::VectorXd>& sample_trajs,
      const Eigen::Vector3d& current_state_VAJ,
      const Eigen::Vector3d& current_state_OAJ);
  
  // 碰撞检测（移植自 checkLineCollision）
  bool checkLineCollision(const Eigen::Vector2d& start, 
                         const Eigen::Vector2d& end,
                         std::shared_ptr<JPSGridAdapter> grid_adapter);
  
  // Bresenham 直线算法（移植自 getGridsBetweenPoints2D）
  std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(
      const Eigen::Vector2i& start, 
      const Eigen::Vector2i& end);
  
private:
  // 辅助方法
  void normalizeAngle(const double& ref_angle, double& angle);
  double evaluateDuration(const double& length, const double& startV, 
                         const double& endV, const double& maxV, 
                         const double& maxA);
  double evaluateLength(const double& curt, const double& locallength, 
                       const double& localtime, const double& startV, 
                       const double& endV, const double& maxV, 
                       const double& maxA);
  
  Config config_;
  std::unique_ptr<JPSGraphSearch> graph_search_;
};
```

### 4. JPSPlannerPlugin（插件接口）

**职责**：实现 NavSim 插件接口，对接整个系统

```cpp
class JPSPlannerPlugin : public plugin::PlannerPluginInterface {
public:
  JPSPlannerPlugin();
  
  // 必须实现的接口
  plugin::PlannerPluginMetadata getMetadata() const override;
  bool initialize(const nlohmann::json& config) override;
  bool plan(const planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           plugin::PlanningResult& result) override;
  std::pair<bool, std::string> isAvailable(
      const planning::PlanningContext& context) const override;
  
  // 可选接口
  void reset() override;
  nlohmann::json getStatistics() const override;

private:
  // 辅助方法
  std::vector<plugin::TrajectoryPoint> convertToTrajectory(
      const FlatTrajData& flat_traj);
  
  JPSPlannerCore::Config config_;
  std::unique_ptr<JPSPlannerCore> planner_core_;
  
  struct Statistics {
    size_t total_plans = 0;
    size_t successful_plans = 0;
    double avg_time_ms = 0.0;
  };
  Statistics stats_;
};
```

---

## 🔄 数据流

### 完整的规划流程

```
1. PlanningContext (NavSim)
   ├─ ego.pose (起点)
   ├─ task.goal_pose (终点)
   └─ occupancy_grid (栅格地图)
   
2. JPSPlannerPlugin::plan()
   ├─ 创建 JPSGridAdapter(occupancy_grid)
   └─ 调用 planner_core_->search()
   
3. JPSPlannerCore::search()
   ├─ 坐标转换：world → grid
   ├─ 调用 graph_search_->plan()
   ├─ 获取原始路径
   └─ 返回 raw_path
   
4. JPSPlannerCore::optimizePath()
   ├─ removeCornerPts 逻辑
   ├─ checkLineCollision 检测
   └─ 返回优化后路径
   
5. JPSPlannerCore::generateSampleTrajectory()
   ├─ 为每个路径点生成 5D 状态 (x, y, theta, dtheta, ds)
   └─ 返回采样轨迹
   
6. JPSPlannerCore::addTimeProfile()
   ├─ 梯形速度曲线计算
   ├─ 时间插值
   └─ 返回 FlatTrajData
   
7. JPSPlannerPlugin::convertToTrajectory()
   ├─ FlatTrajData → vector<TrajectoryPoint>
   └─ 填充 PlanningResult
   
8. PlanningResult (NavSim)
   ├─ trajectory (轨迹点)
   ├─ success (是否成功)
   └─ computation_time_ms (计算时间)
```

---

## 📝 移植清单

### 需要移植的功能模块

| 模块 | 原始类 | 目标类 | 优先级 |
|------|--------|--------|--------|
| JPS 核心算法 | `GraphSearch` | `JPSGraphSearch` | 🔥 高 |
| 地图适配器 | `SDFmap` | `JPSGridAdapter` | 🔥 高 |
| 路径优化 | `JPSPlanner::removeCornerPts` | `JPSPlannerCore::optimizePath` | 🔥 高 |
| 碰撞检测 | `JPSPlanner::checkLineCollision` | `JPSPlannerCore::checkLineCollision` | 🔥 高 |
| Bresenham | `JPSPlanner::getGridsBetweenPoints2D` | `JPSPlannerCore::getGridsBetweenPoints2D` | 🔥 高 |
| 轨迹生成 | `JPSPlanner::getSampleTraj` | `JPSPlannerCore::generateSampleTrajectory` | 🔶 中 |
| 时间规划 | `JPSPlanner::getTrajsWithTime` | `JPSPlannerCore::addTimeProfile` | 🔶 中 |
| 速度曲线 | `JPSPlanner::evaluateDuration` | `JPSPlannerCore::evaluateDuration` | 🔶 中 |
| 角度归一化 | `JPSPlanner::normalizeAngle` | `JPSPlannerCore::normalizeAngle` | 🔷 低 |

### 需要移除的功能

| 功能 | 原因 |
|------|------|
| `pubPath()` | ROS 发布器，NavSim 不需要 |
| `ros::NodeHandle` | ROS 依赖 |
| `nav_msgs::Path` | ROS 消息类型 |
| `visualization_msgs::Marker` | ROS 可视化 |
| `getKinoNodeWithStartPath()` | 特定应用逻辑，可选 |

---

## 🎯 总结

### 为什么需要这样的架构？

1. **JPSGridAdapter** - 因为 `SDFmap` 提供的不仅是数据，还有方法
2. **JPSPlannerCore** - 因为 `JPSPlanner` 包含大量有价值的逻辑（路径优化、轨迹生成）
3. **JPSGraphSearch** - 保留核心 JPS 算法
4. **JPSPlannerPlugin** - 对接 NavSim 插件系统

### 完整的移植不是简单的"包装"

原始代码的价值在于：
- ✅ 成熟的 JPS 实现
- ✅ 完善的路径优化
- ✅ 轨迹生成和时间规划
- ✅ 梯形速度曲线

这些都需要移植，而不仅仅是调用 `GraphSearch`！

### 下一步

参考 `JPS_ADAPTATION_GUIDE.md` 中的代码模板，逐步实现这四个类。


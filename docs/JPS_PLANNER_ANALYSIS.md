# JPS Planner 规划器详细分析

## 📋 目录

1. [概述](#1-概述)
2. [代码结构分析](#2-代码结构分析)
3. [核心算法解析](#3-核心算法解析)
4. [依赖关系分析](#4-依赖关系分析)
5. [适配到 NavSim-Local 的方案](#5-适配到-navsim-local-的方案)
6. [改造步骤](#6-改造步骤)

---

## 1. 概述

### 1.1 JPS 算法简介

**JPS (Jump Point Search)** 是 A* 算法的优化版本，主要用于栅格地图上的路径规划。

**核心优势**：
- ⚡ **速度快**：通过"跳点"机制减少搜索节点数量，比 A* 快 10-100 倍
- 🎯 **最优性**：保证找到最优路径（与 A* 相同）
- 💾 **内存效率**：减少 Open Set 中的节点数量

**适用场景**：
- 2D/3D 栅格地图路径规划
- 静态环境（障碍物不变）
- 需要快速全局路径规划

### 1.2 当前代码来源

这个 JPS 规划器来自 **ROS 生态系统**，具有以下特征：

| 特征 | 说明 |
|------|------|
| **框架** | ROS (Robot Operating System) |
| **依赖** | `ros::NodeHandle`, `nav_msgs::Path`, `visualization_msgs::Marker` |
| **地图表示** | `SDFmap` (Signed Distance Field Map) |
| **输入/输出** | ROS 话题发布/订阅 |
| **配置方式** | ROS 参数服务器 |

---

## 2. 代码结构分析

### 2.1 文件组织

```
jps_planner/
├── include/
│   ├── graph_search.h       # JPS 核心搜索算法
│   ├── jps_planner.h        # JPS 规划器封装
│   └── traj_representation.h # 轨迹表示数据结构
└── src/
    ├── graph_search.cpp     # 搜索算法实现
    └── jps_planner.cpp      # 规划器实现
```

### 2.2 核心类分析

#### 2.2.1 `GraphSearch` 类（核心搜索引擎）

**职责**：实现 JPS 和 A* 搜索算法

**关键成员**：
```cpp
class GraphSearch {
private:
  std::shared_ptr<SDFmap> map_;        // SDF 地图
  int xDim_, yDim_, zDim_;             // 地图尺寸
  double eps_;                          // 启发式权重
  double safe_dis_;                     // 安全距离
  
  priorityQueue pq_;                    // 优先队列（Open Set）
  std::vector<StatePtr> hm_;            // 哈希表（所有节点）
  std::vector<bool> seen_;              // 访问标记
  
  std::shared_ptr<JPS2DNeib> jn2d_;    // 2D JPS 邻居查找
  std::shared_ptr<JPS3DNeib> jn3d_;    // 3D JPS 邻居查找

public:
  // 2D 规划
  bool plan(int xStart, int yStart, int xGoal, int yGoal, 
            bool useJps, int maxExpand = -1);
  
  // 获取路径
  std::vector<StatePtr> getPath() const;
};
```

**核心方法**：
- `plan()` - 主规划循环
- `getSucc()` - A* 获取后继节点
- `getJpsSucc()` - JPS 获取跳点
- `jump()` - JPS 跳跃函数（核心）
- `hasForced()` - 检查强制邻居

#### 2.2.2 `JPSPlanner` 类（规划器封装）

**职责**：封装 JPS 搜索，提供路径优化和轨迹生成

**关键成员**：
```cpp
class JPSPlanner {
private:
  // 参数
  double safe_dis_;              // 安全距离
  double max_jps_dis_;           // 最大搜索距离
  double distance_weight_;       // 距离权重
  double yaw_weight_;            // 航向权重
  double trajCutLength_;         // 轨迹截断长度
  double max_vel_, max_acc_;     // 速度/加速度限制
  double max_omega_, max_domega_; // 角速度/角加速度限制
  
  // 数据
  Eigen::Vector3d start_state_;  // 起点 (x, y, yaw)
  Eigen::Vector3d end_state_;    // 终点 (x, y, yaw)
  std::vector<Eigen::Vector2d> raw_path_;  // 原始路径
  std::vector<Eigen::Vector2d> path_;      // 优化后路径
  
  // ROS 相关
  ros::NodeHandle nh_;
  std::shared_ptr<SDFmap> map_util_;
  std::shared_ptr<GraphSearch> graph_search_;
  ros::Publisher path_pub_;

public:
  bool plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);
  std::vector<Eigen::Vector2d> removeCornerPts(const std::vector<Eigen::Vector2d> &path);
  void getSampleTraj();
  void getTrajsWithTime();
};
```

**核心方法**：
- `plan()` - 执行 JPS 搜索
- `removeCornerPts()` - 路径平滑（去除拐角点）
- `checkLineCollision()` - 直线碰撞检测
- `getSampleTraj()` - 生成采样轨迹
- `getTrajsWithTime()` - 添加时间信息

#### 2.2.3 数据结构

**State（搜索节点）**：
```cpp
struct State {
  int id;                    // 节点 ID
  int x, y, z;               // 栅格坐标
  int dx, dy, dz;            // 搜索方向
  int parentId;              // 父节点 ID
  double g;                  // 实际代价
  double h;                  // 启发式代价
  bool opened, closed;       // 状态标记
  priorityQueue::handle_type heapkey;  // 堆句柄
};
```

**FlatTrajData（轨迹数据）**：
```cpp
struct FlatTrajData {
  std::vector<Eigen::Vector3d> UnOccupied_traj_pts;  // 轨迹点 (yaw, s, t)
  double UnOccupied_initT;                           // 初始时间
  std::vector<Eigen::Vector3d> UnOccupied_positions; // 位置点
  Eigen::MatrixXd start_state;                       // 起点状态 (pva)
  Eigen::MatrixXd final_state;                       // 终点状态
  Eigen::Vector3d start_state_XYTheta;               // 起点 (x, y, theta)
  Eigen::Vector3d final_state_XYTheta;               // 终点 (x, y, theta)
  bool if_cut;                                       // 是否截断
};
```

---

## 3. 核心算法解析

### 3.1 JPS 搜索流程

```
1. 初始化
   ├─ 创建起点节点
   ├─ 加入 Open Set
   └─ 设置 g=0, h=启发式值

2. 主循环
   ├─ 从 Open Set 取出 f 值最小的节点
   ├─ 检查是否到达目标
   ├─ 获取跳点（getJpsSucc）
   │   ├─ 对每个搜索方向
   │   ├─ 调用 jump() 函数
   │   └─ 返回跳点坐标
   ├─ 处理每个跳点
   │   ├─ 计算新的 g 值
   │   ├─ 如果更优，更新节点
   │   └─ 加入 Open Set
   └─ 标记当前节点为 closed

3. 路径恢复
   └─ 从目标节点回溯到起点
```

### 3.2 Jump 函数（核心）

**功能**：沿着方向 (dx, dy) 跳跃，直到找到跳点或障碍物

**跳点定义**：
1. 目标点
2. 有强制邻居的点
3. 对角线方向上，子方向找到跳点

**伪代码**：
```cpp
bool jump(int x, int y, int dx, int dy, int& new_x, int& new_y) {
  // 1. 沿方向前进一步
  x = x + dx;
  y = y + dy;
  
  // 2. 检查是否可行
  if (!isFree(x, y)) return false;
  
  // 3. 检查是否是目标
  if (x == xGoal && y == yGoal) {
    new_x = x; new_y = y;
    return true;
  }
  
  // 4. 检查是否有强制邻居
  if (hasForced(x, y, dx, dy)) {
    new_x = x; new_y = y;
    return true;
  }
  
  // 5. 对角线方向：递归检查子方向
  if (dx != 0 && dy != 0) {
    if (jump(x, y, dx, 0, ...) || jump(x, y, 0, dy, ...)) {
      new_x = x; new_y = y;
      return true;
    }
  }
  
  // 6. 继续跳跃
  return jump(x, y, dx, dy, new_x, new_y);
}
```

### 3.3 路径优化

**removeCornerPts() - 去除拐角点**：

```
原始路径:  A → B → C → D → E
           ↓   ↓   ↓   ↓   ↓
优化策略:  检查 A→C 是否无碰撞
           ├─ 是：跳过 B，继续检查 A→D
           └─ 否：保留 B，从 B 开始检查

优化结果:  A → C → E  (减少路径点)
```

**算法**：
```cpp
std::vector<Eigen::Vector2d> removeCornerPts(const std::vector<Eigen::Vector2d> &path) {
  std::vector<Eigen::Vector2d> optimized_path;
  optimized_path.push_back(path[0]);
  
  Eigen::Vector2d prev_pose = path[0];
  
  for (int i = 1; i < path.size() - 1; i++) {
    Eigen::Vector2d pose1 = path[i];
    Eigen::Vector2d pose2 = path[i + 1];
    
    // 检查是否可以直接连接 prev_pose → pose2
    if (!checkLineCollision(prev_pose, pose2)) {
      // 可以跳过 pose1
      continue;
    } else {
      // 必须保留 pose1
      optimized_path.push_back(pose1);
      prev_pose = pose1;
    }
  }
  
  optimized_path.push_back(path.back());
  return optimized_path;
}
```

### 3.4 轨迹生成

**getSampleTraj() - 生成采样轨迹**：

为每个路径点生成 5D 状态：`(x, y, theta, dtheta, ds)`

```cpp
void getSampleTraj() {
  // 1. 起点
  state5d << start_x, start_y, start_theta, 0, 0;
  trajs.push_back(state5d);
  
  // 2. 中间点
  for (int i = 1; i < path.size() - 1; i++) {
    // 2.1 到达点 i（保持前一个航向）
    state5d << path[i].x, path[i].y, prev_theta, 0, distance;
    trajs.push_back(state5d);
    
    // 2.2 转向（计算新航向）
    new_theta = atan2(path[i+1].y - path[i].y, path[i+1].x - path[i].x);
    state5d << path[i].x, path[i].y, new_theta, dtheta, 0;
    trajs.push_back(state5d);
  }
  
  // 3. 终点
  state5d << goal_x, goal_y, goal_theta, dtheta, distance;
  trajs.push_back(state5d);
}
```

**getTrajsWithTime() - 添加时间信息**：

根据速度/加速度限制计算每段的时间。

---

## 4. 依赖关系分析

### 4.1 外部依赖

| 依赖项 | 用途 | 是否必需 | 替代方案 |
|--------|------|---------|---------|
| **ROS** | 框架、通信、参数 | ❌ | 移除，使用 NavSim 框架 |
| `ros::NodeHandle` | 参数读取 | ❌ | 使用 `nlohmann::json` |
| `nav_msgs::Path` | 路径消息 | ❌ | 使用 `std::vector<TrajectoryPoint>` |
| `visualization_msgs::Marker` | 可视化 | ❌ | 移除或使用 NavSim 可视化 |
| **SDFmap** | SDF 地图 | ⚠️ | 适配到 `OccupancyGrid` |
| **Eigen** | 线性代数 | ✅ | 保留（NavSim 已使用） |
| **Boost** | 优先队列 | ✅ | 保留或使用 `std::priority_queue` |

### 4.2 关键依赖：SDFmap

**SDFmap 接口**：
```cpp
class SDFmap {
public:
  int GLX_SIZE_, GLY_SIZE_;  // 地图尺寸
  
  Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &coord);
  Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i &index);
  
  bool isOccWithSafeDis(int x, int y, double safe_dis);
  bool isOccupied(int x, int y);
  bool isUnOccupied(int x, int y);
  
  double getDistanceReal(const Eigen::Vector2d &coord);
  int Index2Vectornum(int x, int y);
};
```

**NavSim 对应**：
```cpp
// NavSim 的 OccupancyGrid
struct OccupancyGrid {
  int width, height;
  double resolution;
  Eigen::Vector2d origin;
  std::vector<uint8_t> data;  // 0=free, 100=occupied
  
  // 需要添加的方法
  Eigen::Vector2i worldToGrid(const Eigen::Vector2d &world);
  Eigen::Vector2d gridToWorld(const Eigen::Vector2i &grid);
  bool isOccupied(int x, int y);
  bool isFree(int x, int y);
};
```

---

## 5. 适配到 NavSim-Local 的方案

### 5.1 适配策略

**方案 A：完全重写（推荐）** ✅
- 保留核心 JPS 算法（`GraphSearch`）
- 移除所有 ROS 依赖
- 适配到 NavSim 插件接口
- 使用 `OccupancyGrid` 替代 `SDFmap`

**方案 B：最小改动**
- 保留大部分代码
- 创建 `SDFmap` 适配器包装 `OccupancyGrid`
- 风险：代码耦合度高，难以维护

### 5.2 适配架构

```
┌─────────────────────────────────────────┐
│   JPSPlannerPlugin                      │
│   (实现 PlannerPluginInterface)         │
├─────────────────────────────────────────┤
│  • getMetadata()                        │
│  • initialize(config)                   │
│  • plan(context, deadline, result)      │
│  • isAvailable(context)                 │
└──────────────┬──────────────────────────┘
               │
               ↓
┌─────────────────────────────────────────┐
│   JPSSearchEngine                       │
│   (封装 GraphSearch)                    │
├─────────────────────────────────────────┤
│  • search(start, goal, grid)            │
│  • getPath()                            │
│  • optimizePath()                       │
└──────────────┬──────────────────────────┘
               │
               ↓
┌─────────────────────────────────────────┐
│   GraphSearch (核心算法)                │
├─────────────────────────────────────────┤
│  • plan()                               │
│  • jump()                               │
│  • getJpsSucc()                         │
└─────────────────────────────────────────┘
```

### 5.3 数据流

```
PlanningContext (NavSim)
    ↓
  occupancy_grid
    ↓
JPSPlannerPlugin::plan()
    ↓
JPSSearchEngine::search()
    ↓
GraphSearch::plan()
    ↓
  跳点搜索
    ↓
  路径优化
    ↓
  轨迹生成
    ↓
PlanningResult (NavSim)
```

---

## 6. 改造步骤

### 6.1 第一阶段：核心算法移植

**目标**：移植 `GraphSearch` 类，移除 ROS 依赖

**步骤**：

1. **创建新文件**：
   ```
   plugins/planning/jps_planner/
   ├── include/
   │   ├── jps_planner_plugin.hpp
   │   ├── jps_search_engine.hpp
   │   └── jps_graph_search.hpp
   └── src/
       ├── jps_planner_plugin.cpp
       ├── jps_search_engine.cpp
       ├── jps_graph_search.cpp
       └── register.cpp
   ```

2. **移植 `GraphSearch`**：
   - 复制 `graph_search.h/cpp` → `jps_graph_search.hpp/cpp`
   - 移除 `#include <plan_env/sdf_map.h>`
   - 移除 `std::shared_ptr<SDFmap> map_`
   - 添加直接使用栅格地图的接口

3. **创建栅格地图适配器**：
   ```cpp
   class GridMapAdapter {
   public:
     GridMapAdapter(const OccupancyGrid* grid, double safe_dis);
     
     bool isFree(int x, int y) const;
     bool isOccupied(int x, int y) const;
     int getWidth() const { return grid_->width; }
     int getHeight() const { return grid_->height; }
     
   private:
     const OccupancyGrid* grid_;
     double safe_dis_;
   };
   ```

### 6.2 第二阶段：插件封装

**目标**：实现 `JPSPlannerPlugin`

**关键代码**：

```cpp
class JPSPlannerPlugin : public plugin::PlannerPluginInterface {
public:
  plugin::PlannerPluginMetadata getMetadata() const override {
    return {
      .name = "JPSPlanner",
      .version = "1.0.0",
      .description = "Jump Point Search path planner",
      .type = "search",
      .author = "Adapted from ROS JPS",
      .can_be_fallback = false,
      .required_perception = {"occupancy_grid"}
    };
  }
  
  bool initialize(const nlohmann::json& config) override {
    safe_dis_ = config.value("safe_dis", 0.3);
    max_iterations_ = config.value("max_iterations", 100000);
    path_optimization_ = config.value("path_optimization", true);
    
    search_engine_ = std::make_unique<JPSSearchEngine>(safe_dis_);
    return true;
  }
  
  bool plan(const planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           plugin::PlanningResult& result) override {
    // 1. 检查栅格地图
    if (!context.occupancy_grid) {
      result.success = false;
      result.failure_reason = "Missing occupancy grid";
      return false;
    }
    
    // 2. 转换坐标到栅格
    auto start_grid = worldToGrid(context.ego.pose, context.occupancy_grid.get());
    auto goal_grid = worldToGrid(context.task.goal_pose, context.occupancy_grid.get());
    
    // 3. 执行 JPS 搜索
    std::vector<Eigen::Vector2i> grid_path;
    bool success = search_engine_->search(
        start_grid, goal_grid, 
        context.occupancy_grid.get(),
        max_iterations_,
        grid_path);
    
    if (!success) {
      result.success = false;
      result.failure_reason = "JPS search failed";
      return false;
    }
    
    // 4. 路径优化
    if (path_optimization_) {
      grid_path = search_engine_->optimizePath(grid_path, context.occupancy_grid.get());
    }
    
    // 5. 转换为世界坐标轨迹
    result.trajectory = gridPathToTrajectory(grid_path, context.occupancy_grid.get());
    result.success = true;
    result.planner_name = "JPSPlanner";
    
    return true;
  }
  
  std::pair<bool, std::string> isAvailable(
      const planning::PlanningContext& context) const override {
    if (!context.occupancy_grid) {
      return {false, "Missing occupancy grid"};
    }
    return {true, ""};
  }

private:
  double safe_dis_;
  int max_iterations_;
  bool path_optimization_;
  std::unique_ptr<JPSSearchEngine> search_engine_;
};
```

### 6.3 第三阶段：测试和优化

**测试项**：
- [ ] 简单场景（无障碍物）
- [ ] 复杂场景（多障碍物）
- [ ] 边界情况（起点/终点在障碍物附近）
- [ ] 性能测试（大地图）
- [ ] 与 A* 对比

**优化方向**：
- 调整安全距离参数
- 优化路径平滑算法
- 添加超时检测
- 添加统计信息

---

## 总结

### 当前 JPS 规划器特点

✅ **优点**：
- 算法成熟，经过实际验证
- 搜索效率高（比 A* 快）
- 路径优化功能完善
- 支持轨迹生成

❌ **缺点**：
- 强依赖 ROS 生态
- 使用 SDFmap（NavSim 使用 OccupancyGrid）
- 代码耦合度高
- 缺少 NavSim 插件接口

### 适配工作量评估

| 任务 | 工作量 | 优先级 |
|------|--------|--------|
| 移植核心算法 | 2-3 天 | 高 |
| 创建栅格地图适配器 | 1 天 | 高 |
| 实现插件接口 | 1-2 天 | 高 |
| 路径优化适配 | 1 天 | 中 |
| 测试和调试 | 2-3 天 | 高 |
| **总计** | **7-10 天** | - |

### 建议

1. **优先移植核心算法**：`GraphSearch` 是最有价值的部分
2. **简化路径优化**：初期可以使用简单的直线优化
3. **参考 A* 插件**：NavSim 已有的 A* 插件可以作为参考
4. **逐步迭代**：先实现基本功能，再优化性能

---

**下一步行动**：
1. 阅读 NavSim 的 `AStarPlannerPlugin` 实现
2. 创建 `JPSPlannerPlugin` 框架
3. 移植 `GraphSearch` 核心算法
4. 实现栅格地图适配器
5. 测试基本功能


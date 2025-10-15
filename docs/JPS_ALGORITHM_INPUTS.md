# JPS 算法完整输入说明

## 📋 目录

1. [核心输入](#1-核心输入)
2. [配置参数](#2-配置参数)
3. [数据流详解](#3-数据流详解)
4. [NavSim 适配](#4-navsim-适配)

---

## 1. 核心输入

### 1.1 GraphSearch 层（JPS 核心算法）

**函数签名**：
```cpp
bool GraphSearch::plan(int xStart, int yStart, 
                       int xGoal, int yGoal, 
                       bool useJps, 
                       int maxExpand = -1);
```

**输入参数**：

| 参数 | 类型 | 说明 | 示例 |
|------|------|------|------|
| `xStart` | `int` | 起点的栅格 X 坐标 | `50` |
| `yStart` | `int` | 起点的栅格 Y 坐标 | `100` |
| `xGoal` | `int` | 终点的栅格 X 坐标 | `200` |
| `yGoal` | `int` | 终点的栅格 Y 坐标 | `150` |
| `useJps` | `bool` | 是否使用 JPS（false 则使用 A*） | `true` |
| `maxExpand` | `int` | 最大扩展节点数（-1 表示无限制） | `100000` |

**构造函数输入**：
```cpp
GraphSearch::GraphSearch(std::shared_ptr<SDFmap> Map, const double &safe_dis);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `Map` | `std::shared_ptr<SDFmap>` | SDF 地图指针 |
| `safe_dis` | `double` | 安全距离（米） |

**地图信息（通过 SDFmap）**：
- `xDim_` - 地图宽度（栅格数）
- `yDim_` - 地图高度（栅格数）
- `resolution` - 栅格分辨率（米/栅格）
- `origin` - 地图原点（世界坐标）
- 障碍物信息（通过 `isOccupied()` 查询）

---

### 1.2 JPSPlanner 层（高层封装）

**函数签名**：
```cpp
bool JPSPlanner::plan(const Eigen::Vector3d &start, 
                      const Eigen::Vector3d &goal);
```

**输入参数**：

| 参数 | 类型 | 说明 | 维度 |
|------|------|------|------|
| `start` | `Eigen::Vector3d` | 起点状态 (x, y, yaw) | 3D |
| `goal` | `Eigen::Vector3d` | 终点状态 (x, y, yaw) | 3D |

**详细说明**：
```cpp
start = [x_start, y_start, yaw_start]
  x_start   - 起点 X 坐标（世界坐标，米）
  y_start   - 起点 Y 坐标（世界坐标，米）
  yaw_start - 起点航向角（弧度）

goal = [x_goal, y_goal, yaw_goal]
  x_goal   - 终点 X 坐标（世界坐标，米）
  y_goal   - 终点 Y 坐标（世界坐标，米）
  yaw_goal - 终点航向角（弧度）
```

**构造函数输入**：
```cpp
JPSPlanner::JPSPlanner(std::shared_ptr<SDFmap> map, const ros::NodeHandle &nh);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `map` | `std::shared_ptr<SDFmap>` | SDF 地图指针 |
| `nh` | `ros::NodeHandle` | ROS 节点句柄（用于读取参数） |

---

### 1.3 轨迹生成层（可选）

**函数签名**：
```cpp
void JPSPlanner::getKinoNodeWithStartPath(
    const std::vector<Eigen::Vector3d> &start_path, 
    const bool if_forward, 
    const Eigen::Vector3d &current_state_VAJ, 
    const Eigen::Vector3d &current_state_OAJ);
```

**输入参数**：

| 参数 | 类型 | 说明 |
|------|------|------|
| `start_path` | `std::vector<Eigen::Vector3d>` | 起始路径段（可选） |
| `if_forward` | `bool` | 是否前向规划 |
| `current_state_VAJ` | `Eigen::Vector3d` | 当前速度/加速度/加加速度 (v, a, j) |
| `current_state_OAJ` | `Eigen::Vector3d` | 当前角速度/角加速度/角加加速度 (ω, α, j) |

---

## 2. 配置参数

### 2.1 ROS 参数（原始版本）

从 ROS 参数服务器读取：

```yaml
# 路径规划参数
jps_safe_dis: 0.3           # 安全距离（米）
max_jps_dis: 100.0          # 最大搜索距离（米）
jps_distance_weight: 1.0    # 距离权重
jps_yaw_weight: 1.0         # 航向权重
trajCutLength: 10.0         # 轨迹截断长度（米）

# 动力学约束
max_vel: 2.0                # 最大速度（米/秒）
max_acc: 1.0                # 最大加速度（米/秒²）
max_omega: 1.0              # 最大角速度（弧度/秒）
max_domega: 1.0             # 最大角加速度（弧度/秒²）

# 轨迹采样
timeResolution: 0.1         # 时间分辨率（秒）
mintrajNum: 10              # 最小轨迹点数
jps_truncation_time: 5.0    # 轨迹截断时间（秒）
```

### 2.2 NavSim 配置（适配版本）

JSON 配置文件：

```json
{
  "planning": {
    "primary_planner": "JPSPlanner",
    "planners": {
      "JPSPlanner": {
        "safe_dis": 0.3,
        "max_jps_dis": 100.0,
        "distance_weight": 1.0,
        "yaw_weight": 1.0,
        "traj_cut_length": 10.0,
        "max_vel": 2.0,
        "max_acc": 1.0,
        "max_omega": 1.0,
        "max_domega": 1.0,
        "time_step": 0.1,
        "min_traj_num": 10,
        "truncation_time": 5.0,
        "max_iterations": 100000,
        "path_optimization": true
      }
    }
  }
}
```

---

## 3. 数据流详解

### 3.1 完整的输入数据流

```
┌─────────────────────────────────────────────────────────┐
│  外部输入                                                │
├─────────────────────────────────────────────────────────┤
│  • 起点：(x_start, y_start, yaw_start)                   │
│  • 终点：(x_goal, y_goal, yaw_goal)                      │
│  • 地图：OccupancyGrid / SDFmap                          │
│  • 配置：JSON / ROS Params                               │
└────────────┬────────────────────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────────────────────┐
│  JPSPlanner::plan()                                     │
├─────────────────────────────────────────────────────────┤
│  1. 坐标转换：世界坐标 → 栅格坐标                         │
│     start_idx = map_->coord2gridIndex(start.head(2))    │
│     goal_idx = map_->coord2gridIndex(goal.head(2))      │
│                                                          │
│  2. 动态调整安全距离                                      │
│     start_dis = map_->getDistanceReal(start_idx)        │
│     goal_dis = map_->getDistanceReal(goal_idx)          │
│     safe_dis = min(safe_dis_, start_dis, goal_dis)      │
└────────────┬────────────────────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────────────────────┐
│  GraphSearch::plan()                                    │
├─────────────────────────────────────────────────────────┤
│  输入：                                                  │
│    • xStart, yStart - 起点栅格坐标                       │
│    • xGoal, yGoal - 终点栅格坐标                         │
│    • useJps = true - 使用 JPS                            │
│    • maxExpand = 100000 - 最大扩展数                     │
│                                                          │
│  内部使用：                                              │
│    • map_ - 地图指针                                     │
│    • safe_dis_ - 安全距离                                │
│    • eps_ - 启发式权重                                   │
│    • xDim_, yDim_ - 地图尺寸                             │
└────────────┬────────────────────────────────────────────┘
             │
             ↓
┌─────────────────────────────────────────────────────────┐
│  JPS 核心算法                                            │
├─────────────────────────────────────────────────────────┤
│  • 初始化起点节点                                         │
│  • 主循环：                                              │
│    - 从 Open Set 取出最小 f 值节点                        │
│    - 调用 jump() 查找跳点                                │
│    - 更新后继节点                                         │
│  • 回溯路径                                              │
└─────────────────────────────────────────────────────────┘
```

### 3.2 坐标转换示例

**世界坐标 → 栅格坐标**：

```cpp
// 输入：世界坐标
Eigen::Vector3d start(10.5, 20.3, 0.0);  // (x, y, yaw) in meters

// 地图参数
double resolution = 0.1;  // 0.1 米/栅格
Eigen::Vector2d origin(-50.0, -50.0);  // 地图原点

// 转换
Eigen::Vector2i start_idx = map_->coord2gridIndex(start.head(2));
// start_idx.x() = (10.5 - (-50.0)) / 0.1 = 605
// start_idx.y() = (20.3 - (-50.0)) / 0.1 = 703

// 传递给 GraphSearch
graph_search_->plan(605, 703, goal_x, goal_y, true);
```

**栅格坐标 → 世界坐标**：

```cpp
// 输入：栅格坐标
Eigen::Vector2i grid(605, 703);

// 转换
Eigen::Vector2d world = map_->gridIndex2coordd(grid);
// world.x() = -50.0 + 605 * 0.1 = 10.5
// world.y() = -50.0 + 703 * 0.1 = 20.3
```

---

## 4. NavSim 适配

### 4.1 NavSim 的输入结构

**PlanningContext**：
```cpp
struct PlanningContext {
  // 自车状态
  struct {
    Pose pose;           // (x, y, theta)
    double velocity;
    double acceleration;
  } ego;
  
  // 任务信息
  struct {
    Pose goal_pose;      // (x, y, theta)
  } task;
  
  // 感知数据
  std::shared_ptr<OccupancyGrid> occupancy_grid;
  std::shared_ptr<ObjectList> objects;
  // ...
};
```

**OccupancyGrid**：
```cpp
struct OccupancyGrid {
  int width;                    // 栅格宽度
  int height;                   // 栅格高度
  double resolution;            // 分辨率（米/栅格）
  Eigen::Vector2d origin;       // 原点（世界坐标）
  std::vector<uint8_t> data;    // 占据数据（0=free, 100=occupied）
};
```

### 4.2 输入映射

**原始 ROS 版本 → NavSim 版本**：

| 原始输入 | NavSim 输入 | 说明 |
|---------|------------|------|
| `start` (Vector3d) | `context.ego.pose` | 起点状态 |
| `goal` (Vector3d) | `context.task.goal_pose` | 终点状态 |
| `SDFmap` | `context.occupancy_grid` | 地图数据 |
| `current_state_VAJ` | `context.ego.velocity/acceleration` | 动力学状态 |
| ROS Params | JSON config | 配置参数 |

### 4.3 适配代码示例

```cpp
bool JPSPlannerPlugin::plan(const planning::PlanningContext& context,
                            std::chrono::milliseconds deadline,
                            plugin::PlanningResult& result) {
  // 1. 提取输入
  Eigen::Vector3d start(
      context.ego.pose.x,
      context.ego.pose.y,
      context.ego.pose.theta
  );
  
  Eigen::Vector3d goal(
      context.task.goal_pose.x,
      context.task.goal_pose.y,
      context.task.goal_pose.theta
  );
  
  // 2. 创建地图适配器
  auto grid_adapter = std::make_shared<JPSGridAdapter>(
      context.occupancy_grid.get(),
      config_.safe_dis
  );
  
  // 3. 坐标转换
  Eigen::Vector2i start_grid = grid_adapter->worldToGrid(start.head(2));
  Eigen::Vector2i goal_grid = grid_adapter->worldToGrid(goal.head(2));
  
  // 4. 调用 JPS 搜索
  bool success = planner_core_->search(
      start, goal, grid_adapter, raw_path
  );
  
  // 5. 路径优化
  if (success && config_.path_optimization) {
    optimized_path = planner_core_->optimizePath(raw_path, grid_adapter);
  }
  
  // 6. 生成轨迹
  result.trajectory = convertToTrajectory(optimized_path);
  result.success = success;
  
  return success;
}
```

---

## 5. 总结

### 5.1 核心输入总结

**最小输入**（仅搜索）：
1. ✅ 起点栅格坐标 `(xStart, yStart)`
2. ✅ 终点栅格坐标 `(xGoal, yGoal)`
3. ✅ 地图数据（通过 `SDFmap` 或 `JPSGridAdapter`）
4. ✅ 安全距离 `safe_dis`

**完整输入**（含轨迹生成）：
1. ✅ 起点状态 `(x, y, yaw)`
2. ✅ 终点状态 `(x, y, yaw)`
3. ✅ 地图数据
4. ✅ 配置参数（安全距离、速度限制等）
5. ✅ 当前动力学状态（速度、加速度）

### 5.2 输入层次

```
Level 1: GraphSearch (JPS 核心)
  输入：栅格坐标 + 地图
  
Level 2: JPSPlanner (高层封装)
  输入：世界坐标 + 地图 + 配置
  
Level 3: JPSPlannerPlugin (NavSim 插件)
  输入：PlanningContext (包含所有信息)
```

### 5.3 关键点

1. **坐标系统**：
   - 外部使用世界坐标（米）
   - 内部使用栅格坐标（整数）
   - 需要坐标转换

2. **地图接口**：
   - 原始版本：`SDFmap`（提供方法）
   - NavSim 版本：`OccupancyGrid`（仅数据）
   - 需要适配器：`JPSGridAdapter`

3. **配置方式**：
   - 原始版本：ROS 参数服务器
   - NavSim 版本：JSON 配置文件
   - 需要转换逻辑

4. **动力学状态**：
   - 基础搜索：不需要
   - 轨迹生成：需要速度、加速度等
   - 可选功能


# NavSim 插件配置快速指南

**5 分钟学会如何配置 NavSim 插件**

---

## 🎯 配置文件位置

**主配置文件**: `config/default.json`

这是程序默认加载的配置文件。

---

## ⚡ 快速配置

### 1️⃣ **选择感知插件**

在 `config/default.json` 中找到 `perception_plugins` 部分：

```json
{
  "perception_plugins": [
    {
      "name": "GridMapBuilder",    // ⬅️ 插件名称
      "enabled": true,              // ⬅️ true=启用, false=禁用
      "priority": 100,
      "params": {
        "resolution": 0.1,          // ⬅️ 栅格分辨率
        "map_width": 100.0,         // ⬅️ 地图宽度
        "inflation_radius": 0.5     // ⬅️ 障碍物膨胀半径
      }
    }
  ]
}
```

**可用插件**：
- `GridMapBuilder` - 栅格地图构建器 ✅

### 2️⃣ **选择规划器**

在 `config/default.json` 中找到 `planning` 部分：

```json
{
  "planning": {
    "primary_planner": "StraightLinePlanner",   // ⬅️ 主规划器
    "fallback_planner": "StraightLinePlanner",  // ⬅️ 降级规划器
    "enable_fallback": true
  }
}
```

**可用规划器**：
- `StraightLinePlanner` - 直线规划器（快速，简单）
- `AStarPlanner` - A* 路径规划器（避障，智能）

### 3️⃣ **调整规划器参数**

在 `config/default.json` 中找到 `planners` 部分：

```json
{
  "planners": {
    "StraightLinePlanner": {
      "default_velocity": 1.5,      // ⬅️ 速度（米/秒）
      "planning_horizon": 5.0,      // ⬅️ 规划时长（秒）
      "max_acceleration": 1.0       // ⬅️ 最大加速度
    },
    "AStarPlanner": {
      "heuristic_weight": 1.2,      // ⬅️ 启发式权重
      "max_iterations": 10000,      // ⬅️ 最大迭代次数
      "goal_tolerance": 0.5         // ⬅️ 目标容差（米）
    }
  }
}
```

---

## 📋 常见配置场景

### 场景 1: 使用 A* 规划器

**修改**：
```json
{
  "planning": {
    "primary_planner": "AStarPlanner",          // ⬅️ 改这里
    "fallback_planner": "StraightLinePlanner"
  }
}
```

**保存后直接运行**：
```bash
$ ./build/navsim_algo
[DynamicPluginLoader] Loading plugin 'AStarPlanner' from: ./build/plugins/planning/astar/libastar_planner_plugin.so
[DynamicPluginLoader] Successfully loaded plugin: AStarPlanner
```

### 场景 2: 禁用感知插件

**修改**：
```json
{
  "perception_plugins": [
    {
      "name": "GridMapBuilder",
      "enabled": false,              // ⬅️ 改为 false
      "priority": 100,
      "params": { ... }
    }
  ]
}
```

### 场景 3: 提高规划速度

**修改**：
```json
{
  "planners": {
    "StraightLinePlanner": {
      "default_velocity": 3.0,       // ⬅️ 提高速度
      "max_acceleration": 2.0        // ⬅️ 提高加速度
    }
  }
}
```

### 场景 4: 更精确的路径规划

**修改**：
```json
{
  "perception_plugins": [
    {
      "name": "GridMapBuilder",
      "params": {
        "resolution": 0.05,          // ⬅️ 更高分辨率
        "inflation_radius": 0.3      // ⬅️ 更小膨胀半径
      }
    }
  ],
  "planners": {
    "AStarPlanner": {
      "step_size": 0.2,              // ⬅️ 更小步长
      "goal_tolerance": 0.2          // ⬅️ 更小容差
    }
  }
}
```

---

## 🔧 如何应用配置

### 方法 1: 修改默认配置（推荐）

```bash
# 1. 编辑配置文件
$ vim config/default.json

# 2. 直接运行（自动加载 default.json）
$ ./build/navsim_algo
```

### 方法 2: 使用示例配置

```bash
# 1. 复制示例配置
$ cp config/example_astar.json config/default.json

# 2. 运行
$ ./build/navsim_algo
```

### 方法 3: 指定配置文件

```bash
# 运行时指定配置文件
$ ./build/navsim_algo --config config/example_astar.json
```

---

## ✅ 验证配置

运行程序后，查看输出确认插件加载：

```bash
$ ./build/test_plugin_system

# 应该看到：
[DynamicPluginLoader] Loading plugins from config: config/default.json
[DynamicPluginLoader] Found 1 perception plugins in config
[DynamicPluginLoader] Perception plugin: GridMapBuilder (enabled: 1)
[DynamicPluginLoader] Loading plugin 'GridMapBuilder' from: ./build/plugins/perception/grid_map_builder/libgrid_map_builder_plugin.so
[DynamicPluginLoader] Successfully loaded plugin: GridMapBuilder
[DynamicPluginLoader] Loading plugin 'StraightLinePlanner' from: ./build/plugins/planning/straight_line/libstraight_line_planner_plugin.so
[DynamicPluginLoader] Successfully loaded plugin: StraightLinePlanner
[DynamicPluginLoader] Loaded 2 plugins from config
```

---

## 📊 配置参数速查表

### 感知插件参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `resolution` | float | 0.1 | 栅格分辨率（米/格） |
| `map_width` | float | 100.0 | 地图宽度（米） |
| `map_height` | float | 100.0 | 地图高度（米） |
| `obstacle_cost` | int | 100 | 障碍物代价 |
| `inflation_radius` | float | 0.5 | 膨胀半径（米） |

### StraightLine 规划器参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `default_velocity` | float | 1.5 | 默认速度（米/秒） |
| `time_step` | float | 0.1 | 时间步长（秒） |
| `planning_horizon` | float | 5.0 | 规划时域（秒） |
| `use_trapezoidal_profile` | bool | true | 使用梯形速度曲线 |
| `max_acceleration` | float | 1.0 | 最大加速度（米/秒²） |
| `max_deceleration` | float | 1.5 | 最大减速度（米/秒²） |

### A* 规划器参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `time_step` | float | 0.1 | 时间步长（秒） |
| `heuristic_weight` | float | 1.2 | 启发式权重（1.0=最优，>1.0=更快） |
| `step_size` | float | 0.5 | 搜索步长（米） |
| `max_iterations` | int | 10000 | 最大迭代次数 |
| `goal_tolerance` | float | 0.5 | 目标容差（米） |
| `default_velocity` | float | 1.5 | 默认速度（米/秒） |

---

## ❓ 常见问题

### Q1: 修改配置后需要重新编译吗？

**A**: **不需要！** 这就是动态加载的优势。

```bash
# 1. 修改配置
$ vim config/default.json

# 2. 直接运行（无需编译）
$ ./build/navsim_algo
```

### Q2: 如何知道有哪些可用插件？

**A**: 查看 `build/plugins` 目录：

```bash
$ find build/plugins -name "*.so"
build/plugins/perception/grid_map_builder/libgrid_map_builder_plugin.so
build/plugins/planning/straight_line/libstraight_line_planner_plugin.so
build/plugins/planning/astar/libastar_planner_plugin.so
```

插件名称：
- `libgrid_map_builder_plugin.so` → `GridMapBuilder`
- `libstraight_line_planner_plugin.so` → `StraightLinePlanner`
- `libastar_planner_plugin.so` → `AStarPlanner`

### Q3: 如何禁用某个插件？

**A**: 将 `enabled` 设置为 `false`：

```json
{
  "perception_plugins": [
    {
      "name": "GridMapBuilder",
      "enabled": false              // ⬅️ 禁用
    }
  ]
}
```

### Q4: 配置文件格式错误怎么办？

**A**: 程序会输出错误信息：

```
[ConfigLoader] Failed to parse config file: config/default.json
[ConfigLoader] JSON parse error: ...
```

使用 JSON 验证工具检查：
```bash
$ python3 -m json.tool config/default.json
```

---

## 📚 更多信息

- **插件开发**: 查看 `plugins/README.md`
- **外部插件**: 查看 `external_plugins/README.md`
- **动态加载原理**: 查看 `DYNAMIC_PLUGIN_LOADING_REPORT.md`

---

**最后更新**: 2025-10-13


# 可用插件列表

本文档列出了 navsim-local 新架构中所有可用的插件。

## 📊 插件统计

- **规划器插件**: 3 个 ✅
- **感知插件**: 2 个 ✅
- **总计**: 5 个

---

## 🎯 规划器插件 (Planning Plugins)

### 1. StraightLinePlanner ✅

**描述**: 简单的直线规划器，生成从起点到终点的直线轨迹。

**特点**:
- 不考虑障碍物
- 适用于快速测试和调试
- 无外部依赖

**使用方法**:
```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner StraightLinePlanner
```

**配置参数**:
- `num_points`: 轨迹点数量（默认: 50）
- `max_vel`: 最大速度 (m/s)（默认: 2.0）

**测试结果**:
- ✅ 成功生成 50 个轨迹点
- ✅ 计算时间: ~0.02 ms

---

### 2. AStarPlanner ✅

**描述**: 基于 A* 算法的栅格路径规划器。

**特点**:
- 基于栅格地图搜索
- 支持启发式权重调整
- 保证找到最优路径（如果存在）

**依赖**:
- `GridMapBuilder`: 构建栅格占据地图
- `ESDFBuilder`: （可选）用于安全距离查询

**使用方法**:
```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner AStarPlanner \
  --perception GridMapBuilder,ESDFBuilder
```

**配置参数**:
- `heuristic_weight`: 启发式权重（默认: 1.0）
- `step_size`: 搜索步长 (m)（默认: 0.5）
- `max_iterations`: 最大迭代次数（默认: 10000）
- `goal_tolerance`: 目标容差 (m)（默认: 0.5）

**测试结果**:
- ✅ 成功加载和初始化
- ⚠️ 需要合适的场景配置才能找到路径

---

### 3. JpsPlanner ✅

**描述**: Jump Point Search (JPS) 规划器，A* 的优化版本。

**特点**:
- 比 A* 更快（跳过对称路径）
- 基于栅格地图和 ESDF
- 适用于大规模地图

**依赖**:
- `GridMapBuilder`: 构建栅格占据地图（必需）
- `ESDFBuilder`: 构建 ESDF 距离场（必需）
- Boost 库: 用于优先队列

**使用方法**:
```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner JpsPlanner \
  --perception GridMapBuilder,ESDFBuilder
```

**配置参数**:
- `safe_dis`: 安全距离 (m)（默认: 0.3）
- `max_vel`: 最大速度 (m/s)（默认: 2.0）
- `max_acc`: 最大加速度 (m/s²)（默认: 2.0）
- `resolution`: 地图分辨率 (m/cell)（默认: 0.1）

**测试结果**:
- ✅ 成功生成 2 个轨迹点
- ✅ 计算时间: ~9 ms

---

## 👁️ 感知插件 (Perception Plugins)

### 1. GridMapBuilder ✅

**描述**: 栅格占据地图构建器，从 BEV 障碍物构建栅格地图。

**特点**:
- 支持圆形、矩形、多边形障碍物
- 支持障碍物膨胀（安全距离）
- 以自车为中心的局部地图

**输出**:
- `context.occupancy_grid`: 栅格占据地图

**使用方法**:
```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner AStarPlanner \
  --perception GridMapBuilder
```

**配置参数**:
- `resolution`: 栅格分辨率 (m/cell)（默认: 0.1）
- `map_width`: 地图宽度 (m)（默认: 100.0）
- `map_height`: 地图高度 (m)（默认: 100.0）
- `inflation_radius`: 膨胀半径 (m)（默认: 0.5）

**测试结果**:
- ✅ 成功构建 1000x1000 栅格地图
- ✅ 正确处理静态障碍物

---

### 2. ESDFBuilder ✅

**描述**: ESDF (Euclidean Signed Distance Field) 地图构建器。

**特点**:
- 计算每个栅格到最近障碍物的欧几里得距离
- 支持动态障碍物
- 用于基于梯度的轨迹优化

**依赖**:
- `GridMapBuilder`: 需要先构建栅格地图

**输出**:
- `context.esdf_map`: ESDF 距离场地图

**使用方法**:
```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner JpsPlanner \
  --perception GridMapBuilder,ESDFBuilder
```

**配置参数**:
- `resolution`: 栅格分辨率 (m/cell)（默认: 0.1）
- `map_width`: 地图宽度 (m)（默认: 100.0）
- `map_height`: 地图高度 (m)（默认: 100.0）
- `max_distance`: 最大距离 (m)（默认: 10.0）
- `include_dynamic`: 是否包含动态障碍物（默认: true）

**测试结果**:
- ✅ 成功构建 1000x1000 ESDF 地图
- ✅ 正确计算距离场

---

## 📦 插件文件位置

```
build/plugins/
├── planning/
│   ├── straight_line/
│   │   └── libstraight_line_planner_plugin.so  ✅
│   ├── astar/
│   │   └── liba_star_planner_plugin.so         ✅
│   └── jps_planner_plugin/
│       └── libjps_planner_plugin.so            ✅
└── perception/
    ├── grid_map_builder/
    │   └── libgrid_map_builder_plugin.so       ✅
    └── esdf_builder/
        └── libesdf_builder_plugin.so           ✅
```

---

## 🔧 插件命名规范

插件名称遵循以下转换规则：

| 插件名称 (CamelCase) | 库文件名 (snake_case) |
|---------------------|----------------------|
| `StraightLinePlanner` | `libstraight_line_planner_plugin.so` |
| `AStarPlanner` | `liba_star_planner_plugin.so` |
| `JpsPlanner` | `libjps_planner_plugin.so` |
| `GridMapBuilder` | `libgrid_map_builder_plugin.so` |
| `ESDFBuilder` | `libesdf_builder_plugin.so` |

**转换逻辑**:
1. 驼峰命名转下划线命名
2. 连续大写字母特殊处理（如 `ESDF` → `esdf`）
3. 添加前缀 `lib` 和后缀 `_plugin.so`

---

## 🚀 使用示例

### 示例 1: 仅使用规划器（无障碍物）

```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner StraightLinePlanner
```

### 示例 2: 使用 A* 规划器 + 栅格地图

```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner AStarPlanner \
  --perception GridMapBuilder
```

### 示例 3: 使用 JPS 规划器 + 完整感知

```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner JpsPlanner \
  --perception GridMapBuilder,ESDFBuilder \
  --verbose
```

### 示例 4: 使用完整路径加载插件

```bash
./build/navsim_local_debug \
  --scenario scenarios/parking_scenario.json \
  --planner build/plugins/planning/astar/liba_star_planner_plugin.so \
  --perception build/plugins/perception/grid_map_builder/libgrid_map_builder_plugin.so
```

---

## ⚠️ 已知问题

### 1. 段错误 (Segmentation Fault)

**问题**: 程序退出时可能出现段错误。

**原因**: 插件卸载顺序问题。

**影响**: 不影响核心功能，仅在程序退出时发生。

**状态**: 待修复

### 2. ESDFBuilder 注册函数警告

**问题**: `Warning: Cannot find register function 'registerESDFBuilderPlugin'`

**原因**: 注册函数名称不匹配（`registerEsdfBuilderPlugin` vs `registerESDFBuilderPlugin`）。

**影响**: 不影响功能，插件使用静态注册。

**状态**: 待修复

---

## 🎯 未来计划

### 即将添加的插件

1. **RRT* Planner**: 基于采样的路径规划器
2. **Hybrid A* Planner**: 考虑车辆运动学的规划器
3. **TEB Planner**: 基于时间弹性带的轨迹优化器
4. **Point Cloud Map Builder**: 点云地图构建器
5. **Semantic Segmentation**: 语义分割插件

### 改进计划

1. 修复插件卸载时的段错误
2. 统一插件注册函数命名
3. 添加插件性能分析工具
4. 添加插件单元测试
5. 完善插件文档和示例

---

## 📚 相关文档

- [本地调试模式使用指南](LOCAL_DEBUG_MODE.md)
- [场景文件格式](../scenarios/README.md)
- [插件开发指南](../templates/README.md)
- [重构方案](../REFACTORING_PROPOSAL.md)


# NavSim-Local 配置文件

本目录包含 navsim-local 插件系统的配置文件。

## 📁 目录结构

```
config/
├── README.md                    # 本文档
├── default.json.example         # 默认配置模板
└── examples/                    # 配置示例
    ├── astar_planner.json       # 使用 A* 规划器
    ├── minimal.json             # 最小配置
    └── ...
```

## 🚀 快速开始

### 1. 创建配置文件

```bash
# 复制默认配置
cp config/default.json.example config/my_config.json

# 或使用示例配置
cp config/examples/astar_planner.json config/my_config.json
```

### 2. 编辑配置文件

根据需求修改 `my_config.json`：

```json
{
  "perception": {
    "plugins": [
      {
        "name": "GridBuilderPlugin",
        "enabled": true,
        "params": {
          "resolution": 0.1
        }
      }
    ]
  },
  "planning": {
    "primary_planner": "AStarPlannerPlugin"
  }
}
```

### 3. 使用配置文件

```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/my_config.json
```

## 📋 配置文件结构

### 顶层结构

```json
{
  "version": "1.0",              // 配置文件版本
  "algorithm": { ... },          // 算法管理器配置
  "perception": { ... },         // 感知插件配置
  "planning": { ... },           // 规划器插件配置
  "visualization": { ... }       // 可视化配置
}
```

### 感知配置

感知配置分为两部分：

1. **公共前置处理层** - 固定流程，解析原始数据
2. **感知插件层** - 可扩展，构建地图表示

```json
{
  "perception": {
    "preprocessing": {
      "bev_extraction": {          // BEV 数据提取配置
        "detection_range": 50.0,
        "confidence_threshold": 0.5
      },
      "dynamic_prediction": {      // 动态障碍物预测配置
        "prediction_horizon": 5.0,
        "time_step": 0.1
      }
    },

    "plugins": [
      {
        "name": "PluginName",      // 插件名称 (必需)
        "enabled": true,           // 是否启用 (默认: true)
        "priority": 1,             // 执行优先级 (数字越小越先执行)
        "description": "...",      // 描述 (可选)
        "params": {                // 插件参数
          "param1": value1,
          "param2": value2
        }
      }
    ]
  }
}
```

### 规划器插件配置

```json
{
  "planning": {
    "primary_planner": "PlannerName",     // 主规划器
    "fallback_planner": "FallbackName",   // 降级规划器
    "enable_fallback": true,              // 是否启用降级
    
    "planners": {
      "PlannerName": {                    // 规划器参数
        "param1": value1,
        "param2": value2
      }
    }
  }
}
```

## 📚 配置示例

### 示例 1: 默认配置

使用所有感知插件和直线规划器：

```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/default.json.example
```

### 示例 2: A* 规划器

使用栅格地图和 A* 规划器：

```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/examples/astar_planner.json
```

### 示例 3: 最小配置

只使用必需的插件：

```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/examples/minimal.json
```

## 🔧 可用插件

### 公共前置处理层

前置处理层是固定流程，不是插件，负责解析原始数据。

| 模块 | 功能 | 配置项 |
|------|------|--------|
| BEV 提取器 | 从 WorldTick 提取 BEV 障碍物 | `perception.preprocessing.bev_extraction` |
| 动态障碍物预测器 | 生成动态障碍物预测轨迹 | `perception.preprocessing.dynamic_prediction` |

### 感知插件

感知插件从标准化数据构建地图表示。

| 插件名称 | 功能 | 必需参数 |
|---------|------|---------|
| `GridMapBuilderPlugin` | 从 BEV 障碍物构建栅格占据地图 | `resolution`, `map_width`, `map_height` |
| `ESDFBuilderPlugin` | 从 BEV 障碍物构建 ESDF 距离场 | `resolution`, `max_distance` |
| `PointCloudMapBuilderPlugin` | 构建点云地图 | `resolution` |

### 规划器插件

| 插件名称 | 类型 | 必需感知数据 |
|---------|------|-------------|
| `StraightLinePlannerPlugin` | 几何 | 无 |
| `AStarPlannerPlugin` | 搜索 | `occupancy_grid` |
| `OptimizationPlannerPlugin` | 优化 | `bev_obstacles` |

## ⚙️ 参数说明

### 前置处理参数

#### bev_extraction

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `detection_range` | double | 50.0 | 检测范围 (m) |
| `confidence_threshold` | double | 0.5 | 置信度阈值 |
| `include_static` | bool | true | 是否包含静态障碍物 |
| `include_dynamic` | bool | true | 是否包含动态障碍物 |

#### dynamic_prediction

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `prediction_horizon` | double | 5.0 | 预测时域 (s) |
| `time_step` | double | 0.1 | 时间步长 (s) |
| `max_trajectories` | int | 3 | 每个障碍物最大轨迹数 |
| `prediction_model` | string | "constant_velocity" | 预测模型 |

### 感知插件参数

#### GridMapBuilderPlugin

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `resolution` | double | 0.1 | 栅格分辨率 (m/cell) |
| `map_width` | double | 100.0 | 地图宽度 (m) |
| `map_height` | double | 100.0 | 地图高度 (m) |
| `inflation_radius` | double | 0.3 | 膨胀半径 (m) |
| `obstacle_cost` | int | 100 | 障碍物代价值 |

#### ESDFBuilderPlugin

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `resolution` | double | 0.1 | 距离场分辨率 (m) |
| `max_distance` | double | 10.0 | 最大距离 (m) |

### 规划器插件参数

#### StraightLinePlannerPlugin

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `time_step` | double | 0.1 | 时间步长 (s) |
| `default_velocity` | double | 2.0 | 默认速度 (m/s) |
| `max_acceleration` | double | 2.0 | 最大加速度 (m/s²) |
| `arrival_tolerance` | double | 0.5 | 到达容差 (m) |

#### AStarPlannerPlugin

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `time_step` | double | 0.1 | 时间步长 (s) |
| `heuristic_weight` | double | 1.0 | 启发式权重 |
| `step_size` | double | 0.5 | 搜索步长 (m) |
| `max_iterations` | int | 10000 | 最大迭代次数 |
| `goal_tolerance` | double | 0.5 | 目标容差 (m) |

#### OptimizationPlannerPlugin

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `time_step` | double | 0.1 | 时间步长 (s) |
| `max_iterations` | int | 100 | 最大迭代次数 |
| `convergence_tolerance` | double | 0.001 | 收敛容差 |
| `smoothness_weight` | double | 1.0 | 平滑性权重 |
| `obstacle_weight` | double | 10.0 | 障碍物权重 |
| `goal_weight` | double | 5.0 | 目标权重 |

## 🐛 故障排查

### 配置文件未加载

**症状**: 程序使用默认配置

**检查**:
1. 配置文件路径是否正确？
2. JSON 格式是否有效？（使用 JSON 验证器）
3. 是否使用了 `--config` 参数？

### 插件未启用

**症状**: 插件未执行

**检查**:
1. `enabled` 是否设置为 `true`？
2. 插件名称是否正确？
3. 是否缺少必需参数？

### 规划器不可用

**症状**: 规划器降级或失败

**检查**:
1. 是否启用了必需的感知插件？
2. 规划器参数是否有效？
3. 查看日志中的错误信息

## 📖 更多信息

- **[插件架构设计](../docs/PLUGIN_ARCHITECTURE_DESIGN.md)** - 完整设计文档
- **[快速参考](../docs/PLUGIN_QUICK_REFERENCE.md)** - 插件开发速查
- **[执行摘要](../docs/PLUGIN_ARCHITECTURE_SUMMARY.md)** - 核心要点

## 💡 提示

1. **从示例开始**: 复制一个示例配置并修改
2. **逐步调整**: 先使用默认值，再根据需求调整
3. **启用日志**: 设置 `verbose_logging: true` 查看详细信息
4. **测试配置**: 使用短时间测试验证配置是否正确

---

**最后更新**: 2025-10-13


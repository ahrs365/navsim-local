# 感知插件架构详解

本文档详细说明感知插件的架构设计，包括公共前置处理层和感知插件层的职责划分。

## 📐 架构概览

### 数据流图

```
┌─────────────────────────────────────────────────────────────────┐
│                      proto::WorldTick                           │
│  (上游话题 - 原始数据)                                            │
│  - ego: 自车状态                                                 │
│  - static_map: 静态地图                                          │
│  - dynamic_obstacles: 动态障碍物                                 │
│  - goal: 目标点                                                  │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│              公共前置处理层 (固定流程，非插件)                     │
├─────────────────────────────────────────────────────────────────┤
│  1. BasicDataConverter                                          │
│     - 转换自车状态 (EgoVehicle)                                  │
│     - 转换任务目标 (PlanningTask)                                │
│                                                                 │
│  2. BEVExtractor                                                │
│     - 从 WorldTick 提取 BEV 障碍物                               │
│     - 输出: BEVObstacles (circles, rectangles, polygons)        │
│                                                                 │
│  3. DynamicObstaclePredictor                                    │
│     - 预测动态障碍物轨迹                                          │
│     - 输出: vector<DynamicObstacle>                             │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    PerceptionInput                              │
│  (标准化中间数据 - 所有感知插件的输入)                             │
│  - ego: EgoVehicle                                              │
│  - task: PlanningTask                                           │
│  - bev_obstacles: BEVObstacles (已解析)                         │
│  - dynamic_obstacles: vector<DynamicObstacle> (已解析)          │
│  - raw_world_tick: const WorldTick* (可选)                      │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                  感知插件层 (可扩展)                              │
├─────────────────────────────────────────────────────────────────┤
│  Plugin 1: GridMapBuilderPlugin                                │
│    - 输入: PerceptionInput                                      │
│    - 功能: 从 BEV 障碍物构建栅格占据地图                          │
│    - 输出: context.occupancy_grid                               │
│                                                                 │
│  Plugin 2: ESDFBuilderPlugin                                    │
│    - 输入: PerceptionInput                                      │
│    - 功能: 从 BEV 障碍物构建 ESDF 距离场                         │
│    - 输出: context.custom_data["esdf_map"]                      │
│                                                                 │
│  Plugin 3: PointCloudMapBuilderPlugin                           │
│    - 输入: PerceptionInput                                      │
│    - 功能: 构建点云地图                                          │
│    - 输出: context.custom_data["point_cloud_map"]               │
│                                                                 │
│  Plugin N: [用户自定义插件]                                      │
│    - 输入: PerceptionInput                                      │
│    - 功能: 构建自定义地图表示                                     │
│    - 输出: context.custom_data["..."]                           │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                  planning::PlanningContext                      │
│  (输出 - 供规划器使用)                                            │
│  - ego: EgoVehicle                                              │
│  - task: PlanningTask                                           │
│  - dynamic_obstacles: vector<DynamicObstacle>                   │
│  - occupancy_grid: unique_ptr<OccupancyGrid>                    │
│  - custom_data: map<string, shared_ptr<void>>                   │
└─────────────────────────────────────────────────────────────────┘
```

## 🎯 设计原则

### 1. 职责分离

**公共前置处理层**:
- ✅ 解析原始数据 (`proto::WorldTick`)
- ✅ 提取标准化的中间数据
- ✅ 所有感知插件共享的通用处理
- ❌ 不负责构建特定的地图表示

**感知插件层**:
- ✅ 接收标准化数据 (`PerceptionInput`)
- ✅ 构建特定的地图表示
- ✅ 用户可自定义扩展
- ❌ 不负责解析原始数据

### 2. 数据标准化

所有感知插件接收相同的标准化输入 `PerceptionInput`，包含：
- 已转换的基础数据 (ego, task)
- 已解析的 BEV 障碍物
- 已预测的动态障碍物轨迹
- 可选的原始数据引用

### 3. 插件专注性

每个感知插件专注于构建一种地图表示：
- `GridMapBuilderPlugin` → 栅格占据地图
- `ESDFBuilderPlugin` → ESDF 距离场
- `PointCloudMapBuilderPlugin` → 点云地图
- 用户自定义插件 → 其他地图表示

## 🔧 实现细节

### PerceptionPluginManager 处理流程

```cpp
bool PerceptionPluginManager::process(
    const proto::WorldTick& world_tick,
    planning::PlanningContext& context) {
  
  // ========== 步骤 1: 公共前置处理 ==========
  PerceptionInput input;
  
  // 1.1 转换基础数据
  input.ego = BasicDataConverter::convertEgo(world_tick);
  input.task = BasicDataConverter::convertTask(world_tick);
  input.timestamp = world_tick.stamp();
  input.tick_id = world_tick.tick_id();
  
  // 1.2 提取 BEV 障碍物
  if (!bev_extractor_->extract(world_tick, input.bev_obstacles)) {
    return false;
  }
  
  // 1.3 预测动态障碍物
  if (!obstacle_predictor_->predict(world_tick, input.dynamic_obstacles)) {
    return false;
  }
  
  // 1.4 保存原始数据引用
  input.raw_world_tick = &world_tick;
  
  // 1.5 填充基础上下文
  context.ego = input.ego;
  context.task = input.task;
  context.timestamp = input.timestamp;
  context.dynamic_obstacles = input.dynamic_obstacles;
  
  // ========== 步骤 2: 执行感知插件 ==========
  for (auto& entry : plugins_) {
    if (!entry.enabled) continue;
    
    if (!entry.plugin->process(input, context)) {
      std::cerr << "Plugin failed: " 
                << entry.plugin->getMetadata().name << std::endl;
      return false;
    }
  }
  
  return true;
}
```

### 感知插件实现示例

```cpp
class GridMapBuilderPlugin : public PerceptionPluginInterface {
public:
  bool process(const PerceptionInput& input,
              planning::PlanningContext& context) override {
    // 1. 访问标准化数据
    const auto& bev_obstacles = input.bev_obstacles;
    const auto& ego = input.ego;
    
    // 2. 创建栅格地图
    auto grid = std::make_unique<planning::OccupancyGrid>();
    grid->config.origin = {ego.pose.x - map_width_ / 2,
                          ego.pose.y - map_height_ / 2};
    grid->config.resolution = resolution_;
    grid->config.width = static_cast<int>(map_width_ / resolution_);
    grid->config.height = static_cast<int>(map_height_ / resolution_);
    grid->data.resize(grid->config.width * grid->config.height, 0);
    
    // 3. 从 BEV 障碍物填充栅格地图
    fillGridFromBEVObstacles(bev_obstacles, *grid);
    
    // 4. 膨胀障碍物
    inflateObstacles(*grid, inflation_radius_);
    
    // 5. 输出到规划上下文
    context.occupancy_grid = std::move(grid);
    
    return true;
  }
  
private:
  void fillGridFromBEVObstacles(
      const planning::BEVObstacles& obstacles,
      planning::OccupancyGrid& grid) {
    // 处理圆形障碍物
    for (const auto& circle : obstacles.circles) {
      fillCircle(grid, circle.center, circle.radius);
    }
    
    // 处理矩形障碍物
    for (const auto& rect : obstacles.rectangles) {
      fillRectangle(grid, rect.pose, rect.width, rect.height);
    }
    
    // 处理多边形障碍物
    for (const auto& polygon : obstacles.polygons) {
      fillPolygon(grid, polygon.vertices);
    }
  }
};
```

## 📝 配置示例

```json
{
  "perception": {
    "preprocessing": {
      "bev_extraction": {
        "detection_range": 50.0,
        "confidence_threshold": 0.5,
        "include_static": true,
        "include_dynamic": true
      },
      "dynamic_prediction": {
        "prediction_horizon": 5.0,
        "time_step": 0.1,
        "max_trajectories": 3,
        "prediction_model": "constant_velocity"
      }
    },
    
    "plugins": [
      {
        "name": "GridMapBuilderPlugin",
        "enabled": true,
        "priority": 1,
        "params": {
          "resolution": 0.1,
          "map_width": 100.0,
          "map_height": 100.0,
          "inflation_radius": 0.3
        }
      },
      {
        "name": "ESDFBuilderPlugin",
        "enabled": false,
        "priority": 2,
        "params": {
          "resolution": 0.1,
          "max_distance": 10.0
        }
      }
    ]
  }
}
```

## ✅ 优势

1. **清晰的职责划分**: 前置处理层和插件层各司其职
2. **数据标准化**: 所有插件接收相同格式的输入
3. **易于扩展**: 用户只需关注地图构建逻辑
4. **避免重复**: BEV 提取和动态预测只执行一次
5. **性能优化**: 前置处理结果可被多个插件共享
6. **向后兼容**: 高级插件仍可访问原始数据

## 🚀 添加自定义插件

### 步骤 1: 实现插件接口

```cpp
class MyCustomMapBuilderPlugin : public PerceptionPluginInterface {
public:
  Metadata getMetadata() const override {
    return {
      .name = "MyCustomMapBuilderPlugin",
      .version = "1.0.0",
      .description = "Builds custom map representation",
      .requires_raw_data = false  // 不需要原始数据
    };
  }
  
  bool initialize(const nlohmann::json& config) override {
    // 读取配置...
    return true;
  }
  
  bool process(const PerceptionInput& input,
              planning::PlanningContext& context) override {
    // 使用标准化数据构建自定义地图
    const auto& bev_obstacles = input.bev_obstacles;
    const auto& dynamic_obstacles = input.dynamic_obstacles;
    
    // 构建自定义地图...
    auto custom_map = buildCustomMap(bev_obstacles, dynamic_obstacles);
    
    // 输出到上下文
    context.setCustomData("my_custom_map", custom_map);
    
    return true;
  }
};

REGISTER_PERCEPTION_PLUGIN(MyCustomMapBuilderPlugin)
```

### 步骤 2: 配置文件中启用

```json
{
  "perception": {
    "plugins": [
      {
        "name": "MyCustomMapBuilderPlugin",
        "enabled": true,
        "priority": 3,
        "params": {
          "custom_param": 1.0
        }
      }
    ]
  }
}
```

## 📚 相关文档

- [插件架构设计](PLUGIN_ARCHITECTURE_DESIGN.md) - 完整架构设计
- [插件快速参考](PLUGIN_QUICK_REFERENCE.md) - 开发速查手册
- [配置文件指南](../config/README.md) - 配置文件说明


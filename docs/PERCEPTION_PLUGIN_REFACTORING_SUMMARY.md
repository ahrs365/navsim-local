# 感知插件架构重构总结

**更新日期**: 2025-10-13  
**版本**: v2.0

## 🎯 核心变化

### 一句话总结

**感知插件不再接收原始数据，而是接收标准化的中间数据。**

### 架构对比

#### 原架构 (v1.0)

```
proto::WorldTick → [感知插件] → PlanningContext
```

所有插件都接收原始的 `proto::WorldTick`，需要自己解析数据。

#### 新架构 (v2.0)

```
proto::WorldTick → [前置处理层] → PerceptionInput → [感知插件] → PlanningContext
```

前置处理层解析原始数据，感知插件接收标准化的 `PerceptionInput`。

## 📊 数据流

```
┌──────────────────┐
│ proto::WorldTick │  原始数据
└────────┬─────────┘
         ↓
┌────────────────────────────────┐
│   公共前置处理层 (固定流程)      │
├────────────────────────────────┤
│ • BEVExtractor                 │
│ • DynamicObstaclePredictor     │
│ • BasicDataConverter           │
└────────┬───────────────────────┘
         ↓
┌──────────────────┐
│ PerceptionInput  │  标准化数据
├──────────────────┤
│ • ego            │
│ • task           │
│ • bev_obstacles  │  ← 已解析
│ • dynamic_obs    │  ← 已解析
└────────┬─────────┘
         ↓
┌────────────────────────────────┐
│   感知插件层 (可扩展)            │
├────────────────────────────────┤
│ • GridMapBuilderPlugin         │
│ • ESDFBuilderPlugin            │
│ • [用户自定义插件]              │
└────────┬───────────────────────┘
         ↓
┌──────────────────┐
│ PlanningContext  │  输出
└──────────────────┘
```

## 🔑 关键接口变化

### 感知插件接口

#### 原接口 (v1.0)

```cpp
class PerceptionPluginInterface {
  virtual bool process(
    const proto::WorldTick& world_tick,
    planning::PlanningContext& context) = 0;
};
```

#### 新接口 (v2.0)

```cpp
struct PerceptionInput {
  planning::EgoVehicle ego;
  planning::PlanningTask task;
  planning::BEVObstacles bev_obstacles;  // 已解析
  std::vector<planning::DynamicObstacle> dynamic_obstacles;  // 已解析
  const proto::WorldTick* raw_world_tick;  // 可选
};

class PerceptionPluginInterface {
  virtual bool process(
    const PerceptionInput& input,
    planning::PlanningContext& context) = 0;
};
```

## 📝 配置文件变化

### 原配置 (v1.0)

```json
{
  "perception": {
    "plugins": [
      {"name": "BEVExtractorPlugin", "enabled": true},
      {"name": "DynamicPredictorPlugin", "enabled": true},
      {"name": "GridBuilderPlugin", "enabled": true}
    ]
  }
}
```

### 新配置 (v2.0)

```json
{
  "perception": {
    "preprocessing": {
      "bev_extraction": {
        "detection_range": 50.0,
        "confidence_threshold": 0.5
      },
      "dynamic_prediction": {
        "prediction_horizon": 5.0,
        "time_step": 0.1
      }
    },
    "plugins": [
      {"name": "GridMapBuilderPlugin", "enabled": true}
    ]
  }
}
```

**关键变化**:
- ✅ 新增 `preprocessing` 配置节
- ✅ BEV 提取和动态预测不再是插件

## 🔧 插件开发变化

### 原插件开发 (v1.0)

```cpp
class MyPlugin : public PerceptionPluginInterface {
  bool process(const proto::WorldTick& world_tick,
              planning::PlanningContext& context) override {
    // 需要自己解析原始数据
    const auto& ego = world_tick.ego();
    const auto& obstacles = world_tick.dynamic_obstacles();
    
    // 处理逻辑...
  }
};
```

### 新插件开发 (v2.0)

```cpp
class MyPlugin : public PerceptionPluginInterface {
  bool process(const PerceptionInput& input,
              planning::PlanningContext& context) override {
    // 直接使用已解析的数据
    const auto& ego = input.ego;
    const auto& bev_obstacles = input.bev_obstacles;
    const auto& dynamic_obstacles = input.dynamic_obstacles;
    
    // 专注于地图构建逻辑...
  }
};
```

**优势**:
- ✅ 不需要解析原始数据
- ✅ 专注于地图构建逻辑
- ✅ 代码更简洁

## 📦 插件列表变化

### 原插件列表 (v1.0)

**感知插件**:
- `BEVExtractorPlugin` - BEV 障碍物提取
- `DynamicPredictorPlugin` - 动态障碍物预测
- `GridBuilderPlugin` - 栅格地图构建

### 新插件列表 (v2.0)

**公共前置处理模块** (非插件):
- `BEVExtractor` - BEV 障碍物提取
- `DynamicObstaclePredictor` - 动态障碍物预测
- `BasicDataConverter` - 基础数据转换

**感知插件**:
- `GridMapBuilderPlugin` - 栅格地图构建
- `ESDFBuilderPlugin` - ESDF 距离场构建
- `PointCloudMapBuilderPlugin` - 点云地图构建

## ✅ 优势

1. **职责清晰**
   - 前置处理层：解析原始数据
   - 插件层：构建地图表示

2. **数据标准化**
   - 所有插件接收相同的输入格式
   - 避免重复解析

3. **易于扩展**
   - 用户只需关注地图构建逻辑
   - 不需要处理原始数据解析

4. **性能优化**
   - BEV 提取和动态预测只执行一次
   - 结果被所有插件共享

5. **向后兼容**
   - 高级插件仍可访问原始数据
   - 通过 `input.raw_world_tick` 访问

## 🚀 迁移指南

### 对于插件开发者

如果你已经开发了自定义感知插件，需要进行以下修改：

1. **修改函数签名**
   ```cpp
   // 原代码
   bool process(const proto::WorldTick& world_tick, ...)
   
   // 新代码
   bool process(const PerceptionInput& input, ...)
   ```

2. **使用标准化数据**
   ```cpp
   // 原代码
   const auto& ego = world_tick.ego();
   
   // 新代码
   const auto& ego = input.ego;
   const auto& bev_obstacles = input.bev_obstacles;
   ```

3. **访问原始数据（如果需要）**
   ```cpp
   if (input.hasRawData()) {
     const auto& world_tick = *input.raw_world_tick;
     // 访问原始数据...
   }
   ```

### 对于配置文件

1. **移除 BEV 和动态预测插件**
   ```json
   // 删除这些
   {"name": "BEVExtractorPlugin", ...}
   {"name": "DynamicPredictorPlugin", ...}
   ```

2. **添加前置处理配置**
   ```json
   "preprocessing": {
     "bev_extraction": {...},
     "dynamic_prediction": {...}
   }
   ```

## 📚 相关文档

- [感知插件架构详解](PERCEPTION_PLUGIN_ARCHITECTURE.md)
- [感知架构更新说明](PERCEPTION_ARCHITECTURE_UPDATE.md)
- [插件架构设计](PLUGIN_ARCHITECTURE_DESIGN.md)
- [配置文件指南](../config/README.md)

## 🎉 总结

v2.0 架构通过引入公共前置处理层，实现了：

1. ✅ 清晰的职责划分
2. ✅ 数据标准化
3. ✅ 易于扩展
4. ✅ 性能优化
5. ✅ 向后兼容

这是一个更加合理、清晰、易于扩展的架构设计！


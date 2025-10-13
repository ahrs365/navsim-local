# 感知插件架构更新说明

**更新日期**: 2025-10-13  
**版本**: v2.0  
**状态**: 设计完成

## 📋 更新概述

本次更新重新设计了感知插件架构，将 BEV 提取和动态障碍物预测从插件层移至公共前置处理层，使感知插件专注于地图表示的构建。

## 🎯 更新动机

### 原设计的问题

在原设计中，`BEVExtractorPlugin` 和 `DynamicPredictorPlugin` 被设计为独立的感知插件，存在以下问题：

1. **架构层次混淆**
   - BEV 提取和动态预测是**所有感知插件的公共前置步骤**
   - 不应该与地图构建插件处于同一层次

2. **职责不清晰**
   - 感知插件应该专注于构建特定的地图表示
   - 原始数据解析应该是固定流程，不应该是可选插件

3. **数据流不合理**
   - 所有插件都接收 `proto::WorldTick` 作为输入
   - 每个插件都需要重复解析相同的数据

4. **扩展性差**
   - 用户添加新插件时需要处理原始数据解析
   - 增加了插件开发的复杂度

### 新设计的优势

1. **清晰的职责划分**
   - 前置处理层：解析原始数据 → 标准化中间数据
   - 插件层：标准化数据 → 特定地图表示

2. **数据标准化**
   - 所有插件接收相同的 `PerceptionInput`
   - BEV 障碍物和动态预测已经解析完成

3. **易于扩展**
   - 用户只需关注地图构建逻辑
   - 不需要处理原始数据解析

4. **性能优化**
   - BEV 提取和动态预测只执行一次
   - 结果被所有插件共享

## 🔄 架构对比

### 原架构

```
proto::WorldTick
    ↓
[感知插件层]
├─ BEVExtractorPlugin (插件)
├─ DynamicPredictorPlugin (插件)
├─ GridBuilderPlugin (插件)
└─ [其他插件]
    ↓
PlanningContext
```

**问题**: BEV 提取和动态预测与地图构建处于同一层次

### 新架构

```
proto::WorldTick
    ↓
[公共前置处理层] - 固定流程
├─ BEVExtractor (非插件)
├─ DynamicObstaclePredictor (非插件)
└─ BasicDataConverter (非插件)
    ↓
PerceptionInput (标准化数据)
    ↓
[感知插件层] - 可扩展
├─ GridMapBuilderPlugin
├─ ESDFBuilderPlugin
└─ [用户自定义插件]
    ↓
PlanningContext
```

**优势**: 清晰的分层，职责明确

## 📝 主要变更

### 1. 新增数据结构

#### PerceptionInput

```cpp
struct PerceptionInput {
  // 基础数据
  planning::EgoVehicle ego;
  planning::PlanningTask task;
  double timestamp;
  uint64_t tick_id;
  
  // 已解析的标准化数据
  planning::BEVObstacles bev_obstacles;
  std::vector<planning::DynamicObstacle> dynamic_obstacles;
  
  // 原始数据（可选）
  const proto::WorldTick* raw_world_tick;
};
```

### 2. 感知插件接口变更

#### 原接口

```cpp
class PerceptionPluginInterface {
  virtual bool process(const proto::WorldTick& world_tick,
                      planning::PlanningContext& context) = 0;
};
```

#### 新接口

```cpp
class PerceptionPluginInterface {
  virtual bool process(const PerceptionInput& input,
                      planning::PlanningContext& context) = 0;
};
```

**关键变化**: 输入从 `proto::WorldTick` 改为 `PerceptionInput`

### 3. 插件元信息变更

```cpp
struct Metadata {
  std::string name;
  std::string version;
  std::string description;
  std::string author;
  std::vector<std::string> dependencies;
  bool requires_raw_data = false;  // 新增：是否需要原始数据
};
```

### 4. 公共前置处理模块

#### BEVExtractor

```cpp
class BEVExtractor {
public:
  bool extract(const proto::WorldTick& world_tick,
              planning::BEVObstacles& bev_obstacles);
};
```

#### DynamicObstaclePredictor

```cpp
class DynamicObstaclePredictor {
public:
  bool predict(const proto::WorldTick& world_tick,
              std::vector<planning::DynamicObstacle>& dynamic_obstacles);
};
```

#### BasicDataConverter

```cpp
class BasicDataConverter {
public:
  static planning::EgoVehicle convertEgo(const proto::WorldTick& world_tick);
  static planning::PlanningTask convertTask(const proto::WorldTick& world_tick);
};
```

### 5. PerceptionPluginManager 变更

```cpp
class PerceptionPluginManager {
private:
  // 公共前置处理模块（非插件）
  std::unique_ptr<BEVExtractor> bev_extractor_;
  std::unique_ptr<DynamicObstaclePredictor> obstacle_predictor_;
  
  // 感知插件列表
  std::vector<PluginEntry> plugins_;
  
public:
  bool process(const proto::WorldTick& world_tick,
              planning::PlanningContext& context) {
    // 1. 公共前置处理
    PerceptionInput input;
    bev_extractor_->extract(world_tick, input.bev_obstacles);
    obstacle_predictor_->predict(world_tick, input.dynamic_obstacles);
    // ...
    
    // 2. 执行感知插件
    for (auto& entry : plugins_) {
      entry.plugin->process(input, context);
    }
  }
};
```

### 6. 配置文件结构变更

#### 原配置

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

#### 新配置

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
- 新增 `preprocessing` 配置节
- BEV 提取和动态预测不再是插件

### 7. 感知插件示例变更

#### 原插件示例

```cpp
class GridBuilderPlugin : public PerceptionPluginInterface {
  bool process(const proto::WorldTick& world_tick,
              planning::PlanningContext& context) override {
    // 需要自己解析 world_tick
    const auto& obstacles = world_tick.dynamic_obstacles();
    // ...
  }
};
```

#### 新插件示例

```cpp
class GridMapBuilderPlugin : public PerceptionPluginInterface {
  bool process(const PerceptionInput& input,
              planning::PlanningContext& context) override {
    // 直接使用已解析的数据
    const auto& bev_obstacles = input.bev_obstacles;
    const auto& dynamic_obstacles = input.dynamic_obstacles;
    
    // 专注于地图构建
    fillGridFromBEVObstacles(bev_obstacles, grid);
    // ...
  }
};
```

## 📦 插件列表变更

### 原插件列表

- `BEVExtractorPlugin` - BEV 障碍物提取
- `DynamicPredictorPlugin` - 动态障碍物预测
- `GridBuilderPlugin` - 栅格地图构建

### 新插件列表

**公共前置处理模块**（非插件）:
- `BEVExtractor` - BEV 障碍物提取
- `DynamicObstaclePredictor` - 动态障碍物预测
- `BasicDataConverter` - 基础数据转换

**感知插件**:
- `GridMapBuilderPlugin` - 栅格地图构建
- `ESDFBuilderPlugin` - ESDF 距离场构建
- `PointCloudMapBuilderPlugin` - 点云地图构建

## 🔧 迁移指南

### 对于插件开发者

如果你已经开发了自定义感知插件，需要进行以下修改：

#### 1. 修改 process 函数签名

```cpp
// 原代码
bool process(const proto::WorldTick& world_tick,
            planning::PlanningContext& context) override {
  // ...
}

// 新代码
bool process(const PerceptionInput& input,
            planning::PlanningContext& context) override {
  // ...
}
```

#### 2. 使用标准化数据

```cpp
// 原代码
const auto& ego = world_tick.ego();
const auto& obstacles = world_tick.dynamic_obstacles();

// 新代码
const auto& ego = input.ego;
const auto& bev_obstacles = input.bev_obstacles;
const auto& dynamic_obstacles = input.dynamic_obstacles;
```

#### 3. 访问原始数据（如果需要）

```cpp
// 新代码
if (input.hasRawData()) {
  const auto& world_tick = *input.raw_world_tick;
  // 访问原始数据...
}
```

### 对于配置文件

#### 1. 移除 BEV 和动态预测插件

```json
// 原配置 - 删除这些
{
  "perception": {
    "plugins": [
      {"name": "BEVExtractorPlugin", ...},
      {"name": "DynamicPredictorPlugin", ...}
    ]
  }
}
```

#### 2. 添加前置处理配置

```json
// 新配置 - 添加这个
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
    }
  }
}
```

## ✅ 更新的文档

以下文档已更新以反映新架构：

1. ✅ `PLUGIN_ARCHITECTURE_DESIGN.md` - 第 3.1 节感知插件接口
2. ✅ `PLUGIN_ARCHITECTURE_SUMMARY.md` - 感知插件接口示例
3. ✅ `PLUGIN_QUICK_REFERENCE.md` - 感知插件速查部分
4. ✅ `config/default.json.example` - 默认配置示例
5. ✅ `config/examples/astar_planner.json` - A* 规划器配置示例
6. ✅ `config/examples/minimal.json` - 最小配置示例
7. ✅ `config/README.md` - 配置文件说明
8. ✅ `PERCEPTION_PLUGIN_ARCHITECTURE.md` - 感知插件架构详解（新增）

## 📚 相关文档

- [感知插件架构详解](PERCEPTION_PLUGIN_ARCHITECTURE.md) - 详细架构说明
- [插件架构设计](PLUGIN_ARCHITECTURE_DESIGN.md) - 完整架构设计
- [插件快速参考](PLUGIN_QUICK_REFERENCE.md) - 开发速查手册
- [配置文件指南](../config/README.md) - 配置文件说明

## 🎉 总结

本次架构更新通过引入公共前置处理层，实现了：

1. ✅ **清晰的职责划分** - 前置处理 vs 地图构建
2. ✅ **数据标准化** - 统一的 PerceptionInput 接口
3. ✅ **易于扩展** - 用户只需关注地图构建逻辑
4. ✅ **性能优化** - 避免重复解析
5. ✅ **向后兼容** - 高级插件仍可访问原始数据

这是一个更加合理、清晰、易于扩展的架构设计！


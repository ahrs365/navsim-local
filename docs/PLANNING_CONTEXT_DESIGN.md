# PlanningContext 设计文档

**版本**: 2.0  
**日期**: 2025-10-13

---

## 概述

`PlanningContext` 是感知插件和规划器插件之间的数据接口。它需要支持：

1. **标准数据类型** - 常用的地图表示（栅格地图、BEV 障碍物等）
2. **自定义数据类型** - 用户自定义的地图表示（ESDF、点云、语义地图等）

---

## 数据结构设计

### 完整定义

```cpp
// include/planning/planning_context.hpp
namespace navsim {
namespace planning {

/**
 * @brief 规划上下文
 * 包含规划器所需的所有感知数据
 */
class PlanningContext {
public:
  // ========== 基础数据 (必需) ==========
  
  /**
   * @brief 自车状态
   */
  EgoVehicle ego;
  
  /**
   * @brief 规划任务（目标点、路径点等）
   */
  PlanningTask task;
  
  /**
   * @brief 时间戳
   */
  double timestamp = 0.0;
  
  // ========== 标准感知数据 (可选) ==========
  
  /**
   * @brief 栅格占据地图
   * 由 GridMapBuilderPlugin 填充
   */
  std::unique_ptr<OccupancyGrid> occupancy_grid;
  
  /**
   * @brief 动态障碍物预测
   * 由前置处理层填充
   */
  std::vector<DynamicObstacle> dynamic_obstacles;
  
  /**
   * @brief 车道线信息
   * 由 LaneDetectorPlugin 填充（如果有）
   */
  std::unique_ptr<LaneLines> lane_lines;
  
  // ========== 自定义数据 (扩展) ==========
  
  /**
   * @brief 自定义数据存储
   * 键：数据名称（例如 "esdf_map", "point_cloud", "semantic_map"）
   * 值：数据指针（使用 shared_ptr<void> 实现类型擦除）
   */
  std::map<std::string, std::shared_ptr<void>> custom_data;
  
  // ========== 工具函数 ==========
  
  /**
   * @brief 设置自定义数据
   * @tparam T 数据类型
   * @param name 数据名称
   * @param data 数据指针
   */
  template<typename T>
  void setCustomData(const std::string& name, std::shared_ptr<T> data) {
    custom_data[name] = std::static_pointer_cast<void>(data);
  }
  
  /**
   * @brief 获取自定义数据
   * @tparam T 数据类型
   * @param name 数据名称
   * @return 数据指针，如果不存在或类型不匹配返回 nullptr
   */
  template<typename T>
  std::shared_ptr<T> getCustomData(const std::string& name) const {
    auto it = custom_data.find(name);
    if (it == custom_data.end()) {
      return nullptr;
    }
    return std::static_pointer_cast<T>(it->second);
  }
  
  /**
   * @brief 检查是否有指定的自定义数据
   * @param name 数据名称
   * @return 是否存在
   */
  bool hasCustomData(const std::string& name) const {
    return custom_data.find(name) != custom_data.end();
  }
  
  /**
   * @brief 清空所有数据
   */
  void clear() {
    occupancy_grid.reset();
    lane_lines.reset();
    dynamic_obstacles.clear();
    custom_data.clear();
  }
};

} // namespace planning
} // namespace navsim
```

---

## 使用示例

### 1. 感知插件输出数据

#### 示例 1: 标准数据（栅格地图）

```cpp
// GridMapBuilderPlugin
class GridMapBuilderPlugin : public PerceptionPluginInterface {
public:
  bool process(const PerceptionInput& input,
              planning::PlanningContext& context) override {
    // 创建栅格地图
    auto grid = std::make_unique<planning::OccupancyGrid>();
    
    // 配置地图参数
    grid->config.origin = {input.ego.pose.x - 50.0, input.ego.pose.y - 50.0};
    grid->config.resolution = 0.1;
    grid->config.width = 1000;
    grid->config.height = 1000;
    grid->data.resize(1000 * 1000, 0);
    
    // 填充地图数据
    fillGridFromBEVObstacles(input.bev_obstacles, *grid);
    
    // 输出到标准字段
    context.occupancy_grid = std::move(grid);
    
    return true;
  }
};
```

#### 示例 2: 自定义数据（ESDF 距离场）

```cpp
// 定义 ESDF 数据结构
struct ESDFMap {
  struct Config {
    Point2d origin;
    double resolution;
    int width;
    int height;
  } config;
  
  std::vector<float> distance_data;  // 距离值
  std::vector<Point2d> gradient_data;  // 梯度
};

// ESDFBuilderPlugin
class ESDFBuilderPlugin : public PerceptionPluginInterface {
public:
  bool process(const PerceptionInput& input,
              planning::PlanningContext& context) override {
    // 创建 ESDF 地图
    auto esdf = std::make_shared<ESDFMap>();
    
    // 配置参数
    esdf->config.origin = {input.ego.pose.x - 50.0, input.ego.pose.y - 50.0};
    esdf->config.resolution = 0.1;
    esdf->config.width = 1000;
    esdf->config.height = 1000;
    
    // 计算距离场
    computeESDFFromBEV(input.bev_obstacles, *esdf);
    
    // 输出到自定义数据字段
    context.setCustomData("esdf_map", esdf);
    
    return true;
  }
};
```

#### 示例 3: 自定义数据（点云地图）

```cpp
// 定义点云数据结构
struct PointCloudMap {
  struct Point {
    float x, y, z;
    uint8_t intensity;
  };
  
  std::vector<Point> points;
  double timestamp;
};

// PointCloudMapBuilderPlugin
class PointCloudMapBuilderPlugin : public PerceptionPluginInterface {
public:
  bool process(const PerceptionInput& input,
              planning::PlanningContext& context) override {
    // 创建点云地图
    auto point_cloud = std::make_shared<PointCloudMap>();
    
    // 从 BEV 障碍物生成点云
    generatePointCloudFromBEV(input.bev_obstacles, *point_cloud);
    
    point_cloud->timestamp = input.timestamp;
    
    // 输出到自定义数据字段
    context.setCustomData("point_cloud_map", point_cloud);
    
    return true;
  }
};
```

#### 示例 4: 自定义数据（语义地图）

```cpp
// 定义语义地图数据结构
struct SemanticMap {
  struct Config {
    Point2d origin;
    double resolution;
    int width;
    int height;
  } config;
  
  enum class SemanticLabel : uint8_t {
    UNKNOWN = 0,
    ROAD = 1,
    SIDEWALK = 2,
    BUILDING = 3,
    VEGETATION = 4,
    VEHICLE = 5,
    PEDESTRIAN = 6
  };
  
  std::vector<SemanticLabel> labels;
};

// SemanticMapBuilderPlugin
class SemanticMapBuilderPlugin : public PerceptionPluginInterface {
public:
  bool process(const PerceptionInput& input,
              planning::PlanningContext& context) override {
    auto semantic_map = std::make_shared<SemanticMap>();
    
    // 配置参数
    semantic_map->config.resolution = 0.2;
    semantic_map->config.width = 500;
    semantic_map->config.height = 500;
    
    // 生成语义标签
    generateSemanticLabels(input, *semantic_map);
    
    // 输出到自定义数据字段
    context.setCustomData("semantic_map", semantic_map);
    
    return true;
  }
};
```

### 2. 规划器读取数据

#### 示例 1: 使用标准数据

```cpp
// AStarPlannerPlugin
class AStarPlannerPlugin : public PlannerPluginInterface {
public:
  bool plan(const planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           planning::PlanningResult& result) override {
    // 检查必需数据
    if (!context.occupancy_grid) {
      std::cerr << "[AStarPlanner] Missing occupancy grid" << std::endl;
      return false;
    }
    
    // 使用栅格地图进行 A* 搜索
    const auto& grid = *context.occupancy_grid;
    auto path = astarSearch(context.ego.pose, context.task.goal, grid);
    
    // 生成轨迹
    result.trajectory = generateTrajectory(path);
    result.success = true;
    
    return true;
  }
  
  std::pair<bool, std::string> isAvailable(
      const planning::PlanningContext& context) const override {
    if (!context.occupancy_grid) {
      return {false, "Missing occupancy grid"};
    }
    return {true, ""};
  }
};
```

#### 示例 2: 使用自定义数据（ESDF）

```cpp
// ESDFGradientPlannerPlugin
class ESDFGradientPlannerPlugin : public PlannerPluginInterface {
public:
  bool plan(const planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           planning::PlanningResult& result) override {
    // 获取 ESDF 地图
    auto esdf = context.getCustomData<ESDFMap>("esdf_map");
    if (!esdf) {
      std::cerr << "[ESDFGradientPlanner] Missing ESDF map" << std::endl;
      return false;
    }
    
    // 使用 ESDF 梯度进行规划
    auto path = gradientDescent(context.ego.pose, context.task.goal, *esdf);
    
    // 生成轨迹
    result.trajectory = generateTrajectory(path);
    result.success = true;
    
    return true;
  }
  
  std::pair<bool, std::string> isAvailable(
      const planning::PlanningContext& context) const override {
    if (!context.hasCustomData("esdf_map")) {
      return {false, "Missing ESDF map"};
    }
    return {true, ""};
  }
};
```

#### 示例 3: 使用多种数据

```cpp
// HybridPlannerPlugin
class HybridPlannerPlugin : public PlannerPluginInterface {
public:
  bool plan(const planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           planning::PlanningResult& result) override {
    // 使用栅格地图进行全局路径规划
    if (!context.occupancy_grid) {
      return false;
    }
    auto global_path = astarSearch(context.ego.pose, context.task.goal,
                                   *context.occupancy_grid);
    
    // 如果有 ESDF，使用梯度优化路径
    auto esdf = context.getCustomData<ESDFMap>("esdf_map");
    if (esdf) {
      global_path = optimizePathWithESDFGradient(global_path, *esdf);
    }
    
    // 如果有语义地图，考虑语义信息
    auto semantic_map = context.getCustomData<SemanticMap>("semantic_map");
    if (semantic_map) {
      global_path = adjustPathWithSemantics(global_path, *semantic_map);
    }
    
    // 生成轨迹
    result.trajectory = generateTrajectory(global_path);
    result.success = true;
    
    return true;
  }
};
```

---

## 配置示例

### 配置文件

```json
{
  "perception": {
    "preprocessing": {
      "bev_extraction": {"detection_range": 50.0},
      "dynamic_prediction": {"prediction_horizon": 5.0}
    },
    "plugins": [
      {
        "name": "GridMapBuilderPlugin",
        "enabled": true,
        "priority": 1,
        "params": {
          "resolution": 0.1,
          "map_width": 100.0
        }
      },
      {
        "name": "ESDFBuilderPlugin",
        "enabled": true,
        "priority": 2,
        "params": {
          "resolution": 0.1,
          "max_distance": 10.0
        }
      },
      {
        "name": "SemanticMapBuilderPlugin",
        "enabled": true,
        "priority": 3,
        "params": {
          "resolution": 0.2
        }
      }
    ]
  },
  "planning": {
    "primary_planner": "HybridPlannerPlugin",
    "fallback_planner": "AStarPlannerPlugin",
    "planners": {
      "HybridPlannerPlugin": {
        "use_esdf_optimization": true,
        "use_semantic_adjustment": true
      },
      "AStarPlannerPlugin": {
        "heuristic_weight": 1.0
      }
    }
  }
}
```

---

## 设计优势

### 1. 灵活性

- ✅ 支持标准数据类型（栅格地图、动态障碍物）
- ✅ 支持任意自定义数据类型（ESDF、点云、语义地图）
- ✅ 无需修改 `PlanningContext` 定义即可添加新数据类型

### 2. 类型安全

- ✅ 使用模板函数 `setCustomData<T>()` 和 `getCustomData<T>()` 保证类型安全
- ✅ 类型不匹配时返回 `nullptr`，避免崩溃

### 3. 易用性

- ✅ 标准数据使用直接字段访问（`context.occupancy_grid`）
- ✅ 自定义数据使用简单的 API（`context.setCustomData()`, `context.getCustomData()`）
- ✅ 提供 `hasCustomData()` 检查数据是否存在

### 4. 性能

- ✅ 使用 `shared_ptr` 避免数据拷贝
- ✅ 使用 `std::map` 快速查找（O(log n)）
- ✅ 类型擦除（`void*`）避免虚函数开销

---

## 最佳实践

### 1. 数据命名规范

使用清晰的命名约定：

```cpp
// 推荐
context.setCustomData("esdf_map", esdf);
context.setCustomData("point_cloud_map", point_cloud);
context.setCustomData("semantic_map", semantic_map);

// 不推荐
context.setCustomData("data1", esdf);
context.setCustomData("my_data", point_cloud);
```

### 2. 数据类型文档化

在插件元信息中说明输出的数据类型：

```cpp
Metadata getMetadata() const override {
  return {
    .name = "ESDFBuilderPlugin",
    .description = "Builds ESDF distance field. "
                   "Output: custom_data['esdf_map'] -> shared_ptr<ESDFMap>",
    // ...
  };
}
```

### 3. 数据依赖检查

规划器应该检查必需数据是否存在：

```cpp
std::pair<bool, std::string> isAvailable(
    const planning::PlanningContext& context) const override {
  if (!context.hasCustomData("esdf_map")) {
    return {false, "Missing ESDF map (required by ESDFGradientPlanner)"};
  }
  return {true, ""};
}
```

---

## 总结

`PlanningContext` 通过 **固定字段 + 自定义数据** 的设计，实现了：

1. ✅ **标准化** - 常用数据类型有固定字段
2. ✅ **可扩展** - 自定义数据支持任意类型
3. ✅ **类型安全** - 模板函数保证类型安全
4. ✅ **易用性** - 简单的 API
5. ✅ **性能** - 避免拷贝，快速查找

这种设计使得新增感知插件可以输出任意类型的地图表示，而无需修改核心数据结构！


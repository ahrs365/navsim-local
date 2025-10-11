# NavSim-Local 配置指南

## 概述

`navsim-local` 目前**没有独立的配置文件**（如 `.json`、`.yaml` 等），所有配置都是在代码中硬编码的。配置主要在以下位置：

## 配置位置

### 1. 主程序配置 (`src/main.cpp`)

**位置**：第 65-69 行

```cpp
// 初始化算法管理器
navsim::AlgorithmManager::Config algo_config;
algo_config.primary_planner = "StraightLinePlanner";  // 可以从命令行参数配置
algo_config.enable_occupancy_grid = true;
algo_config.enable_bev_obstacles = true;
algo_config.verbose_logging = false;  // 可以通过环境变量控制
```

**可配置项**：
- `primary_planner`: 主规划器类型
- `enable_occupancy_grid`: 是否启用栅格地图
- `enable_bev_obstacles`: 是否启用BEV障碍物提取
- `verbose_logging`: 是否启用详细日志

### 2. 算法管理器配置 (`include/algorithm_manager.hpp`)

**位置**：第 22-36 行

```cpp
struct Config {
  // 感知配置
  bool enable_occupancy_grid = true;
  bool enable_bev_obstacles = true;
  bool enable_dynamic_prediction = true;

  // 规划配置
  std::string primary_planner = "StraightLinePlanner";
  std::string fallback_planner = "StraightLinePlanner";
  bool enable_planner_fallback = true;

  // 性能配置
  double max_computation_time_ms = 25.0;  // 最大计算时间
  bool verbose_logging = false;           // 详细日志
};
```

**配置说明**：

| 配置项 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `enable_occupancy_grid` | bool | true | 启用栅格地图构建 |
| `enable_bev_obstacles` | bool | true | 启用BEV障碍物提取 |
| `enable_dynamic_prediction` | bool | true | 启用动态障碍物预测 |
| `primary_planner` | string | "StraightLinePlanner" | 主规划器名称 |
| `fallback_planner` | string | "StraightLinePlanner" | 备用规划器名称 |
| `enable_planner_fallback` | bool | true | 启用规划器降级 |
| `max_computation_time_ms` | double | 25.0 | 最大计算时间(ms) |
| `verbose_logging` | bool | false | 详细日志输出 |

### 3. 感知模块配置

#### 3.1 栅格地图构建器 (`include/perception_processor.hpp`)

**位置**：第 40-46 行

```cpp
struct Config {
  double resolution = 0.1;      // 栅格分辨率 (m/cell)
  double map_width = 100.0;     // 地图宽度 (m)
  double map_height = 100.0;    // 地图高度 (m)
  uint8_t obstacle_cost = 100;  // 障碍物代价值
  double inflation_radius = 0.5; // 膨胀半径 (m)
};
```

**实际使用**（`src/algorithm_manager.cpp` 第 147-154 行）：

```cpp
if (config_.enable_occupancy_grid) {
  perception::OccupancyGridBuilder::Config grid_config;
  grid_config.resolution = 0.1;
  grid_config.map_width = 100.0;
  grid_config.map_height = 100.0;
  grid_config.inflation_radius = 0.3;  // 实际使用 0.3m

  auto grid_builder = std::make_unique<perception::OccupancyGridBuilder>(grid_config);
  perception_pipeline_->addProcessor(std::move(grid_builder), true);
}
```

#### 3.2 BEV障碍物提取器

**位置**：第 72-76 行

```cpp
struct Config {
  double detection_range = 50.0;    // 检测范围 (m)
  double confidence_threshold = 0.5; // 置信度阈值
  bool track_dynamic_only = false;   // 是否只跟踪动态障碍物
};
```

**实际使用**（`src/algorithm_manager.cpp` 第 157-164 行）：

```cpp
if (config_.enable_bev_obstacles) {
  perception::BEVObstacleExtractor::Config bev_config;
  bev_config.detection_range = 30.0;  // 30m 检测范围
  bev_config.confidence_threshold = 0.5;

  auto bev_extractor = std::make_unique<perception::BEVObstacleExtractor>(bev_config);
  perception_pipeline_->addProcessor(std::move(bev_extractor), true);
}
```

#### 3.3 动态障碍物预测器

**位置**：第 105-110 行

```cpp
struct Config {
  double prediction_horizon = 5.0;   // 预测时域 (s)
  double time_step = 0.1;           // 时间步长 (s)
  int max_trajectories = 3;         // 每个障碍物最大轨迹数
  std::string prediction_model = "constant_velocity"; // 预测模型
};
```

**实际使用**（`src/algorithm_manager.cpp` 第 166-173 行）：

```cpp
if (config_.enable_dynamic_prediction) {
  perception::DynamicObstaclePredictor::Config pred_config;
  pred_config.prediction_horizon = 3.0;  // 3秒预测时域
  pred_config.prediction_model = "constant_velocity";

  auto predictor = std::make_unique<perception::DynamicObstaclePredictor>(pred_config);
  perception_pipeline_->addProcessor(std::move(predictor), true);
}
```

### 4. 规划模块配置

#### 4.1 A* 规划器配置

**位置**：`include/planner_interface.hpp`

```cpp
struct Config {
  double max_planning_time_ms = 20.0;
  double waypoint_spacing = 0.5;
  double goal_tolerance = 0.3;
  double heuristic_weight = 1.0;
  int max_iterations = 10000;
};
```

## 如何修改配置

### 方法1：直接修改源代码

修改 `src/main.cpp` 或 `src/algorithm_manager.cpp` 中的配置值，然后重新编译：

```bash
cd navsim-local
mkdir -p build && cd build
cmake ..
make
```

### 方法2：通过命令行参数（需要扩展）

当前只支持 WebSocket URL 和 Room ID：

```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

**建议扩展**：可以添加更多命令行参数，例如：

```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo \
  --planner=AStarPlanner \
  --verbose \
  --detection-range=50.0
```

### 方法3：通过环境变量（需要实现）

可以添加环境变量支持：

```bash
export NAVSIM_VERBOSE=1
export NAVSIM_PLANNER=AStarPlanner
export NAVSIM_DETECTION_RANGE=50.0
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

### 方法4：通过配置文件（推荐，需要实现）

创建 `config.json` 或 `config.yaml`：

```json
{
  "algorithm": {
    "primary_planner": "AStarPlanner",
    "enable_occupancy_grid": true,
    "enable_bev_obstacles": true,
    "verbose_logging": true
  },
  "perception": {
    "occupancy_grid": {
      "resolution": 0.1,
      "map_width": 100.0,
      "map_height": 100.0,
      "inflation_radius": 0.3
    },
    "bev_obstacles": {
      "detection_range": 30.0,
      "confidence_threshold": 0.5
    },
    "dynamic_prediction": {
      "prediction_horizon": 3.0,
      "time_step": 0.1,
      "prediction_model": "constant_velocity"
    }
  },
  "planning": {
    "max_planning_time_ms": 20.0,
    "waypoint_spacing": 0.5,
    "goal_tolerance": 0.3
  }
}
```

## 当前配置总览

### 默认配置值

```
算法管理器:
  - primary_planner: "StraightLinePlanner"
  - enable_occupancy_grid: true
  - enable_bev_obstacles: true
  - enable_dynamic_prediction: true
  - max_computation_time_ms: 25.0
  - verbose_logging: false

栅格地图:
  - resolution: 0.1 m/cell
  - map_width: 100.0 m
  - map_height: 100.0 m
  - inflation_radius: 0.3 m
  - obstacle_cost: 100

BEV障碍物:
  - detection_range: 30.0 m
  - confidence_threshold: 0.5

动态预测:
  - prediction_horizon: 3.0 s
  - time_step: 0.1 s
  - prediction_model: "constant_velocity"

规划器:
  - max_planning_time_ms: 20.0 ms
  - waypoint_spacing: 0.5 m
  - goal_tolerance: 0.3 m
```

## 配置文件实现建议

如果要添加配置文件支持，建议使用 `nlohmann/json` 库（已包含在项目中）：

### 1. 创建配置加载器

```cpp
// config_loader.hpp
#pragma once
#include <string>
#include "algorithm_manager.hpp"

namespace navsim {

class ConfigLoader {
public:
  static bool loadFromFile(const std::string& filepath, 
                          AlgorithmManager::Config& config);
  static bool saveToFile(const std::string& filepath, 
                        const AlgorithmManager::Config& config);
};

} // namespace navsim
```

### 2. 修改 main.cpp

```cpp
// 尝试加载配置文件
navsim::AlgorithmManager::Config algo_config;
if (argc >= 4) {
  std::string config_file = argv[3];
  if (!navsim::ConfigLoader::loadFromFile(config_file, algo_config)) {
    std::cerr << "Failed to load config from: " << config_file << std::endl;
    // 使用默认配置
  }
} else {
  // 使用默认配置
  algo_config.primary_planner = "StraightLinePlanner";
  algo_config.enable_occupancy_grid = true;
  algo_config.enable_bev_obstacles = true;
  algo_config.verbose_logging = false;
}
```

## 总结

- ✅ **当前状态**：所有配置都在代码中硬编码
- ✅ **配置位置**：主要在 `src/main.cpp` 和 `src/algorithm_manager.cpp`
- ⚠️ **修改方式**：需要修改源代码并重新编译
- 💡 **建议改进**：添加配置文件支持（JSON/YAML）

## 相关文件

- `src/main.cpp` - 主程序入口，初始化配置
- `include/algorithm_manager.hpp` - 算法管理器配置定义
- `src/algorithm_manager.cpp` - 算法管理器配置实现
- `include/perception_processor.hpp` - 感知模块配置定义
- `include/planner_interface.hpp` - 规划器配置定义


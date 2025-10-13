# NavSim-Local 插件化架构系统

**版本**: 2.0  
**状态**: 设计完成，待实施  
**最新更新**: 2025-10-13

---

## 📋 目录

- [项目概述](#项目概述)
- [架构设计](#架构设计)
- [数据流向](#数据流向)
- [工程结构](#工程结构)
- [快速开始](#快速开始)
- [开发指南](#开发指南)
- [配置指南](#配置指南)
- [常见使用场景](#常见使用场景)
- [故障排查](#故障排查)
- [文档索引](#文档索引)

---

## 项目概述

### 重构目标

将 navsim-local 项目重构为**插件化架构**，实现以下目标：

1. **感知处理模块插件化**
   - 支持用户自定义感知数据转换插件
   - 插件可独立编译和动态加载
   - 清晰的插件接口定义

2. **规划器模块插件化**
   - 支持用户新增和适配不同的规划器
   - 支持运行时选择不同的规划器
   - 统一的规划器接口

3. **插件管理机制**
   - 工厂模式 + 注册机制
   - 编译时注册（简单、快速、类型安全）
   - 可选的运行时动态加载

4. **配置系统**
   - JSON 格式配置文件
   - 支持选择和配置插件
   - 配置优先级：命令行 > 环境变量 > 配置文件 > 默认值

### 重构动机

**原系统的问题**:
- 感知和规划模块耦合度高，难以扩展
- 添加新算法需要修改核心代码
- 缺乏统一的配置管理
- 用户难以自定义算法

**插件化架构的优势**:
- ✅ **可扩展性**: 用户可轻松添加自定义插件
- ✅ **可配置性**: 通过配置文件选择和配置插件
- ✅ **模块化**: 清晰的模块划分和接口定义
- ✅ **易维护**: 插件独立开发和测试
- ✅ **向后兼容**: 现有功能迁移为插件，保持兼容

### 设计优势总结

| 特性 | 原系统 | 插件化系统 |
|------|--------|-----------|
| 添加新算法 | 修改核心代码 | 编写插件，无需修改核心 |
| 算法选择 | 编译时固定 | 运行时通过配置选择 |
| 配置管理 | 硬编码 | JSON 配置文件 |
| 扩展性 | 低 | 高 |
| 维护性 | 中 | 高 |
| 测试隔离 | 难 | 易 |

---

## 架构设计

### 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                      AlgorithmManager                           │
│                   (算法管理器 - 核心协调)                          │
└────────────┬────────────────────────────────────┬───────────────┘
             │                                    │
             ↓                                    ↓
┌────────────────────────────┐      ┌────────────────────────────┐
│  PerceptionPluginManager   │      │   PlannerPluginManager     │
│    (感知插件管理器)          │      │     (规划器插件管理器)      │
└────────────┬───────────────┘      └────────────┬───────────────┘
             │                                    │
             ↓                                    ↓
┌────────────────────────────┐      ┌────────────────────────────┐
│   公共前置处理层 (固定)      │      │      规划器插件层           │
├────────────────────────────┤      ├────────────────────────────┤
│ • BEVExtractor             │      │ • StraightLinePlanner      │
│ • DynamicObstaclePredictor │      │ • AStarPlanner             │
│ • BasicDataConverter       │      │ • OptimizationPlanner      │
└────────────┬───────────────┘      │ • [用户自定义规划器]        │
             │                      └────────────────────────────┘
             ↓
┌────────────────────────────┐
│      感知插件层 (可扩展)     │
├────────────────────────────┤
│ • GridMapBuilderPlugin     │
│ • ESDFBuilderPlugin        │
│ • PointCloudMapBuilder     │
│ • [用户自定义感知插件]      │
└────────────────────────────┘
```

### 模块划分和职责

#### 1. 公共前置处理层

**职责**: 解析原始数据，生成标准化中间数据

| 模块 | 职责 | 输入 | 输出 |
|------|------|------|------|
| **BEVExtractor** | 提取 BEV 障碍物 | `proto::WorldTick` | `BEVObstacles` |
| **DynamicObstaclePredictor** | 预测动态障碍物轨迹 | `proto::WorldTick` | `vector<DynamicObstacle>` |
| **BasicDataConverter** | 转换基础数据 | `proto::WorldTick` | `EgoVehicle`, `PlanningTask` |

**特点**:
- 固定流程，不是插件
- 所有感知插件共享其输出
- 只执行一次，避免重复计算

#### 2. 感知插件层

**职责**: 从标准化数据构建特定的地图表示

| 插件 | 功能 | 输入 | 输出 |
|------|------|------|------|
| **GridMapBuilderPlugin** | 构建栅格占据地图 | `PerceptionInput` | `context.occupancy_grid` |
| **ESDFBuilderPlugin** | 构建 ESDF 距离场 | `PerceptionInput` | `context.custom_data["esdf"]` |
| **PointCloudMapBuilder** | 构建点云地图 | `PerceptionInput` | `context.custom_data["pointcloud"]` |
| **[用户自定义]** | 自定义地图表示 | `PerceptionInput` | `context.custom_data[...]` |

**特点**:
- 可扩展，用户可添加自定义插件
- 接收标准化的 `PerceptionInput`
- 专注于地图构建逻辑

#### 3. 规划器插件层

**职责**: 根据规划上下文生成轨迹

| 插件 | 类型 | 必需数据 | 特点 |
|------|------|---------|------|
| **StraightLinePlanner** | 几何 | 无 | 简单、快速、降级方案 |
| **AStarPlanner** | 搜索 | `occupancy_grid` | 全局路径规划 |
| **OptimizationPlanner** | 优化 | `bev_obstacles` | 平滑轨迹、考虑动力学 |
| **[用户自定义]** | 自定义 | 自定义 | 用户算法 |

**特点**:
- 可扩展，用户可添加自定义规划器
- 支持降级机制（主规划器失败时使用降级规划器）
- 统一的接口定义

#### 4. 插件管理机制

**PluginRegistry** (插件注册表):
- 单例模式
- 工厂模式创建插件
- 编译时注册（通过宏）

**PluginManager** (插件管理器):
- 加载和初始化插件
- 管理插件生命周期
- 执行插件（按优先级）

#### 5. 配置系统

**ConfigLoader** (配置加载器):
- 加载 JSON 配置文件
- 支持配置优先级
- 验证配置有效性

---

## 数据流向

### 完整数据流图

```
┌──────────────────┐
│ proto::WorldTick │  ← WebSocket 接收
│  (上游话题)       │
└────────┬─────────┘
         │
         ↓
┌─────────────────────────────────────────────────────────┐
│              公共前置处理层 (固定流程)                     │
├─────────────────────────────────────────────────────────┤
│  1. BasicDataConverter::convertEgo()                    │
│     → EgoVehicle                                        │
│                                                         │
│  2. BasicDataConverter::convertTask()                   │
│     → PlanningTask                                      │
│                                                         │
│  3. BEVExtractor::extract()                             │
│     → BEVObstacles (circles, rectangles, polygons)     │
│                                                         │
│  4. DynamicObstaclePredictor::predict()                 │
│     → vector<DynamicObstacle> (predicted trajectories) │
└────────┬────────────────────────────────────────────────┘
         │
         ↓
┌──────────────────┐
│ PerceptionInput  │  ← 标准化中间数据
├──────────────────┤
│ • ego            │
│ • task           │
│ • bev_obstacles  │
│ • dynamic_obs    │
│ • raw_world_tick │
└────────┬─────────┘
         │
         ↓
┌─────────────────────────────────────────────────────────┐
│              感知插件层 (可扩展)                          │
├─────────────────────────────────────────────────────────┤
│  Plugin 1: GridMapBuilderPlugin::process()             │
│     → context.occupancy_grid                            │
│                                                         │
│  Plugin 2: ESDFBuilderPlugin::process()                 │
│     → context.custom_data["esdf_map"]                   │
│                                                         │
│  Plugin N: [UserPlugin]::process()                      │
│     → context.custom_data[...]                          │
└────────┬────────────────────────────────────────────────┘
         │
         ↓
┌──────────────────────┐
│  PlanningContext     │  ← 规划上下文
├──────────────────────┤
│ • ego                │
│ • task               │
│ • occupancy_grid     │
│ • dynamic_obstacles  │
│ • custom_data        │
└────────┬─────────────┘
         │
         ↓
┌─────────────────────────────────────────────────────────┐
│              规划器插件层 (可扩展)                        │
├─────────────────────────────────────────────────────────┤
│  Primary Planner::plan()                                │
│     → PlanningResult (trajectory, success)              │
│                                                         │
│  If failed → Fallback Planner::plan()                   │
│     → PlanningResult                                    │
└────────┬────────────────────────────────────────────────┘
         │
         ↓
┌──────────────────┐
│ PlanningResult   │  ← 规划结果
├──────────────────┤
│ • trajectory     │
│ • success        │
│ • metadata       │
└────────┬─────────┘
         │
         ↓
┌──────────────────┐
│ proto::PlanUpdate│  → WebSocket 发送
│  (下游话题)       │
└──────────────────┘
```

### 关键数据结构

#### PerceptionInput

```cpp
struct PerceptionInput {
  planning::EgoVehicle ego;                    // 自车状态
  planning::PlanningTask task;                 // 任务目标
  planning::BEVObstacles bev_obstacles;        // BEV 障碍物 (已解析)
  std::vector<planning::DynamicObstacle> dynamic_obstacles;  // 动态障碍物 (已解析)
  const proto::WorldTick* raw_world_tick;      // 原始数据 (可选)
  double timestamp;                            // 时间戳
  uint64_t tick_id;                            // Tick ID
};
```

#### PlanningContext

```cpp
class PlanningContext {
public:
  // 基础数据
  EgoVehicle ego;                              // 自车状态
  PlanningTask task;                           // 规划任务
  double timestamp;                            // 时间戳

  // 标准感知数据
  std::unique_ptr<OccupancyGrid> occupancy_grid;  // 栅格地图
  std::vector<DynamicObstacle> dynamic_obstacles; // 动态障碍物
  std::unique_ptr<LaneLines> lane_lines;       // 车道线

  // 自定义数据（支持任意类型）
  std::map<std::string, std::shared_ptr<void>> custom_data;

  // 工具函数
  template<typename T>
  void setCustomData(const std::string& name, std::shared_ptr<T> data);

  template<typename T>
  std::shared_ptr<T> getCustomData(const std::string& name) const;

  bool hasCustomData(const std::string& name) const;
};
```

**设计说明**:
- **固定字段** - 常用的标准数据类型（栅格地图、动态障碍物）
- **自定义数据** - 使用 `custom_data` 存储任意类型（ESDF、点云、语义地图等）
- **类型安全** - 模板函数保证类型安全
- **易扩展** - 新增感知插件可输出任意类型，无需修改核心结构

**使用示例**:
```cpp
// 感知插件输出自定义数据
auto esdf = std::make_shared<ESDFMap>();
context.setCustomData("esdf_map", esdf);

// 规划器读取自定义数据
auto esdf = context.getCustomData<ESDFMap>("esdf_map");
if (esdf) {
  // 使用 ESDF 进行规划
}
```

详见：[PlanningContext 设计文档](docs/PLANNING_CONTEXT_DESIGN.md)

#### PlanningResult

```cpp
struct PlanningResult {
  std::vector<TrajectoryPoint> trajectory;     // 轨迹点
  bool success;                                // 是否成功
  std::string planner_name;                    // 规划器名称
  double computation_time_ms;                  // 计算时间
  std::map<std::string, double> metadata;      // 元数据
};
```

---

## 工程结构

### 重构后的目录结构

```
navsim-local/
├── README.md                        # 项目主文档
├── PLUGIN_SYSTEM_README.md          # 插件系统文档 (本文档)
├── CMakeLists.txt                   # 主 CMake 配置
│
├── include/                         # 头文件目录
│   ├── algorithm_manager.hpp        # 算法管理器
│   ├── bridge.hpp                   # WebSocket 通信
│   │
│   ├── perception/                  # 感知模块
│   │   ├── preprocessing.hpp        # 前置处理模块
│   │   ├── bev_extractor.hpp
│   │   ├── dynamic_predictor.hpp
│   │   └── basic_converter.hpp
│   │
│   ├── planning/                    # 规划模块
│   │   ├── planning_context.hpp    # 规划上下文
│   │   └── planning_result.hpp     # 规划结果
│   │
│   └── plugin/                      # 插件系统 (新增)
│       ├── perception_input.hpp     # 感知输入数据结构
│       ├── perception_plugin_interface.hpp  # 感知插件接口
│       ├── planner_plugin_interface.hpp     # 规划器插件接口
│       ├── plugin_registry.hpp      # 插件注册表
│       ├── perception_plugin_manager.hpp    # 感知插件管理器
│       ├── planner_plugin_manager.hpp       # 规划器插件管理器
│       └── config_loader.hpp        # 配置加载器
│
├── src/                             # 源文件目录
│   ├── main.cpp                     # 主程序入口
│   ├── algorithm_manager.cpp
│   ├── bridge.cpp
│   │
│   ├── perception/                  # 感知实现
│   │   ├── bev_extractor.cpp
│   │   ├── dynamic_predictor.cpp
│   │   └── basic_converter.cpp
│   │
│   └── plugin/                      # 插件系统实现 (新增)
│       ├── plugin_registry.cpp
│       ├── perception_plugin_manager.cpp
│       ├── planner_plugin_manager.cpp
│       └── config_loader.cpp
│
├── plugins/                         # 插件实现目录 (新增)
│   ├── perception/                  # 感知插件
│   │   ├── CMakeLists.txt
│   │   ├── grid_map_builder_plugin.hpp
│   │   ├── grid_map_builder_plugin.cpp
│   │   ├── esdf_builder_plugin.hpp
│   │   ├── esdf_builder_plugin.cpp
│   │   └── README.md                # 插件开发指南
│   │
│   └── planning/                    # 规划器插件
│       ├── CMakeLists.txt
│       ├── straight_line_planner_plugin.hpp
│       ├── straight_line_planner_plugin.cpp
│       ├── astar_planner_plugin.hpp
│       ├── astar_planner_plugin.cpp
│       ├── optimization_planner_plugin.hpp
│       ├── optimization_planner_plugin.cpp
│       └── README.md                # 插件开发指南
│
├── config/                          # 配置文件目录 (新增)
│   ├── README.md                    # 配置文件说明
│   ├── default.json.example         # 默认配置模板
│   └── examples/                    # 配置示例
│       ├── astar_planner.json
│       ├── minimal.json
│       └── custom_plugin.json
│
├── docs/                            # 文档目录
│   ├── PLUGIN_ARCHITECTURE_DESIGN.md        # 完整架构设计
│   ├── PLUGIN_ARCHITECTURE_SUMMARY.md       # 执行摘要
│   ├── PLUGIN_QUICK_REFERENCE.md            # 快速参考
│   ├── PERCEPTION_PLUGIN_ARCHITECTURE.md    # 感知插件架构
│   ├── PERCEPTION_ARCHITECTURE_UPDATE.md    # 架构更新说明
│   └── PERCEPTION_PLUGIN_REFACTORING_SUMMARY.md  # 重构总结
│
├── tests/                           # 测试目录
│   ├── test_perception_plugins.cpp
│   ├── test_planner_plugins.cpp
│   └── test_plugin_manager.cpp
│
├── proto/                           # Protobuf 消息定义
│   ├── world_tick.proto
│   ├── plan_update.proto
│   └── ego_cmd.proto
│
├── third_party/                     # 第三方库
│   ├── ixwebsocket/
│   └── nlohmann/                    # JSON 库
│
└── build/                           # 构建输出目录
    ├── navsim_algo                  # 可执行文件
    └── ...
```

### 各目录职责说明

| 目录 | 职责 | 说明 |
|------|------|------|
| `include/` | 头文件 | 核心接口定义 |
| `include/plugin/` | 插件系统接口 | 插件接口、注册表、管理器 |
| `src/` | 核心实现 | 算法管理器、通信、前置处理 |
| `src/plugin/` | 插件系统实现 | 插件管理器、配置加载器 |
| `plugins/` | 插件实现 | 感知插件、规划器插件 |
| `config/` | 配置文件 | JSON 配置文件和示例 |
| `docs/` | 文档 | 设计文档、开发指南 |
| `tests/` | 测试 | 单元测试、集成测试 |

### CMake 构建结构

```cmake
# 主 CMakeLists.txt
cmake_minimum_required(VERSION 3.14)
project(navsim_local)

# ========== 插件系统核心库 ==========
add_library(navsim_plugin_system STATIC
    src/plugin/plugin_registry.cpp
    src/plugin/perception_plugin_manager.cpp
    src/plugin/planner_plugin_manager.cpp
    src/plugin/config_loader.cpp
)

# ========== 前置处理模块 ==========
add_library(navsim_preprocessing STATIC
    src/perception/bev_extractor.cpp
    src/perception/dynamic_predictor.cpp
    src/perception/basic_converter.cpp
)

# ========== 感知插件 ==========
add_subdirectory(plugins/perception)

# ========== 规划器插件 ==========
add_subdirectory(plugins/planning)

# ========== 主程序 ==========
add_executable(navsim_algo
    src/main.cpp
    src/algorithm_manager.cpp
    src/bridge.cpp
)

target_link_libraries(navsim_algo
    PRIVATE
      navsim_plugin_system
      navsim_preprocessing
      navsim_perception_plugins
      navsim_planning_plugins
      navsim_proto
      ixwebsocket
      nlohmann_json::nlohmann_json
)
```

---

## 快速开始

### 环境准备

**依赖项**:
- C++17 编译器 (GCC 7+, Clang 5+)
- CMake 3.14+
- Protobuf 3.0+
- nlohmann/json (已包含在 third_party/)
- ixwebsocket (已包含在 third_party/)

**构建步骤**:

```bash
# 1. 克隆仓库
git clone https://github.com/ahrs365/ahrs-simulator.git
cd ahrs-simulator/navsim-local

# 2. 创建构建目录
mkdir -p build
cd build

# 3. 配置 CMake
cmake -DCMAKE_BUILD_TYPE=Release ..

# 4. 编译
cmake --build . -j$(nproc)

# 5. 运行测试
ctest --output-on-failure
```

### 使用默认配置运行

```bash
# 使用默认配置
./build/navsim_algo ws://127.0.0.1:8080/ws demo

# 使用自定义配置
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/my_config.json

# 查看帮助
./build/navsim_algo --help
```

### 命令行参数

```
用法: navsim_algo <websocket_url> <scenario_name> [选项]

位置参数:
  websocket_url       WebSocket 服务器地址 (例如: ws://127.0.0.1:8080/ws)
  scenario_name       场景名称 (例如: demo)

选项:
  --config=<path>     配置文件路径 (默认: config/default.json)
  --verbose           启用详细日志
  --help              显示帮助信息
  --version           显示版本信息

示例:
  ./navsim_algo ws://127.0.0.1:8080/ws demo
  ./navsim_algo ws://127.0.0.1:8080/ws demo --config=config/astar_planner.json
  ./navsim_algo ws://127.0.0.1:8080/ws demo --config=config/minimal.json --verbose
```

---

## 开发指南

### 如何开发感知插件

感知插件从标准化的 `PerceptionInput` 构建特定的地图表示。

#### 完整示例：自定义地图构建插件

**文件**: `plugins/perception/my_custom_map_builder.hpp`

```cpp
#pragma once
#include "plugin/perception_plugin_interface.hpp"

namespace navsim {
namespace plugin {

class MyCustomMapBuilderPlugin : public PerceptionPluginInterface {
public:
  // 1. 元信息
  Metadata getMetadata() const override {
    return {
      .name = "MyCustomMapBuilderPlugin",
      .version = "1.0.0",
      .description = "Builds custom map representation",
      .author = "Your Name",
      .dependencies = {},
      .requires_raw_data = false
    };
  }

  // 2. 初始化
  bool initialize(const nlohmann::json& config) override {
    resolution_ = config.value("resolution", 0.1);
    map_size_ = config.value("map_size", 100.0);
    std::cout << "[MyCustomMapBuilder] Initialized" << std::endl;
    return true;
  }

  // 3. 处理函数 (核心)
  bool process(const PerceptionInput& input,
              planning::PlanningContext& context) override {
    // 访问标准化数据
    const auto& bev_obstacles = input.bev_obstacles;
    const auto& dynamic_obstacles = input.dynamic_obstacles;

    // 构建自定义地图
    auto custom_map = buildCustomMap(bev_obstacles);

    // 输出到上下文
    context.setCustomData("my_custom_map", custom_map);
    return true;
  }

private:
  double resolution_;
  double map_size_;

  std::shared_ptr<void> buildCustomMap(
      const planning::BEVObstacles& obstacles) {
    // 实现地图构建逻辑...
    return nullptr;
  }
};

// 注册插件
REGISTER_PERCEPTION_PLUGIN(MyCustomMapBuilderPlugin)

} // namespace plugin
} // namespace navsim
```

**添加到 CMake** (`plugins/perception/CMakeLists.txt`):

```cmake
add_library(navsim_perception_plugins STATIC
    grid_map_builder_plugin.cpp
    esdf_builder_plugin.cpp
    my_custom_map_builder.cpp  # 新增
)
```

**配置文件** (`config/my_config.json`):

```json
{
  "perception": {
    "preprocessing": {
      "bev_extraction": {"detection_range": 50.0},
      "dynamic_prediction": {"prediction_horizon": 5.0}
    },
    "plugins": [
      {
        "name": "MyCustomMapBuilderPlugin",
        "enabled": true,
        "priority": 1,
        "params": {
          "resolution": 0.2,
          "map_size": 50.0
        }
      }
    ]
  }
}
```

### 如何开发规划器插件

规划器插件从 `PlanningContext` 生成轨迹。

#### 完整示例：自定义规划器

**文件**: `plugins/planning/my_custom_planner.hpp`

```cpp
#pragma once
#include "plugin/planner_plugin_interface.hpp"

namespace navsim {
namespace plugin {

class MyCustomPlannerPlugin : public PlannerPluginInterface {
public:
  // 1. 元信息
  Metadata getMetadata() const override {
    return {
      .name = "MyCustomPlannerPlugin",
      .version = "1.0.0",
      .description = "My custom path planner",
      .type = "custom",
      .author = "Your Name",
      .dependencies = {},
      .required_perception = {"occupancy_grid"}
    };
  }

  // 2. 初始化
  bool initialize(const nlohmann::json& config) override {
    time_step_ = config.value("time_step", 0.1);
    max_velocity_ = config.value("max_velocity", 5.0);
    return true;
  }

  // 3. 规划函数 (核心)
  bool plan(const planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           planning::PlanningResult& result) override {
    auto start_time = std::chrono::steady_clock::now();

    // 生成轨迹
    std::vector<planning::TrajectoryPoint> trajectory;
    // trajectory = generateTrajectory(context);

    // 填充结果
    result.trajectory = trajectory;
    result.success = true;
    result.planner_name = "MyCustomPlannerPlugin";

    auto end_time = std::chrono::steady_clock::now();
    result.computation_time_ms =
        std::chrono::duration<double, std::milli>(end_time - start_time).count();

    return true;
  }

  // 4. 可用性检查
  std::pair<bool, std::string> isAvailable(
      const planning::PlanningContext& context) const override {
    if (!context.occupancy_grid) {
      return {false, "Missing occupancy grid"};
    }
    return {true, ""};
  }

private:
  double time_step_;
  double max_velocity_;
};

// 注册插件
REGISTER_PLANNER_PLUGIN(MyCustomPlannerPlugin)

} // namespace plugin
} // namespace navsim
```

**配置文件**:

```json
{
  "planning": {
    "primary_planner": "MyCustomPlannerPlugin",
    "fallback_planner": "StraightLinePlannerPlugin",
    "enable_fallback": true,
    "planners": {
      "MyCustomPlannerPlugin": {
        "time_step": 0.1,
        "max_velocity": 5.0
      }
    }
  }
}
```

### 插件注册方法

#### 感知插件注册

```cpp
// 在插件类定义后使用宏注册
REGISTER_PERCEPTION_PLUGIN(MyPerceptionPlugin)

// 宏展开后等价于:
namespace {
  static PluginRegistrar<MyPerceptionPlugin>
      registrar_MyPerceptionPlugin("MyPerceptionPlugin");
}
```

#### 规划器插件注册

```cpp
// 在插件类定义后使用宏注册
REGISTER_PLANNER_PLUGIN(MyPlannerPlugin)

// 宏展开后等价于:
namespace {
  static PlannerRegistrar<MyPlannerPlugin>
      registrar_MyPlannerPlugin("MyPlannerPlugin");
}
```

### 插件接口说明

#### 感知插件接口

| 方法 | 必须实现 | 说明 |
|------|---------|------|
| `getMetadata()` | ✅ | 返回插件元信息 |
| `initialize(config)` | ✅ | 初始化插件，读取配置 |
| `process(input, context)` | ✅ | 处理感知数据，构建地图 |
| `reset()` | ❌ | 重置插件状态 (可选) |
| `getStatistics()` | ❌ | 返回统计信息 (可选) |
| `isAvailable()` | ❌ | 检查插件是否可用 (可选) |

#### 规划器插件接口

| 方法 | 必须实现 | 说明 |
|------|---------|------|
| `getMetadata()` | ✅ | 返回插件元信息 |
| `initialize(config)` | ✅ | 初始化插件，读取配置 |
| `plan(context, deadline, result)` | ✅ | 生成轨迹 |
| `isAvailable(context)` | ✅ | 检查是否可用 (必需数据是否存在) |
| `reset()` | ❌ | 重置插件状态 (可选) |

### 测试指南

#### 编写插件单元测试

**文件**: `tests/test_my_custom_plugin.cpp`

```cpp
#include <gtest/gtest.h>
#include "plugins/perception/my_custom_map_builder.hpp"

using namespace navsim::plugin;

TEST(MyCustomMapBuilderTest, Initialization) {
  MyCustomMapBuilderPlugin plugin;
  nlohmann::json config = {
    {"resolution", 0.2},
    {"map_size", 50.0}
  };
  EXPECT_TRUE(plugin.initialize(config));
}

TEST(MyCustomMapBuilderTest, Process) {
  MyCustomMapBuilderPlugin plugin;
  plugin.initialize({});

  PerceptionInput input;
  // 准备输入数据...

  planning::PlanningContext context;
  EXPECT_TRUE(plugin.process(input, context));
  EXPECT_TRUE(context.hasCustomData("my_custom_map"));
}
```

#### 运行测试

```bash
# 编译测试
cd build
cmake --build . --target tests -j$(nproc)

# 运行所有测试
ctest --output-on-failure

# 运行特定测试
./tests/test_my_custom_plugin

# 运行测试并显示详细输出
./tests/test_my_custom_plugin --gtest_filter=MyCustomMapBuilderTest.*
```

---

## 配置指南

### 配置文件结构

完整的配置文件结构：

```json
{
  "version": "1.0",

  "algorithm": {
    "max_computation_time_ms": 25.0,
    "verbose_logging": false
  },

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
      }
    ]
  },

  "planning": {
    "primary_planner": "AStarPlannerPlugin",
    "fallback_planner": "StraightLinePlannerPlugin",
    "enable_fallback": true,
    "planners": {
      "AStarPlannerPlugin": {
        "time_step": 0.1,
        "heuristic_weight": 1.0,
        "step_size": 0.5,
        "max_iterations": 10000
      },
      "StraightLinePlannerPlugin": {
        "time_step": 0.1,
        "default_velocity": 2.0
      }
    }
  },

  "visualization": {
    "enabled": false,
    "send_perception_debug": false,
    "send_planning_debug": false
  }
}
```

### 感知前置处理配置

#### BEV 提取配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `detection_range` | double | 50.0 | 检测范围 (m) |
| `confidence_threshold` | double | 0.5 | 置信度阈值 [0, 1] |
| `include_static` | bool | true | 是否包含静态障碍物 |
| `include_dynamic` | bool | true | 是否包含动态障碍物 |

#### 动态障碍物预测配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `prediction_horizon` | double | 5.0 | 预测时域 (s) |
| `time_step` | double | 0.1 | 时间步长 (s) |
| `max_trajectories` | int | 3 | 每个障碍物最大轨迹数 |
| `prediction_model` | string | "constant_velocity" | 预测模型 |

### 感知插件配置

#### GridMapBuilderPlugin

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `resolution` | double | 0.1 | 栅格分辨率 (m/cell) |
| `map_width` | double | 100.0 | 地图宽度 (m) |
| `map_height` | double | 100.0 | 地图高度 (m) |
| `inflation_radius` | double | 0.3 | 障碍物膨胀半径 (m) |
| `obstacle_cost` | int | 100 | 障碍物代价值 [0, 255] |

#### ESDFBuilderPlugin

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `resolution` | double | 0.1 | 距离场分辨率 (m) |
| `max_distance` | double | 10.0 | 最大距离 (m) |

### 规划器插件配置

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

### 配置文件优先级

配置参数的优先级（从高到低）：

1. **命令行参数** - 最高优先级
   ```bash
   ./navsim_algo ws://... demo --config=my_config.json --verbose
   ```

2. **环境变量**
   ```bash
   export NAVSIM_CONFIG_PATH=config/my_config.json
   ./navsim_algo ws://... demo
   ```

3. **配置文件** - 通过 `--config` 指定
   ```bash
   ./navsim_algo ws://... demo --config=config/my_config.json
   ```

4. **默认值** - 代码中的默认值

---

## 常见使用场景

### 场景 1: 使用 A* 规划器

**适用场景**: 需要全局路径规划，避开障碍物

**配置文件** (`config/astar_planner.json`):

```json
{
  "perception": {
    "preprocessing": {
      "bev_extraction": {"detection_range": 50.0},
      "dynamic_prediction": {"prediction_horizon": 3.0}
    },
    "plugins": [
      {
        "name": "GridMapBuilderPlugin",
        "enabled": true,
        "priority": 1,
        "params": {
          "resolution": 0.1,
          "map_width": 100.0,
          "inflation_radius": 0.5
        }
      }
    ]
  },
  "planning": {
    "primary_planner": "AStarPlannerPlugin",
    "fallback_planner": "StraightLinePlannerPlugin",
    "enable_fallback": true,
    "planners": {
      "AStarPlannerPlugin": {
        "time_step": 0.1,
        "heuristic_weight": 1.0,
        "step_size": 0.5,
        "max_iterations": 10000
      }
    }
  }
}
```

**运行**:
```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/astar_planner.json
```

### 场景 2: 使用优化规划器

**适用场景**: 需要平滑轨迹，考虑动力学约束

**配置文件** (`config/optimization_planner.json`):

```json
{
  "perception": {
    "preprocessing": {
      "bev_extraction": {"detection_range": 50.0},
      "dynamic_prediction": {
        "prediction_horizon": 5.0,
        "max_trajectories": 3
      }
    },
    "plugins": [
      {
        "name": "GridMapBuilderPlugin",
        "enabled": true,
        "params": {"resolution": 0.1}
      }
    ]
  },
  "planning": {
    "primary_planner": "OptimizationPlannerPlugin",
    "fallback_planner": "AStarPlannerPlugin",
    "enable_fallback": true,
    "planners": {
      "OptimizationPlannerPlugin": {
        "time_step": 0.1,
        "max_iterations": 100,
        "smoothness_weight": 1.0,
        "obstacle_weight": 10.0
      }
    }
  }
}
```

### 场景 3: 最小化配置

**适用场景**: 快速测试，最小资源消耗

**配置文件** (`config/minimal.json`):

```json
{
  "perception": {
    "preprocessing": {
      "bev_extraction": {"detection_range": 30.0},
      "dynamic_prediction": {"prediction_horizon": 3.0}
    },
    "plugins": [
      {
        "name": "GridMapBuilderPlugin",
        "enabled": true,
        "params": {
          "resolution": 0.2,
          "map_width": 50.0,
          "map_height": 50.0
        }
      }
    ]
  },
  "planning": {
    "primary_planner": "StraightLinePlannerPlugin",
    "enable_fallback": false,
    "planners": {
      "StraightLinePlannerPlugin": {
        "time_step": 0.1,
        "default_velocity": 2.0
      }
    }
  }
}
```

### 场景 4: 自定义插件配置

**适用场景**: 使用自定义感知插件和规划器

**配置文件** (`config/custom_plugin.json`):

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
        "params": {"resolution": 0.1}
      },
      {
        "name": "MyCustomMapBuilderPlugin",
        "enabled": true,
        "priority": 2,
        "params": {
          "resolution": 0.2,
          "map_size": 50.0
        }
      }
    ]
  },
  "planning": {
    "primary_planner": "MyCustomPlannerPlugin",
    "fallback_planner": "StraightLinePlannerPlugin",
    "enable_fallback": true,
    "planners": {
      "MyCustomPlannerPlugin": {
        "time_step": 0.1,
        "max_velocity": 5.0
      }
    }
  }
}
```

---

## 故障排查

### 常见问题

#### 1. 配置文件未加载

**症状**: 程序使用默认配置，忽略配置文件

**可能原因**:
- 配置文件路径错误
- JSON 格式错误
- 未使用 `--config` 参数

**解决方法**:
```bash
# 检查配置文件是否存在
ls -l config/my_config.json

# 验证 JSON 格式
cat config/my_config.json | python -m json.tool

# 确保使用 --config 参数
./navsim_algo ws://... demo --config=config/my_config.json
```

#### 2. 插件未启用

**症状**: 插件未执行，日志中无插件输出

**可能原因**:
- `enabled` 设置为 `false`
- 插件名称拼写错误
- 插件未注册

**解决方法**:
```json
// 检查配置文件
{
  "perception": {
    "plugins": [
      {
        "name": "GridMapBuilderPlugin",  // 确保名称正确
        "enabled": true,                 // 确保启用
        "params": {...}
      }
    ]
  }
}
```

```bash
# 启用详细日志查看插件加载情况
./navsim_algo ws://... demo --config=config/my_config.json --verbose
```

#### 3. 规划器不可用

**症状**: 规划器降级或失败

**可能原因**:
- 缺少必需的感知数据
- 规划器初始化失败
- 规划器参数配置错误

**解决方法**:
```bash
# 查看日志中的错误信息
./navsim_algo ws://... demo --config=config/my_config.json --verbose 2>&1 | grep -i error

# 检查规划器的必需数据
# 例如 AStarPlanner 需要 occupancy_grid
# 确保 GridMapBuilderPlugin 已启用
```

#### 4. 编译错误：找不到插件

**症状**: 编译时报错 "undefined reference to plugin"

**可能原因**:
- 插件未添加到 CMakeLists.txt
- 插件未注册

**解决方法**:
```cmake
# 检查 plugins/perception/CMakeLists.txt
add_library(navsim_perception_plugins STATIC
    grid_map_builder_plugin.cpp
    my_custom_plugin.cpp  # 确保添加了
)
```

```cpp
// 检查插件文件末尾是否有注册宏
REGISTER_PERCEPTION_PLUGIN(MyCustomPlugin)
```

#### 5. 运行时崩溃

**症状**: 程序运行时崩溃或段错误

**可能原因**:
- 空指针访问
- 数组越界
- 插件未正确初始化

**解决方法**:
```bash
# 使用 gdb 调试
gdb --args ./navsim_algo ws://... demo --config=config/my_config.json
(gdb) run
(gdb) bt  # 查看堆栈跟踪

# 使用 valgrind 检查内存错误
valgrind --leak-check=full ./navsim_algo ws://... demo --config=config/my_config.json
```

### 日志查看方法

#### 启用详细日志

```bash
# 方法 1: 命令行参数
./navsim_algo ws://... demo --verbose

# 方法 2: 配置文件
{
  "algorithm": {
    "verbose_logging": true
  }
}
```

#### 日志级别

| 级别 | 说明 | 示例 |
|------|------|------|
| **ERROR** | 错误信息 | `[ERROR] Plugin initialization failed` |
| **WARN** | 警告信息 | `[WARN] Planner fallback triggered` |
| **INFO** | 一般信息 | `[INFO] Plugin loaded: GridMapBuilder` |
| **DEBUG** | 调试信息 | `[DEBUG] Processing tick 12345` |

#### 过滤日志

```bash
# 只查看错误
./navsim_algo ... 2>&1 | grep ERROR

# 只查看特定插件的日志
./navsim_algo ... 2>&1 | grep GridMapBuilder

# 保存日志到文件
./navsim_algo ... 2>&1 | tee navsim.log
```

### 性能调优建议

#### 1. 感知插件优化

**问题**: 感知处理耗时过长

**优化方法**:
- 减小地图分辨率
  ```json
  {"resolution": 0.2}  // 从 0.1 增加到 0.2
  ```
- 减小地图范围
  ```json
  {"map_width": 50.0, "map_height": 50.0}  // 从 100.0 减小到 50.0
  ```
- 禁用不必要的插件
  ```json
  {"name": "ESDFBuilderPlugin", "enabled": false}
  ```

#### 2. 规划器优化

**问题**: 规划器计算时间超过限制

**优化方法**:
- 减小搜索步长
  ```json
  {"step_size": 1.0}  // 从 0.5 增加到 1.0
  ```
- 减小最大迭代次数
  ```json
  {"max_iterations": 5000}  // 从 10000 减小到 5000
  ```
- 使用更快的规划器
  ```json
  {"primary_planner": "StraightLinePlannerPlugin"}
  ```

#### 3. 前置处理优化

**问题**: BEV 提取或动态预测耗时过长

**优化方法**:
- 减小检测范围
  ```json
  {"detection_range": 30.0}  // 从 50.0 减小到 30.0
  ```
- 减小预测时域
  ```json
  {"prediction_horizon": 3.0}  // 从 5.0 减小到 3.0
  ```
- 减少预测轨迹数
  ```json
  {"max_trajectories": 1}  // 从 3 减小到 1
  ```

#### 4. 性能监控

```bash
# 启用性能统计
{
  "algorithm": {
    "verbose_logging": true
  }
}

# 查看性能日志
./navsim_algo ... --verbose 2>&1 | grep "computation_time"
```

**性能指标**:
- 感知处理时间: < 10ms
- 规划计算时间: < 25ms
- 总处理时间: < 50ms (20Hz)

---

## 文档索引

### 核心设计文档

1. **[插件架构设计](docs/PLUGIN_ARCHITECTURE_DESIGN.md)** ⭐⭐⭐⭐⭐
   - 完整的架构设计方案（1800+ 行）
   - 架构分层、接口设计、插件管理、配置系统、目录结构、实施计划
   - 适合：架构师、技术负责人、需要深入了解设计的开发者

2. **[执行摘要](docs/PLUGIN_ARCHITECTURE_SUMMARY.md)** ⭐⭐⭐⭐
   - 核心要点总结（300 行）
   - 重构目标、核心架构、接口设计、配置系统、实施计划
   - 适合：项目经理、快速了解方案的开发者

3. **[快速参考手册](docs/PLUGIN_QUICK_REFERENCE.md)** ⭐⭐⭐
   - 插件开发速查（280 行）
   - 接口速查、配置速查、代码片段、常见问题
   - 适合：插件开发者日常参考

### 感知插件专题文档

4. **[感知插件架构详解](docs/PERCEPTION_PLUGIN_ARCHITECTURE.md)** ⭐⭐⭐⭐⭐
   - 感知插件架构详细说明（300 行）
   - 数据流图、设计原则、实现细节、配置示例、添加插件指南
   - 适合：感知插件开发者、需要理解感知架构的开发者

5. **[感知架构更新说明](docs/PERCEPTION_ARCHITECTURE_UPDATE.md)** ⭐⭐⭐⭐
   - v2.0 架构更新详细说明（300 行）
   - 更新动机、架构对比、主要变更、迁移指南
   - 适合：所有开发者，了解 v1.0 到 v2.0 的变化

6. **[感知插件重构总结](docs/PERCEPTION_PLUGIN_REFACTORING_SUMMARY.md)** ⭐⭐⭐
   - 快速总结文档（250 行）
   - 核心变化、接口对比、配置对比、迁移指南
   - 适合：快速了解感知插件架构变化

7. **[PlanningContext 设计文档](docs/PLANNING_CONTEXT_DESIGN.md)** ⭐⭐⭐⭐⭐ 🆕
   - PlanningContext 详细设计（300 行）
   - 数据结构定义、使用示例、配置示例、最佳实践
   - 适合：插件开发者，了解如何输出和读取数据

### 配置文件

8. **[默认配置模板](config/default.json.example)** ⭐⭐⭐
   - 完整的默认配置示例
   - 包含所有插件和参数说明

9. **[配置示例](config/examples/)** ⭐⭐⭐
   - `astar_planner.json` - 使用 A* 规划器的配置
   - `minimal.json` - 最小化配置示例
   - `custom_plugin.json` - 自定义插件配置示例

10. **[配置文件说明](config/README.md)** ⭐⭐⭐
    - 配置文件使用指南
    - 参数说明和故障排查

### 实施文档

11. **[重构计划](PLUGIN_REFACTORING_PLAN.md)** ⭐⭐⭐⭐
    - 总体重构计划
    - 文档索引
    - 实施路线图

12. **[重构完成报告](PERCEPTION_REFACTORING_COMPLETE.md)** ⭐⭐⭐
    - 重构完成情况总结
    - 已完成的工作
    - 下一步行动

### 阅读建议

**快速入门路径**:
1. 本文档 (PLUGIN_SYSTEM_README.md) - 了解整体架构
2. [快速参考手册](docs/PLUGIN_QUICK_REFERENCE.md) - 开发插件
3. [配置文件说明](config/README.md) - 配置系统

**深入学习路径**:
1. [插件架构设计](docs/PLUGIN_ARCHITECTURE_DESIGN.md) - 完整设计
2. [感知插件架构详解](docs/PERCEPTION_PLUGIN_ARCHITECTURE.md) - 感知架构
3. [感知架构更新说明](docs/PERCEPTION_ARCHITECTURE_UPDATE.md) - 架构演进

**实施路径**:
1. [重构计划](PLUGIN_REFACTORING_PLAN.md) - 了解计划
2. [插件架构设计](docs/PLUGIN_ARCHITECTURE_DESIGN.md) - 实施细节
3. [重构完成报告](PERCEPTION_REFACTORING_COMPLETE.md) - 验收标准

---

## 总结

NavSim-Local 插件化架构系统提供了：

✅ **可扩展性** - 用户可轻松添加自定义插件
✅ **可配置性** - 通过配置文件选择和配置插件
✅ **模块化** - 清晰的模块划分和接口定义
✅ **易维护** - 插件独立开发和测试
✅ **向后兼容** - 现有功能迁移为插件，保持兼容

**核心设计亮点**:
- 公共前置处理层 + 感知插件层的清晰分层
- 标准化的 `PerceptionInput` 数据接口
- 工厂模式 + 注册机制的插件管理
- JSON 配置文件的灵活配置
- 降级机制的规划器管理

**下一步**:
1. 阅读相关文档，了解详细设计
2. 评审设计方案，收集反馈
3. 开始实施，按照 Phase 1 → Phase 2 → Phase 3 的顺序

---

**文档版本**: 2.0
**最后更新**: 2025-10-13
**维护者**: NavSim Team
**联系方式**: ahrs365@outlook.com


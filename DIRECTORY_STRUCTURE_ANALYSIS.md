# 目录结构分析报告

**日期**: 2025-10-13  
**目的**: 分析当前目录结构，为重构做准备

---

## 当前目录结构

```
navsim-local/
├── include/                         # 头文件目录
│   ├── algorithm_manager.hpp        # [核心] 算法管理器（新旧系统切换）
│   ├── bridge.hpp                   # [核心] WebSocket 通信桥接
│   ├── perception_processor.hpp    # [旧系统] 感知处理器接口
│   ├── planner.hpp                  # [旧系统] 简单规划器（已废弃？）
│   ├── planner_interface.hpp       # [旧系统] 规划器接口和实现
│   ├── planning_context.hpp        # [共享] 规划上下文（新旧系统共用）
│   ├── websocket_visualizer.hpp    # [核心] WebSocket 可视化
│   ├── perception/                  # [新系统] 感知前置处理
│   │   └── preprocessing.hpp
│   └── plugin/                      # [新系统] 插件系统
│       ├── config_loader.hpp
│       ├── perception_input.hpp
│       ├── perception_plugin_interface.hpp
│       ├── perception_plugin_manager.hpp
│       ├── planner_plugin_interface.hpp
│       ├── planner_plugin_manager.hpp
│       ├── planning_result.hpp
│       ├── plugin_init.hpp
│       ├── plugin_metadata.hpp
│       └── plugin_registry.hpp
├── src/                             # 源文件目录
│   ├── algorithm_manager.cpp        # [核心] 算法管理器实现
│   ├── bridge.cpp                   # [核心] WebSocket 通信实现
│   ├── main.cpp                     # [核心] 主程序入口
│   ├── perception_processor.cpp    # [旧系统] 感知处理器实现
│   ├── planner.cpp                  # [旧系统] 简单规划器实现
│   ├── planner_interface.cpp       # [旧系统] 规划器接口实现
│   ├── planning_context.cpp        # [共享] 规划上下文实现
│   ├── websocket_visualizer.cpp    # [核心] WebSocket 可视化实现
│   ├── perception/                  # [新系统] 感知前置处理实现
│   │   ├── basic_converter.cpp
│   │   ├── bev_extractor.cpp
│   │   ├── dynamic_predictor.cpp
│   │   └── preprocessing_pipeline.cpp
│   └── plugin/                      # [新系统] 插件系统实现
│       ├── config_loader.cpp
│       ├── perception_plugin_manager.cpp
│       ├── planner_plugin_manager.cpp
│       └── plugin_init.cpp
├── plugins/                         # [新系统] 具体插件实现
│   ├── perception/
│   │   ├── grid_map_builder_plugin.hpp
│   │   └── grid_map_builder_plugin.cpp
│   └── planning/
│       ├── astar_planner_plugin.hpp
│       ├── astar_planner_plugin.cpp
│       ├── straight_line_planner_plugin.hpp
│       └── straight_line_planner_plugin.cpp
├── tests/                           # 测试文件
│   └── test_plugin_system.cpp
├── config/                          # 配置文件
│   ├── default.json
│   └── examples/
└── proto/                           # Protobuf 定义
    ├── ego_cmd.proto
    ├── plan_update.proto
    └── world_tick.proto
```

---

## 文件分类

### 1. 核心文件（新旧系统共用）

**头文件**:
- `include/algorithm_manager.hpp` - 算法管理器，支持新旧系统切换
- `include/bridge.hpp` - WebSocket 通信桥接
- `include/planning_context.hpp` - 规划上下文（新旧系统共用）
- `include/websocket_visualizer.hpp` - WebSocket 可视化

**源文件**:
- `src/algorithm_manager.cpp`
- `src/bridge.cpp`
- `src/main.cpp`
- `src/planning_context.cpp`
- `src/websocket_visualizer.cpp`

**用途**: 这些文件是系统的核心，负责整体协调和通信

### 2. 旧系统文件（Legacy System）

**头文件**:
- `include/perception_processor.hpp` - 旧的感知处理器接口
  - `OccupancyGridBuilder`
  - `BEVObstacleExtractor`
  - `DynamicObstaclePredictor`
  - `PerceptionPipeline`
- `include/planner_interface.hpp` - 旧的规划器接口
  - `StraightLinePlanner`
  - `AStarPlanner`
  - `OptimizationPlanner`
  - `PlannerManager`
- `include/planner.hpp` - 简单规划器（可能已废弃）

**源文件**:
- `src/perception_processor.cpp`
- `src/planner_interface.cpp`
- `src/planner.cpp`

**用途**: 这些文件是旧系统的实现，用于向后兼容

**状态**: 
- ✅ 仍在使用（通过 `AlgorithmManager` 的 `use_plugin_system=false` 模式）
- ⚠️ `planner.hpp/cpp` 可能已废弃（需要确认）

### 3. 新系统 - 插件框架

**头文件** (`include/plugin/`):
- `perception_plugin_interface.hpp` - 感知插件接口
- `planner_plugin_interface.hpp` - 规划器插件接口
- `plugin_metadata.hpp` - 插件元数据
- `plugin_registry.hpp` - 插件注册表
- `perception_plugin_manager.hpp` - 感知插件管理器
- `planner_plugin_manager.hpp` - 规划器插件管理器
- `perception_input.hpp` - 感知输入数据结构
- `planning_result.hpp` - 规划结果数据结构
- `config_loader.hpp` - 配置加载器
- `plugin_init.hpp` - 插件初始化

**源文件** (`src/plugin/`):
- `perception_plugin_manager.cpp`
- `planner_plugin_manager.cpp`
- `config_loader.cpp`
- `plugin_init.cpp`

**用途**: 插件系统的核心框架

### 4. 新系统 - 前置处理层

**头文件** (`include/perception/`):
- `preprocessing.hpp` - 前置处理接口定义

**源文件** (`src/perception/`):
- `bev_extractor.cpp` - BEV 障碍物提取
- `dynamic_predictor.cpp` - 动态障碍物预测
- `basic_converter.cpp` - 基础数据转换
- `preprocessing_pipeline.cpp` - 前置处理管线

**用途**: 为插件系统提供标准化的输入数据

**注意**: 这些类与旧系统的类名相同但实现不同！

### 5. 新系统 - 具体插件实现

**感知插件** (`plugins/perception/`):
- `grid_map_builder_plugin.hpp/cpp` - 栅格地图构建插件

**规划器插件** (`plugins/planning/`):
- `straight_line_planner_plugin.hpp/cpp` - 直线规划器插件
- `astar_planner_plugin.hpp/cpp` - A* 规划器插件

**用途**: 具体的插件实现

---

## 问题分析

### 问题 1: 目录结构混乱

**现象**:
- `include/` 根目录下有新旧系统的文件混在一起
- `include/perception/` 只有新系统的前置处理
- `include/plugin/` 是新系统的插件框架
- `plugins/` 是独立的插件实现目录

**影响**: 不清楚哪些文件属于哪个系统

### 问题 2: 类名冲突

**现象**:
- 旧系统: `include/perception_processor.hpp` 中的 `BEVObstacleExtractor`, `DynamicObstaclePredictor`
- 新系统: `src/perception/bev_extractor.cpp`, `src/perception/dynamic_predictor.cpp` 中的同名类

**当前解决方案**: 使用前向声明避免头文件冲突

**影响**: 容易混淆，维护困难

### 问题 3: 文件用途不明确

**现象**:
- `planner.hpp/cpp` 的用途不明确
- 不清楚哪些文件可以删除

**影响**: 代码冗余，维护成本高

---

## 文件使用情况分析

### 确认使用的文件

通过 `CMakeLists.txt` 和代码引用分析：

**核心库** (`navsim_planning`):
- ✅ `src/planning_context.cpp`
- ✅ `src/perception_processor.cpp` (旧系统)
- ✅ `src/planner_interface.cpp` (旧系统)
- ✅ `src/algorithm_manager.cpp`
- ✅ `src/websocket_visualizer.cpp`

**插件系统库** (`navsim_plugin_system`):
- ✅ `src/plugin/perception_plugin_manager.cpp`
- ✅ `src/plugin/planner_plugin_manager.cpp`
- ✅ `src/plugin/config_loader.cpp`
- ✅ `src/plugin/plugin_init.cpp`
- ✅ `src/perception/bev_extractor.cpp`
- ✅ `src/perception/dynamic_predictor.cpp`
- ✅ `src/perception/basic_converter.cpp`
- ✅ `src/perception/preprocessing_pipeline.cpp`
- ✅ `plugins/perception/grid_map_builder_plugin.cpp`
- ✅ `plugins/planning/straight_line_planner_plugin.cpp`
- ✅ `plugins/planning/astar_planner_plugin.cpp`

**主程序** (`navsim_algo`):
- ✅ `src/main.cpp`
- ✅ `src/planner.cpp` (?)
- ✅ `src/bridge.cpp`

### 可能未使用的文件

**需要确认**:
- ❓ `include/planner.hpp` / `src/planner.cpp` - 可能已被 `planner_interface.hpp` 替代

---

## 建议的新目录结构

### 方案 A: 按系统分离（推荐）

```
navsim-local/
├── include/
│   ├── core/                        # 核心模块（新旧系统共用）
│   │   ├── algorithm_manager.hpp
│   │   ├── bridge.hpp
│   │   ├── planning_context.hpp
│   │   └── websocket_visualizer.hpp
│   ├── legacy/                      # 旧系统（向后兼容）
│   │   ├── perception_processor.hpp
│   │   ├── planner_interface.hpp
│   │   └── planner_manager.hpp
│   └── plugin/                      # 新系统（插件架构）
│       ├── framework/               # 插件框架
│       │   ├── perception_plugin_interface.hpp
│       │   ├── planner_plugin_interface.hpp
│       │   ├── plugin_metadata.hpp
│       │   ├── plugin_registry.hpp
│       │   ├── perception_plugin_manager.hpp
│       │   ├── planner_plugin_manager.hpp
│       │   ├── config_loader.hpp
│       │   └── plugin_init.hpp
│       ├── data/                    # 数据结构
│       │   ├── perception_input.hpp
│       │   └── planning_result.hpp
│       ├── preprocessing/           # 前置处理
│       │   └── preprocessing.hpp
│       └── plugins/                 # 具体插件（头文件）
│           ├── perception/
│           │   └── grid_map_builder_plugin.hpp
│           └── planning/
│               ├── straight_line_planner_plugin.hpp
│               └── astar_planner_plugin.hpp
├── src/
│   ├── core/
│   │   ├── algorithm_manager.cpp
│   │   ├── bridge.cpp
│   │   ├── main.cpp
│   │   ├── planning_context.cpp
│   │   └── websocket_visualizer.cpp
│   ├── legacy/
│   │   ├── perception_processor.cpp
│   │   └── planner_interface.cpp
│   └── plugin/
│       ├── framework/
│       │   ├── perception_plugin_manager.cpp
│       │   ├── planner_plugin_manager.cpp
│       │   ├── config_loader.cpp
│       │   └── plugin_init.cpp
│       ├── preprocessing/
│       │   ├── bev_extractor.cpp
│       │   ├── dynamic_predictor.cpp
│       │   ├── basic_converter.cpp
│       │   └── preprocessing_pipeline.cpp
│       └── plugins/
│           ├── perception/
│           │   └── grid_map_builder_plugin.cpp
│           └── planning/
│               ├── straight_line_planner_plugin.cpp
│               └── astar_planner_plugin.cpp
├── tests/
├── config/
└── proto/
```

**优点**:
- 新旧系统完全分离，职责清晰
- 便于后续删除旧系统
- 插件系统内部结构清晰（框架、数据、前置处理、插件）

**缺点**:
- 需要大量移动文件
- 需要更新所有 `#include` 路径

---

## 文件用途确认结果

### `planner.hpp/cpp` 分析

**发现**:
- ✅ 在 `CMakeLists.txt` 中被编译到 `navsim_algo`
- ✅ 在 `main.cpp` 中被 include
- ❌ 但在 `main.cpp` 中没有实际使用 `Planner` 类
- ❌ 功能已被 `AlgorithmManager` 替代

**结论**: 可以安全删除

---

## 推荐的重构方案

### 方案 A: 按系统分离（推荐）✅

**优点**:
1. 新旧系统完全分离，职责清晰
2. 便于后续删除旧系统
3. 插件系统内部结构清晰
4. 符合成熟 CMake 工程的最佳实践

**缺点**:
1. 需要移动大量文件
2. 需要更新所有 `#include` 路径
3. 需要仔细测试

**工作量**: 中等（预计 1-2 小时）

---

## 重构执行计划

### 阶段 1: 准备工作

1. ✅ 分析当前目录结构
2. ✅ 确认文件用途
3. [ ] 备份当前代码（git commit）
4. [ ] 创建新目录结构

### 阶段 2: 移动文件

#### 2.1 核心模块
- [ ] 移动 `include/algorithm_manager.hpp` → `include/core/`
- [ ] 移动 `include/bridge.hpp` → `include/core/`
- [ ] 移动 `include/planning_context.hpp` → `include/core/`
- [ ] 移动 `include/websocket_visualizer.hpp` → `include/core/`
- [ ] 移动对应的 `.cpp` 文件到 `src/core/`

#### 2.2 旧系统模块
- [ ] 移动 `include/perception_processor.hpp` → `include/legacy/`
- [ ] 移动 `include/planner_interface.hpp` → `include/legacy/`
- [ ] 移动对应的 `.cpp` 文件到 `src/legacy/`
- [ ] 删除 `include/planner.hpp` 和 `src/planner.cpp`

#### 2.3 插件系统 - 框架
- [ ] 重组 `include/plugin/` → `include/plugin/framework/`
- [ ] 重组 `src/plugin/` → `src/plugin/framework/`

#### 2.4 插件系统 - 数据结构
- [ ] 移动 `include/plugin/perception_input.hpp` → `include/plugin/data/`
- [ ] 移动 `include/plugin/planning_result.hpp` → `include/plugin/data/`

#### 2.5 插件系统 - 前置处理
- [ ] 移动 `include/perception/preprocessing.hpp` → `include/plugin/preprocessing/`
- [ ] 移动 `src/perception/*` → `src/plugin/preprocessing/`

#### 2.6 插件系统 - 具体插件
- [ ] 移动 `plugins/perception/*.hpp` → `include/plugin/plugins/perception/`
- [ ] 移动 `plugins/planning/*.hpp` → `include/plugin/plugins/planning/`
- [ ] 移动 `plugins/perception/*.cpp` → `src/plugin/plugins/perception/`
- [ ] 移动 `plugins/planning/*.cpp` → `src/plugin/plugins/planning/`
- [ ] 删除空的 `plugins/` 目录

### 阶段 3: 更新引用

- [ ] 更新所有 `#include` 路径
- [ ] 更新 `CMakeLists.txt`
- [ ] 更新 `target_include_directories`

### 阶段 4: 测试

- [ ] 编译测试
- [ ] 运行 `test_plugin_system`
- [ ] 运行 `navsim_algo`（如果有测试环境）

### 阶段 5: 清理和文档

- [ ] 删除未使用的文件
- [ ] 更新 `PLUGIN_SYSTEM_README.md`
- [ ] 创建重构完成报告
- [ ] Git commit

---

**分析完成，等待用户确认执行重构**


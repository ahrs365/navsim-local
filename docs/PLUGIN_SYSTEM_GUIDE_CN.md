# NavSim-Local 插件系统详解与规划器插件开发指南

## 📚 目录

1. [插件系统架构概览](#1-插件系统架构概览)
2. [插件系统运行机制](#2-插件系统运行机制)
3. [如何新增规划器插件](#3-如何新增规划器插件)
4. [完整示例：开发自定义规划器](#4-完整示例开发自定义规划器)
5. [常见问题与调试](#5-常见问题与调试)

---

## 1. 插件系统架构概览

### 1.1 整体架构

NavSim-Local 采用**分层插件化架构**，将感知和规划模块解耦，支持运行时动态配置和扩展。

```
┌─────────────────────────────────────────────────────────────┐
│                   AlgorithmManager                          │
│                  (核心算法管理器)                             │
└──────────────┬──────────────────────────┬───────────────────┘
               │                          │
               ↓                          ↓
┌──────────────────────────┐   ┌──────────────────────────────┐
│ PerceptionPluginManager  │   │  PlannerPluginManager        │
│  (感知插件管理器)         │   │   (规划器插件管理器)          │
└──────────────┬───────────┘   └──────────────┬───────────────┘
               │                              │
               ↓                              ↓
┌──────────────────────────┐   ┌──────────────────────────────┐
│   感知插件层              │   │    规划器插件层               │
├──────────────────────────┤   ├──────────────────────────────┤
│ • GridMapBuilder         │   │ • StraightLinePlanner        │
│ • ESDFBuilder            │   │ • AStarPlanner               │
│ • [自定义感知插件]        │   │ • [自定义规划器]              │
└──────────────────────────┘   └──────────────────────────────┘
```

### 1.2 核心组件

| 组件 | 职责 | 文件位置 |
|------|------|---------|
| **PluginRegistry** | 插件注册表，管理插件工厂函数 | `include/plugin/framework/plugin_registry.hpp` |
| **PluginManager** | 插件管理器，负责加载、初始化、执行插件 | `src/plugin/framework/*_plugin_manager.cpp` |
| **DynamicPluginLoader** | 动态插件加载器，支持运行时加载 .so 文件 | `src/plugin/framework/dynamic_plugin_loader.cpp` |
| **ConfigLoader** | 配置加载器，从 JSON 读取配置 | `src/plugin/framework/config_loader.cpp` |
| **PlannerPluginInterface** | 规划器插件接口（纯虚基类） | `include/plugin/framework/planner_plugin_interface.hpp` |

### 1.3 数据流向

```
proto::WorldTick (WebSocket输入)
    ↓
[公共前置处理层] → PerceptionInput (标准化数据)
    ↓
[感知插件层] → PlanningContext (规划上下文)
    ↓
[规划器插件] → PlanningResult (轨迹)
    ↓
proto::PlanUpdate (WebSocket输出)
```

---

## 2. 插件系统运行机制

### 2.1 插件注册机制

NavSim-Local 支持**两种插件注册方式**：

#### 方式1: 静态注册（编译时）

每个插件通过 `register.cpp` 文件在程序启动时自动注册：

```cpp
// plugins/planning/astar/src/register.cpp
namespace navsim::plugins::planning {

void registerAStarPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "AStarPlanner",  // 插件名称
        []() -> std::shared_ptr<plugin::PlannerPluginInterface> {
          return std::make_shared<AStarPlannerPlugin>();  // 工厂函数
        });
    registered = true;
  }
}

} // namespace

// 导出 C 风格函数供动态加载
extern "C" {
  void registerAStarPlannerPlugin() {
    navsim::plugins::planning::registerAStarPlannerPlugin();
  }
}

// 静态初始化器（用于静态链接）
namespace {
struct AStarPlannerPluginInitializer {
  AStarPlannerPluginInitializer() {
    navsim::plugins::planning::registerAStarPlannerPlugin();
  }
};
static AStarPlannerPluginInitializer g_astar_planner_initializer;
}
```

**关键点**：
- 静态初始化器在 `main()` 之前执行，自动注册插件
- 工厂函数返回插件实例的智能指针
- 使用单例模式的 `PlannerPluginRegistry` 管理所有插件

#### 方式2: 动态加载（运行时）

通过 `DynamicPluginLoader` 在运行时加载 `.so` 文件：

```cpp
// src/plugin/framework/dynamic_plugin_loader.cpp
bool DynamicPluginLoader::loadPlugin(const std::string& plugin_name, 
                                     const std::string& library_path) {
  // 1. 使用 dlopen 加载共享库
  void* handle = dlopen(lib_path.c_str(), RTLD_LAZY | RTLD_LOCAL);
  
  // 2. 查找注册函数（例如: registerAStarPlannerPlugin）
  std::string register_func_name = "register" + plugin_name + "Plugin";
  RegisterFunc register_func = (RegisterFunc)dlsym(handle, register_func_name.c_str());
  
  // 3. 调用注册函数
  if (register_func) {
    register_func();  // 将插件注册到 Registry
  }
  
  return true;
}
```

### 2.2 插件加载流程

```cpp
// src/core/algorithm_manager.cpp
void AlgorithmManager::setupPluginSystem() {
  // 步骤1: 初始化所有静态插件
  plugin::initializeAllPlugins();
  
  // 步骤2: 动态加载插件（从配置文件）
  plugin::DynamicPluginLoader plugin_loader;
  plugin_loader.addSearchPath("./build/plugins");
  int loaded_count = plugin_loader.loadPluginsFromConfig("config/default.json");
  
  // 步骤3: 加载规划器插件
  std::string primary_planner = "AStarPlanner";  // 从配置读取
  std::string fallback_planner = "StraightLinePlanner";
  
  planner_manager_.loadPlanners(primary_planner, fallback_planner, 
                                true, planner_configs);
  
  // 步骤4: 初始化插件
  planner_manager_.initialize();
}
```

### 2.3 规划器执行流程

```cpp
// src/plugin/framework/planner_plugin_manager.cpp
bool PlannerPluginManager::plan(const planning::PlanningContext& context,
                                std::chrono::milliseconds deadline,
                                PlanningResult& result) {
  // 1. 尝试使用主规划器
  if (tryPlan(primary_planner_, primary_planner_name_, context, deadline, result)) {
    stats_.primary_success++;
    return true;
  }
  
  // 2. 主规划器失败，尝试降级规划器
  if (enable_fallback_ && fallback_planner_) {
    if (tryPlan(fallback_planner_, fallback_planner_name_, context, deadline, result)) {
      stats_.fallback_success++;
      return true;
    }
  }
  
  return false;
}

bool PlannerPluginManager::tryPlan(...) {
  // 1. 检查规划器是否可用
  auto [available, reason] = planner->isAvailable(context);
  if (!available) {
    return false;
  }
  
  // 2. 执行规划
  bool success = planner->plan(context, deadline, result);
  
  return success;
}
```

### 2.4 配置系统

配置文件 `config/default.json` 控制插件的加载和参数：

```json
{
  "planning": {
    "primary_planner": "AStarPlanner",      // 主规划器
    "fallback_planner": "StraightLinePlanner",  // 降级规划器
    "enable_fallback": true,                // 启用降级机制
    "planners": {
      "AStarPlanner": {                     // 规划器参数
        "time_step": 0.1,
        "heuristic_weight": 1.2,
        "step_size": 0.5,
        "max_iterations": 10000
      }
    }
  }
}
```

---

## 3. 如何新增规划器插件

### 3.1 开发步骤总览

1. ✅ 创建插件目录结构
2. ✅ 实现插件接口
3. ✅ 编写注册代码
4. ✅ 配置 CMake 构建
5. ✅ 添加配置文件
6. ✅ 编译和测试

### 3.2 必须实现的接口

所有规划器插件必须继承 `PlannerPluginInterface` 并实现以下方法：

| 方法 | 必须实现 | 说明 |
|------|---------|------|
| `getMetadata()` | ✅ | 返回插件元数据（名称、版本、类型等） |
| `initialize(config)` | ✅ | 初始化插件，读取配置参数 |
| `plan(context, deadline, result)` | ✅ | 核心规划逻辑，生成轨迹 |
| `isAvailable(context)` | ✅ | 检查规划器是否可用（必需数据是否存在） |
| `reset()` | ❌ | 重置插件状态（可选） |
| `getStatistics()` | ❌ | 返回统计信息（可选） |

### 3.3 插件接口详解

#### 3.3.1 `getMetadata()` - 插件元数据

```cpp
plugin::PlannerPluginMetadata getMetadata() const override {
  return {
    .name = "MyPlanner",              // 插件名称（必须与注册名称一致）
    .version = "1.0.0",               // 版本号
    .description = "My custom planner",  // 描述
    .type = "search",                 // 类型: search/optimization/geometric
    .author = "Your Name",            // 作者
    .can_be_fallback = false,         // 是否可作为降级规划器
    .required_perception = {"occupancy_grid"}  // 必需的感知数据
  };
}
```

#### 3.3.2 `initialize(config)` - 初始化

```cpp
bool initialize(const nlohmann::json& config) override {
  // 读取配置参数（带默认值）
  time_step_ = config.value("time_step", 0.1);
  max_velocity_ = config.value("max_velocity", 5.0);
  
  // 验证参数
  if (time_step_ <= 0) {
    std::cerr << "[MyPlanner] Invalid time_step" << std::endl;
    return false;
  }
  
  // 分配资源
  // ...
  
  return true;
}
```

#### 3.3.3 `plan()` - 核心规划逻辑

```cpp
bool plan(const planning::PlanningContext& context,
         std::chrono::milliseconds deadline,
         plugin::PlanningResult& result) override {
  auto start_time = std::chrono::steady_clock::now();
  
  // 1. 从 context 读取感知数据
  const auto& ego = context.ego;              // 自车状态
  const auto& goal = context.task.goal_pose;  // 目标点
  const auto& grid = context.occupancy_grid;  // 栅格地图
  
  // 2. 执行规划算法
  std::vector<plugin::TrajectoryPoint> trajectory;
  // ... 你的规划算法 ...
  
  // 3. 填充结果
  result.trajectory = trajectory;
  result.success = true;
  result.planner_name = "MyPlanner";
  
  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_ms = 
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  return true;
}
```

#### 3.3.4 `isAvailable()` - 可用性检查

```cpp
std::pair<bool, std::string> isAvailable(
    const planning::PlanningContext& context) const override {
  // 检查必需的感知数据
  if (!context.occupancy_grid) {
    return {false, "Missing occupancy grid"};
  }
  
  // 检查其他条件
  if (context.task.goal_pose.x == 0 && context.task.goal_pose.y == 0) {
    return {false, "Invalid goal pose"};
  }
  
  return {true, ""};
}
```

---

## 4. 完整示例：开发自定义规划器

### 4.1 创建目录结构

```bash
cd navsim-local/plugins/planning
mkdir -p my_planner/{include,src}
```

目录结构：
```
plugins/planning/my_planner/
├── CMakeLists.txt
├── include/
│   ├── my_planner_plugin.hpp
│   └── my_planner_plugin_register.hpp
└── src/
    ├── my_planner_plugin.cpp
    └── register.cpp
```

### 4.2 编写头文件

**`include/my_planner_plugin.hpp`**:

```cpp
#pragma once

#include "plugin/framework/planner_plugin_interface.hpp"
#include "core/planning_context.hpp"
#include <nlohmann/json.hpp>

namespace navsim {
namespace plugins {
namespace planning {

class MyPlannerPlugin : public plugin::PlannerPluginInterface {
public:
  // 配置参数
  struct Config {
    double time_step = 0.1;
    double max_velocity = 5.0;
    
    static Config fromJson(const nlohmann::json& json);
  };
  
  MyPlannerPlugin();
  explicit MyPlannerPlugin(const Config& config);
  
  // 必须实现的接口
  plugin::PlannerPluginMetadata getMetadata() const override;
  bool initialize(const nlohmann::json& config) override;
  bool plan(const planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           plugin::PlanningResult& result) override;
  std::pair<bool, std::string> isAvailable(
      const planning::PlanningContext& context) const override;
  
  // 可选接口
  void reset() override;
  nlohmann::json getStatistics() const override;

private:
  Config config_;
  
  struct Statistics {
    size_t total_plans = 0;
    size_t successful_plans = 0;
  };
  Statistics stats_;
};

} // namespace planning
} // namespace plugins
} // namespace navsim
```

### 4.3 编写实现文件

**`src/my_planner_plugin.cpp`**:

```cpp
#include "my_planner_plugin.hpp"
#include <iostream>

namespace navsim {
namespace plugins {
namespace planning {

MyPlannerPlugin::Config MyPlannerPlugin::Config::fromJson(const nlohmann::json& json) {
  Config config;
  config.time_step = json.value("time_step", 0.1);
  config.max_velocity = json.value("max_velocity", 5.0);
  return config;
}

MyPlannerPlugin::MyPlannerPlugin() : config_() {}

MyPlannerPlugin::MyPlannerPlugin(const Config& config) : config_(config) {}

plugin::PlannerPluginMetadata MyPlannerPlugin::getMetadata() const {
  return {
    .name = "MyPlanner",
    .version = "1.0.0",
    .description = "My custom path planner",
    .type = "custom",
    .author = "Your Name",
    .can_be_fallback = false,
    .required_perception = {"occupancy_grid"}
  };
}

bool MyPlannerPlugin::initialize(const nlohmann::json& config) {
  config_ = Config::fromJson(config);
  std::cout << "[MyPlanner] Initialized with time_step=" << config_.time_step << std::endl;
  return true;
}

bool MyPlanner::plan(const planning::PlanningContext& context,
                    std::chrono::milliseconds deadline,
                    plugin::PlanningResult& result) {
  auto start_time = std::chrono::steady_clock::now();
  stats_.total_plans++;
  
  // 获取数据
  const auto& start = context.ego.pose;
  const auto& goal = context.task.goal_pose;
  
  // TODO: 实现你的规划算法
  std::vector<plugin::TrajectoryPoint> trajectory;
  // ...
  
  // 填充结果
  result.trajectory = trajectory;
  result.success = true;
  result.planner_name = "MyPlanner";
  
  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_ms = 
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  stats_.successful_plans++;
  return true;
}

std::pair<bool, std::string> MyPlannerPlugin::isAvailable(
    const planning::PlanningContext& context) const {
  if (!context.occupancy_grid) {
    return {false, "Missing occupancy grid"};
  }
  return {true, ""};
}

void MyPlannerPlugin::reset() {
  stats_ = Statistics();
}

nlohmann::json MyPlannerPlugin::getStatistics() const {
  return {
    {"total_plans", stats_.total_plans},
    {"successful_plans", stats_.successful_plans}
  };
}

} // namespace planning
} // namespace plugins
} // namespace navsim
```

### 4.4 编写注册文件

**`src/register.cpp`**:

```cpp
#include "my_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"

namespace navsim {
namespace plugins {
namespace planning {

void registerMyPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "MyPlanner",
        []() -> std::shared_ptr<plugin::PlannerPluginInterface> {
          return std::make_shared<MyPlannerPlugin>();
        });
    registered = true;
  }
}

} // namespace planning
} // namespace plugins
} // namespace navsim

// 导出 C 风格函数
extern "C" {
  void registerMyPlannerPlugin() {
    navsim::plugins::planning::registerMyPlannerPlugin();
  }
}

// 静态初始化器
namespace {
struct MyPlannerPluginInitializer {
  MyPlannerPluginInitializer() {
    navsim::plugins::planning::registerMyPlannerPlugin();
  }
};
static MyPlannerPluginInitializer g_my_planner_initializer;
}
```

### 4.5 配置 CMake

**`CMakeLists.txt`**:

```cmake
# My Planner Plugin
add_library(my_planner_plugin SHARED
    src/my_planner_plugin.cpp
    src/register.cpp)

set_target_properties(my_planner_plugin PROPERTIES
    OUTPUT_NAME "my_planner_plugin"
    VERSION 1.0.0
    SOVERSION 1)

target_include_directories(my_planner_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(my_planner_plugin
    PUBLIC
        navsim_plugin_framework
    PRIVATE
        Eigen3::Eigen)

target_compile_features(my_planner_plugin PUBLIC cxx_std_17)

message(STATUS "    My planner plugin configured")
```

**修改 `plugins/planning/CMakeLists.txt`**，添加：

```cmake
add_subdirectory(my_planner)
```

### 4.6 添加配置文件

在 `config/default.json` 中添加：

```json
{
  "planning": {
    "primary_planner": "MyPlanner",
    "fallback_planner": "StraightLinePlanner",
    "enable_fallback": true,
    "planners": {
      "MyPlanner": {
        "time_step": 0.1,
        "max_velocity": 5.0
      }
    }
  }
}
```

### 4.7 编译和测试

```bash
cd navsim-local/build
cmake ..
make -j$(nproc)

# 运行
./navsim_algo ws://127.0.0.1:8080/ws demo --config=../config/default.json
```

---

## 5. 常见问题与调试

### 5.1 插件未被加载

**症状**: 日志中没有插件注册信息

**排查步骤**:
1. 检查是否添加了静态初始化器
2. 检查 CMakeLists.txt 是否正确配置
3. 检查插件名称是否一致（注册名 vs 配置文件名）

### 5.2 规划器不可用

**症状**: 日志显示 "Planner is not available"

**排查步骤**:
1. 检查 `isAvailable()` 返回值
2. 确认必需的感知数据是否存在
3. 检查感知插件是否正确配置

### 5.3 调试技巧

```cpp
// 添加详细日志
std::cout << "[MyPlanner] plan() called, ego=(" 
          << context.ego.pose.x << "," << context.ego.pose.y << ")" << std::endl;

// 检查数据有效性
if (context.occupancy_grid) {
  std::cout << "[MyPlanner] Grid size: " 
            << context.occupancy_grid->width << "x" 
            << context.occupancy_grid->height << std::endl;
}
```

---

## 总结

NavSim-Local 的插件系统通过以下机制实现了高度的可扩展性：

1. **注册表模式**: 统一管理所有插件
2. **工厂模式**: 动态创建插件实例
3. **接口抽象**: 清晰的插件接口定义
4. **配置驱动**: JSON 配置文件控制插件行为
5. **降级机制**: 主规划器失败时自动切换

开发新插件只需：
1. 实现 `PlannerPluginInterface` 接口
2. 编写注册代码
3. 配置 CMake
4. 添加配置文件

**参考资料**:
- 完整架构设计: `docs/PLUGIN_ARCHITECTURE_DESIGN.md`
- 快速参考: `docs/PLUGIN_QUICK_REFERENCE.md`
- 示例插件: `plugins/planning/straight_line/`, `plugins/planning/astar/`


# NavSim 插件开发指南

本文档详细介绍如何开发 NavSim 插件，包括使用脚手架工具、编写算法代码、集成已有算法等。

## 📋 目录

- [快速开始](#快速开始)
- [插件系统架构](#插件系统架构)
- [使用脚手架工具](#使用脚手架工具)
- [插件目录结构](#插件目录结构)
- [编写算法层代码](#编写算法层代码)
- [编写适配层代码](#编写适配层代码)
- [配置构建系统](#配置构建系统)
- [编译和测试](#编译和测试)
- [完整示例：A* Planner](#完整示例a-planner)
- [集成已有算法](#集成已有算法)
- [最佳实践](#最佳实践)
- [常见问题](#常见问题)

## 🚀 快速开始

### 5 分钟创建你的第一个插件

```bash
# 1. 使用脚手架工具生成插件模板
python3 tools/navsim_create_plugin.py \
    --name MyPlanner \
    --type planner \
    --output plugins/planning/my_planner \
    --author "Your Name" \
    --description "My awesome planner"

# 2. 实现算法代码
# 编辑 plugins/planning/my_planner/algorithm/my_planner.{hpp,cpp}

# 3. 实现适配层
# 编辑 plugins/planning/my_planner/adapter/my_planner_plugin.{hpp,cpp}

# 4. 更新 CMakeLists.txt
# 添加算法源文件和依赖

# 5. 编译
cd build
cmake ..
make my_planner_plugin -j4

# 6. 测试
./navsim_local_debug \
  --planner MyPlanner \
  --scenario ../scenarios/simple_corridor.json
```

## 🏗️ 插件系统架构

### 三层解耦架构

NavSim 插件系统采用三层架构设计，确保代码的可维护性和可复用性：

```
┌─────────────────────────────────────────────────────────┐
│                    Platform Layer                        │
│  (平台层 - 插件框架、数据结构、接口定义)                  │
│  - PlannerPluginInterface                                │
│  - PerceptionPluginInterface                             │
│  - PlanningContext, PlanningResult                       │
└─────────────────────────────────────────────────────────┘
                            ▲
                            │ 实现接口
                            │
┌─────────────────────────────────────────────────────────┐
│                    Adapter Layer                         │
│  (适配层 - 平台接口适配)                                  │
│  - 实现平台插件接口                                       │
│  - 数据格式转换 (平台 ↔ 算法)                            │
│  - 配置加载和参数管理                                     │
│  - 处理 JSON 配置                                        │
└─────────────────────────────────────────────────────────┘
                            ▲
                            │ 调用算法
                            │
┌─────────────────────────────────────────────────────────┐
│                   Algorithm Layer                        │
│  (算法层 - 纯算法实现)                                    │
│  - 只依赖 Eigen + STL                                    │
│  - 无平台依赖，可独立测试                                 │
│  - 可复用到其他项目                                       │
│  - 不处理 JSON                                           │
└─────────────────────────────────────────────────────────┘
```

### 设计原则

1. **算法层纯净**：
   - ✅ 只依赖 Eigen 和 STL
   - ❌ 不依赖平台 API
   - ❌ 不处理 JSON
   - ✅ 可独立编译和测试

2. **适配层职责**：
   - ✅ 实现平台插件接口
   - ✅ 数据格式转换
   - ✅ 配置加载（JSON → 算法参数）
   - ✅ 错误处理和日志

3. **平台层稳定**：
   - ✅ 提供统一的插件接口
   - ✅ 管理插件生命周期
   - ✅ 提供标准化数据结构

## 🛠️ 使用脚手架工具

### 基本用法

```bash
python3 tools/navsim_create_plugin.py \
    --name <PluginName> \
    --type <planner|perception> \
    --output <output_directory> \
    [--author "Author Name"] \
    [--description "Plugin description"] \
    [--verbose]
```

### 参数说明

| 参数 | 必需 | 说明 | 示例 |
|------|------|------|------|
| `--name` | ✅ | 插件名称（PascalCase） | `AstarPlanner` |
| `--type` | ✅ | 插件类型 | `planner` 或 `perception` |
| `--output` | ✅ | 输出目录 | `plugins/planning/astar_planner` |
| `--author` | ❌ | 作者名称 | `"NavSim Team"` |
| `--description` | ❌ | 插件描述 | `"A* path planner"` |
| `--verbose` | ❌ | 详细输出 | - |

### 生成的文件

```
my_planner/
├── CMakeLists.txt              # 构建配置
├── README.md                   # 插件文档
├── algorithm/                  # 算法层
│   ├── my_planner.hpp         # 算法头文件
│   └── my_planner.cpp         # 算法实现
└── adapter/                    # 适配层
    ├── my_planner_plugin.hpp  # 适配器头文件
    ├── my_planner_plugin.cpp  # 适配器实现
    └── register.cpp           # 注册函数
```

### 生成代码的特点

✅ **所有类型引用都是完全限定的**：
- `navsim::plugin::PlanningResult`（不是 `plugin::PlanningResult`）
- `navsim::planning::PlanningContext`（不是 `PlanningContext`）

✅ **清晰的 TODO 注释**：
- 指导用户如何添加算法实例
- 指导用户如何添加成员变量
- 指导用户如何实现数据转换

✅ **可直接编译**：
- 生成的代码可以直接编译（虽然功能是空的）
- 无需修复类型引用或命名空间

## 📁 插件目录结构

### 标准目录结构

```
my_planner/
├── CMakeLists.txt              # 构建配置
├── README.md                   # 插件文档
├── algorithm/                  # 算法层（纯算法实现）
│   ├── my_planner.hpp         # 算法接口
│   ├── my_planner.cpp         # 算法实现
│   ├── data_structures.hpp    # 算法数据结构（可选）
│   └── utils.hpp              # 辅助函数（可选）
└── adapter/                    # 适配层（平台接口适配）
    ├── my_planner_plugin.hpp  # 适配器头文件
    ├── my_planner_plugin.cpp  # 适配器实现
    └── register.cpp           # 注册函数
```

### 文件职责

| 文件 | 层次 | 职责 |
|------|------|------|
| `algorithm/*.{hpp,cpp}` | 算法层 | 纯算法实现，只依赖 Eigen + STL |
| `adapter/*_plugin.{hpp,cpp}` | 适配层 | 实现平台接口，数据转换 |
| `adapter/register.cpp` | 适配层 | 插件注册函数 |
| `CMakeLists.txt` | 构建 | 编译配置 |
| `README.md` | 文档 | 插件说明 |

## 💻 编写算法层代码

### 算法层设计原则

1. **只依赖 Eigen 和 STL**
2. **不依赖平台 API**
3. **不处理 JSON**
4. **可独立编译和测试**

### 示例：算法头文件

```cpp
// algorithm/my_planner.hpp
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace my_planner {
namespace algorithm {

// 算法配置（纯数据结构）
struct Config {
  double max_velocity = 2.0;
  double max_acceleration = 2.0;
  double step_size = 0.5;
  int max_iterations = 10000;
};

// 算法结果（纯数据结构）
struct Result {
  bool success = false;
  std::string message;
  std::vector<Eigen::Vector3d> path;  // (x, y, yaw)
  std::vector<double> velocities;
  double computation_time_ms = 0.0;
};

// 算法类
class MyPlanner {
public:
  MyPlanner() = default;
  ~MyPlanner() = default;

  // 设置配置
  void setConfig(const Config& config) { config_ = config; }

  // 规划接口
  Result plan(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& goal);

private:
  Config config_;
  
  // 辅助方法
  bool isValid(const Eigen::Vector3d& state);
  double computeCost(const Eigen::Vector3d& state);
};

} // namespace algorithm
} // namespace my_planner
```

### 示例：算法实现

```cpp
// algorithm/my_planner.cpp
#include "my_planner.hpp"
#include <chrono>

namespace my_planner {
namespace algorithm {

Result MyPlanner::plan(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal) {
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  Result result;
  
  // TODO: 实现你的算法
  // 示例：简单的直线插值
  int num_points = static_cast<int>(
      (goal - start).norm() / config_.step_size);
  
  for (int i = 0; i <= num_points; ++i) {
    double t = static_cast<double>(i) / num_points;
    Eigen::Vector3d point = start + t * (goal - start);
    result.path.push_back(point);
    result.velocities.push_back(config_.max_velocity);
  }
  
  result.success = true;
  result.message = "Planning succeeded";
  
  auto end_time = std::chrono::high_resolution_clock::now();
  result.computation_time_ms = 
      std::chrono::duration<double, std::milli>(
          end_time - start_time).count();
  
  return result;
}

bool MyPlanner::isValid(const Eigen::Vector3d& state) {
  // TODO: 实现状态有效性检查
  return true;
}

double MyPlanner::computeCost(const Eigen::Vector3d& state) {
  // TODO: 实现代价计算
  return 0.0;
}

} // namespace algorithm
} // namespace my_planner
```

## 🔌 编写适配层代码

### 适配层职责

1. **实现平台插件接口**
2. **数据格式转换**（平台 ↔ 算法）
3. **配置加载**（JSON → 算法参数）
4. **错误处理和日志**

### 示例：适配器头文件

```cpp
// adapter/my_planner_plugin.hpp
#pragma once

#include "plugin/framework/planner_plugin_interface.hpp"
#include "../algorithm/my_planner.hpp"
#include <memory>

namespace my_planner {
namespace adapter {

class MyPlannerPlugin : public navsim::plugin::PlannerPluginInterface {
public:
  MyPlannerPlugin();
  ~MyPlannerPlugin() override = default;

  // 实现平台接口
  navsim::plugin::PlannerPluginMetadata getMetadata() const override;
  
  bool loadConfig(const nlohmann::json& config) override;
  
  bool plan(
      const navsim::planning::PlanningContext& context,
      std::chrono::milliseconds deadline,
      navsim::plugin::PlanningResult& result) override;
  
  void reset() override;

private:
  // 算法实例
  std::unique_ptr<algorithm::MyPlanner> planner_;
  algorithm::Config config_;
  bool initialized_ = false;
  bool verbose_ = false;

  // 辅助方法
  void convertAlgorithmOutputToResult(
      const algorithm::Result& algo_result,
      navsim::plugin::PlanningResult& result);
};

// 注册函数声明
void registerMyPlannerPlugin();

} // namespace adapter
} // namespace my_planner
```

### 示例：适配器实现（关键部分）

```cpp
// adapter/my_planner_plugin.cpp
#include "my_planner_plugin.hpp"
#include <iostream>

namespace my_planner {
namespace adapter {

MyPlannerPlugin::MyPlannerPlugin() {
  planner_ = std::make_unique<algorithm::MyPlanner>();
}

navsim::plugin::PlannerPluginMetadata 
MyPlannerPlugin::getMetadata() const {
  navsim::plugin::PlannerPluginMetadata metadata;
  metadata.name = "MyPlanner";
  metadata.version = "1.0.0";
  metadata.type = "search";  // 或 "optimization", "sampling"
  metadata.required_perception_data = {};  // 如需要地图，添加 "occupancy_grid"
  metadata.can_be_fallback = true;
  return metadata;
}

bool MyPlannerPlugin::loadConfig(const nlohmann::json& config) {
  try {
    // 加载算法参数
    if (config.contains("max_velocity")) {
      config_.max_velocity = config["max_velocity"].get<double>();
    }
    if (config.contains("max_acceleration")) {
      config_.max_acceleration = config["max_acceleration"].get<double>();
    }
    if (config.contains("step_size")) {
      config_.step_size = config["step_size"].get<double>();
    }
    if (config.contains("max_iterations")) {
      config_.max_iterations = config["max_iterations"].get<int>();
    }
    if (config.contains("verbose")) {
      verbose_ = config["verbose"].get<bool>();
    }

    // 设置算法配置
    planner_->setConfig(config_);
    initialized_ = true;

    if (verbose_) {
      std::cout << "[MyPlanner] Initialized with config:\n"
                << "  - max_velocity: " << config_.max_velocity << " m/s\n"
                << "  - max_acceleration: " << config_.max_acceleration << " m/s²\n"
                << "  - step_size: " << config_.step_size << " m\n"
                << "  - max_iterations: " << config_.max_iterations << std::endl;
    }

    return true;
  } catch (const std::exception& e) {
    std::cerr << "[MyPlanner] Failed to load config: " << e.what() << std::endl;
    return false;
  }
}

bool MyPlannerPlugin::plan(
    const navsim::planning::PlanningContext& context,
    std::chrono::milliseconds deadline,
    navsim::plugin::PlanningResult& result) {
  
  if (!initialized_) {
    result.success = false;
    result.failure_reason = "Plugin not initialized";
    return false;
  }

  // 提取起点和终点
  Eigen::Vector3d start(
      context.ego.pose.x,
      context.ego.pose.y,
      context.ego.pose.yaw);
  
  Eigen::Vector3d goal(
      context.task.goal_pose.x,
      context.task.goal_pose.y,
      context.task.goal_pose.yaw);

  if (verbose_) {
    std::cout << "[MyPlanner] Planning from " << start.transpose()
              << " to " << goal.transpose() << std::endl;
  }

  // 调用算法
  auto algo_result = planner_->plan(start, goal);

  // 转换输出
  convertAlgorithmOutputToResult(algo_result, result);

  if (verbose_) {
    std::cout << "[MyPlanner] Planning " 
              << (result.success ? "succeeded" : "failed")
              << " in " << algo_result.computation_time_ms << " ms" << std::endl;
  }

  return result.success;
}

void MyPlannerPlugin::convertAlgorithmOutputToResult(
    const algorithm::Result& algo_result,
    navsim::plugin::PlanningResult& result) {
  
  result.success = algo_result.success;
  result.failure_reason = algo_result.message;
  result.computation_time_ms = algo_result.computation_time_ms;

  // 转换路径
  result.trajectory.clear();
  for (size_t i = 0; i < algo_result.path.size(); ++i) {
    navsim::plugin::TrajectoryPoint point;
    point.pose.x = algo_result.path[i](0);
    point.pose.y = algo_result.path[i](1);
    point.pose.yaw = algo_result.path[i](2);
    
    if (i < algo_result.velocities.size()) {
      point.twist.vx = algo_result.velocities[i];
    }
    
    point.time_from_start = i * config_.step_size / config_.max_velocity;
    
    result.trajectory.push_back(point);
  }
}

void MyPlannerPlugin::reset() {
  // 重置算法状态（如果需要）
}

} // namespace adapter
} // namespace my_planner
```

## 🔧 配置构建系统

### 更新插件的 CMakeLists.txt

```cmake
# 添加算法源文件
set(ALGORITHM_SOURCES
    algorithm/my_planner.cpp
    # algorithm/other_file.cpp  # 如果有其他文件
)

# 添加依赖
target_link_libraries(my_planner_plugin
    PRIVATE
        Eigen3::Eigen
        # Boost::boost                # 如果需要 Boost
        # ${OpenCV_LIBS}              # 如果需要 OpenCV
)
```

### 注册插件

编辑 `plugins/planning/CMakeLists.txt`：

```cmake
# 添加插件选项
option(BUILD_MY_PLANNER_PLUGIN "Build MyPlanner plugin" ON)

# 添加子目录
if(BUILD_MY_PLANNER_PLUGIN)
    message(STATUS "  [+] MyPlanner plugin")
    add_subdirectory(my_planner)
    list(APPEND PLANNING_PLUGIN_LIBS my_planner_plugin)
endif()
```

编辑 `plugins/plugin_loader.cpp`：

```cpp
#include "planning/my_planner/adapter/my_planner_plugin.hpp"

void loadAllBuiltinPlugins() {
  // ... 其他插件 ...
  my_planner::adapter::registerMyPlannerPlugin();
}
```

## 🧪 编译和测试

### 编译插件

```bash
cd build
cmake ..
make my_planner_plugin -j4
```

### 测试插件

```bash
./navsim_local_debug \
  --planner MyPlanner \
  --scenario ../scenarios/simple_corridor.json \
  --verbose
```

### 调试插件

```bash
# 使用 gdb
gdb --args ./navsim_local_debug \
  --planner MyPlanner \
  --scenario ../scenarios/simple_corridor.json

# 在 gdb 中
(gdb) break my_planner::algorithm::MyPlanner::plan
(gdb) run
```

## 📚 完整示例：A* Planner

详细的 A* Planner 实现示例请参考：
- `plugins/planning/astar_planner/algorithm/astar_planner.{hpp,cpp}`
- `plugins/planning/astar_planner/adapter/astar_planner_plugin.{hpp,cpp}`

关键特点：
1. **GridMapAdapter**：适配平台的 `OccupancyGrid` 到算法的 `GridMapInterface`
2. **手动坐标转换**：避免链接问题
3. **配置参数**：启发式权重、对角线移动、目标容差等

## 🔄 集成已有算法

### 场景：你已经有一个算法实现

1. **复制算法文件到 `algorithm/` 目录**
2. **创建适配器类**
3. **实现数据转换**
4. **更新 CMakeLists.txt**

### 示例：集成第三方库

如果你的算法依赖第三方库（如 OMPL）：

```cmake
# CMakeLists.txt
find_package(ompl REQUIRED)

target_link_libraries(my_planner_plugin
    PRIVATE
        Eigen3::Eigen
        ${OMPL_LIBRARIES}
)

target_include_directories(my_planner_plugin
    PRIVATE
        ${OMPL_INCLUDE_DIRS}
)
```

## ✅ 最佳实践

### 1. 算法层设计

- ✅ 使用 Eigen 进行数学计算
- ✅ 使用 STL 容器
- ❌ 不要依赖平台 API
- ❌ 不要处理 JSON
- ✅ 提供清晰的配置结构
- ✅ 返回详细的结果信息

### 2. 适配层设计

- ✅ 实现所有平台接口方法
- ✅ 提供详细的错误信息
- ✅ 添加日志输出（可通过 `verbose` 控制）
- ✅ 处理边界情况
- ✅ 验证输入数据

### 3. 性能优化

- ✅ 避免不必要的内存分配
- ✅ 使用 `std::move` 转移大对象
- ✅ 预分配容器大小
- ✅ 使用 `const&` 传递大对象

### 4. 代码风格

- ✅ 遵循 C++17 标准
- ✅ 使用有意义的变量名
- ✅ 添加注释说明复杂逻辑
- ✅ 保持函数简短（< 50 行）

## ❓ 常见问题

### Q1: 链接错误 "undefined symbol"

**问题**：
```
undefined symbol: _ZNK6navsim8planning13OccupancyGrid11worldToCellE...
```

**原因**：平台库函数在动态加载的插件中不可用

**解决方案**：在适配器中手动实现简单的转换逻辑

```cpp
// 不要调用 grid->worldToCell()
// 手动实现
int gx = static_cast<int>((x - grid->config.origin.x) / grid->config.resolution);
int gy = static_cast<int>((y - grid->config.origin.y) / grid->config.resolution);
```

### Q2: 如何声明对感知数据的依赖？

**解决方案**：在 `getMetadata()` 中声明

```cpp
metadata.required_perception_data = {"occupancy_grid"};  // 或 "esdf_map"
```

然后在命令行中加载相应的感知插件：
```bash
--perception GridMapBuilder  # 提供 occupancy_grid
```

### Q3: 如何添加自定义配置参数？

**解决方案**：

1. 在算法层的 `Config` 结构中添加参数
2. 在适配层的 `loadConfig()` 中解析 JSON
3. 在场景文件或命令行中提供配置

### Q4: 如何调试算法？

**解决方案**：

1. 使用 `--verbose` 查看详细日志
2. 在算法代码中添加 `std::cout` 输出
3. 使用 gdb 调试器
4. 编写单元测试（独立于平台）

## 📚 参考资料

- [快速开始指南](GETTING_STARTED.md)
- [开发工具指南](DEVELOPMENT_TOOLS.md)
- [架构设计文档](ARCHITECTURE.md)
- 示例插件：
  - `plugins/planning/straight_line_planner/` - 最简单的示例
  - `plugins/planning/astar_planner/` - 完整的搜索算法示例
  - `plugins/planning/jps_planner/` - 复杂算法示例


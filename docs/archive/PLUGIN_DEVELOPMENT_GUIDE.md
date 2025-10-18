# NavSim 插件开发指南

## 目录

- [快速开始](#快速开始)
- [使用脚手架工具](#使用脚手架工具)
- [插件目录结构](#插件目录结构)
- [编写算法层代码](#编写算法层代码)
- [编写适配层代码](#编写适配层代码)
- [配置 CMakeLists.txt](#配置-cmakeliststxt)
- [编译和测试](#编译和测试)
- [完整示例](#完整示例)
- [迁移现有算法](#迁移现有算法)
- [常见问题](#常见问题)

---

## 快速开始

### 5 分钟创建你的第一个插件

```bash
# 1. 使用脚手架工具生成插件
python3 tools/navsim_create_plugin.py \
    --name MyPlanner \
    --type planner \
    --output plugins/planning/my_planner \
    --author "Your Name" \
    --description "My awesome planner"

# 2. 编写算法代码
# 编辑 plugins/planning/my_planner/algorithm/my_planner.cpp

# 3. 修改适配层
# 编辑 plugins/planning/my_planner/adapter/my_planner_plugin.cpp

# 4. 更新 CMakeLists.txt
# 添加源文件和依赖

# 5. 编译
cd build
cmake ..
make my_planner_plugin -j4

# 6. 测试
./navsim_local_debug --planner MyPlanner --scenario scenarios/simple_corridor.json
```

---

## 使用脚手架工具

### 工具介绍

`navsim_create_plugin.py` 是一个自动化工具，用于生成插件模板代码。它会创建完整的目录结构和基础代码，让你专注于算法实现。

### 基本用法

```bash
python3 tools/navsim_create_plugin.py \
    --name <PluginName> \
    --type <planner|perception> \
    --output <output_directory> \
    [--author <author_name>] \
    [--description <description>] \
    [--verbose]
```

### 参数说明

| 参数 | 必需 | 描述 | 示例 |
|------|------|------|------|
| `--name` | ✅ | 插件名称（PascalCase） | `MyPlanner` |
| `--type` | ✅ | 插件类型 | `planner` 或 `perception` |
| `--output` | ✅ | 输出目录 | `plugins/planning/my_planner` |
| `--author` | ❌ | 作者名称 | `"Your Name"` |
| `--description` | ❌ | 插件描述 | `"My awesome planner"` |
| `--verbose` | ❌ | 显示详细信息 | - |

### 示例

#### 创建规划器插件

```bash
python3 tools/navsim_create_plugin.py \
    --name RrtStar \
    --type planner \
    --output plugins/planning/rrt_star \
    --author "NavSim Team" \
    --description "RRT* path planner" \
    --verbose
```

#### 创建感知器插件

```bash
python3 tools/navsim_create_plugin.py \
    --name PointCloudProcessor \
    --type perception \
    --output plugins/perception/pointcloud \
    --author "NavSim Team" \
    --description "Point cloud processing plugin"
```

---

## 插件目录结构

脚手架工具会生成以下目录结构：

```
my_planner/
├── README.md                    # 插件说明文档
├── CMakeLists.txt               # 构建配置
├── algorithm/                   # 算法层（纯算法实现）
│   ├── my_planner.hpp           # 算法接口
│   └── my_planner.cpp           # 算法实现
└── adapter/                     # 适配层（平台接口适配）
    ├── my_planner_plugin.hpp    # 插件接口
    ├── my_planner_plugin.cpp    # 插件实现
    └── register.cpp             # 插件注册
```

### 各文件职责

| 文件 | 层次 | 职责 |
|------|------|------|
| `algorithm/*.hpp/cpp` | 算法层 | 纯算法实现，只依赖 Eigen + STL |
| `adapter/*_plugin.hpp/cpp` | 适配层 | 实现平台接口，数据转换 |
| `adapter/register.cpp` | 适配层 | 插件注册（无需修改） |
| `CMakeLists.txt` | 构建 | 构建配置，添加源文件和依赖 |
| `README.md` | 文档 | 插件说明文档 |

---

## 编写算法层代码

### 设计原则

算法层应该：
- ✅ **纯算法实现**：只包含算法逻辑
- ✅ **只依赖 Eigen + STL**：不依赖平台 API
- ✅ **无 JSON 解析**：配置由适配层解析
- ✅ **可独立测试**：可以脱离平台进行单元测试

算法层不应该：
- ❌ 包含 `#include <nlohmann/json.hpp>`
- ❌ 调用平台 API
- ❌ 直接使用平台数据结构

### 算法接口设计

#### 配置结构体

```cpp
struct Config {
  double max_velocity = 2.0;      // 最大速度 (m/s)
  double max_acceleration = 2.0;  // 最大加速度 (m/s²)
  double time_step = 0.1;         // 时间步长 (s)
  // ... 其他参数
};
```

#### 输入数据结构

```cpp
struct PlanningInput {
  Eigen::Vector3d start;  // 起点 (x, y, yaw)
  Eigen::Vector3d goal;   // 终点 (x, y, yaw)
  // ... 其他输入数据（如地图、障碍物等）
};
```

#### 输出数据结构

```cpp
struct Waypoint {
  Eigen::Vector3d position;  // 位置 (x, y, yaw)
  double velocity = 0.0;     // 速度 (m/s)
  double acceleration = 0.0; // 加速度 (m/s²)
  double timestamp = 0.0;    // 时间戳 (s)
  double path_length = 0.0;  // 路径长度 (m)
};

struct PlanningOutput {
  bool success = false;
  std::string error_message;
  std::vector<Waypoint> path;
  double total_cost = 0.0;
};
```

#### 算法类接口

```cpp
class MyPlanner {
public:
  MyPlanner() = default;
  ~MyPlanner() = default;

  // 执行规划
  PlanningOutput plan(const PlanningInput& input, const Config& config) const;

private:
  // 私有辅助函数
  std::vector<Waypoint> generatePath(...) const;
  void computeVelocityProfile(...) const;
};
```

### 示例：直线规划器

<augment_code_snippet path="navsim-local/plugins/planning/straight_line_planner/algorithm/straight_line.hpp" mode="EXCERPT">
````cpp
struct Config {
  double default_velocity = 2.0;
  double time_step = 0.1;
  double planning_horizon = 5.0;
  bool use_trapezoidal_profile = true;
  double max_acceleration = 2.0;
};

struct Waypoint {
  Eigen::Vector3d position;
  double velocity = 0.0;
  double acceleration = 0.0;
  double timestamp = 0.0;
  double path_length = 0.0;
};
````
</augment_code_snippet>

---

## 编写适配层代码

### 设计原则

适配层应该：
- ✅ **实现平台接口**：继承 `PlannerPluginInterface`
- ✅ **数据转换**：平台数据结构 ↔ 算法数据结构
- ✅ **JSON 配置解析**：解析配置文件
- ✅ **错误处理**：捕获异常，返回错误信息
- ✅ **薄适配层**：逻辑尽量放在算法层

### 必须实现的方法

#### 1. getMetadata()

返回插件元数据：

```cpp
navsim::plugin::PlannerPluginMetadata MyPlannerPlugin::getMetadata() const {
  navsim::plugin::PlannerPluginMetadata metadata;
  metadata.name = "MyPlanner";
  metadata.version = "1.0.0";
  metadata.description = "My awesome planner";
  metadata.author = "Your Name";
  metadata.type = "search";  // geometric, search, optimization
  metadata.required_perception_data = {};  // 或 {"esdf_map"}
  metadata.can_be_fallback = false;
  return metadata;
}
```

**插件类型**：
- `geometric`: 几何规划器（如直线、曲线）
- `search`: 搜索规划器（如 A*, JPS, RRT）
- `optimization`: 优化规划器（如 MPC, TEB）

#### 2. initialize()

初始化插件：

```cpp
bool MyPlannerPlugin::initialize(const nlohmann::json& config) {
  if (initialized_) {
    return false;
  }

  // 解析配置
  config_ = parseConfig(config);

  // 初始化算法（如果需要）
  // ...

  initialized_ = true;
  return true;
}
```

#### 3. isAvailable()

检查插件是否可用：

```cpp
std::pair<bool, std::string> MyPlannerPlugin::isAvailable(
    const navsim::planning::PlanningContext& context) const {
  
  if (!initialized_) {
    return {false, "Plugin not initialized"};
  }

  // 检查是否有必要的感知数据
  if (!context.esdf_map) {
    return {false, "ESDF map not available"};
  }

  return {true, ""};
}
```

#### 4. plan()

执行规划：

```cpp
navsim::plugin::PlanningResult MyPlannerPlugin::plan(
    const navsim::planning::PlanningContext& context,
    std::chrono::milliseconds deadline) {
  
  navsim::plugin::PlanningResult result;

  // 1. 转换输入数据（平台 → 算法）
  algorithm::MyPlanner::PlanningInput input;
  input.start = convertPose(context.ego_state.pose);
  input.goal = convertPose(context.goal.pose);

  // 2. 调用算法
  algorithm::MyPlanner planner;
  auto output = planner.plan(input, config_);

  // 3. 转换输出数据（算法 → 平台）
  result.success = output.success;
  if (!output.success) {
    result.failure_reason = output.error_message;
    return result;
  }

  result.trajectory.reserve(output.path.size());
  for (const auto& wp : output.path) {
    navsim::plugin::TrajectoryPoint point;
    point.pose.x = wp.position.x();
    point.pose.y = wp.position.y();
    point.pose.yaw = wp.position.z();
    point.twist.vx = wp.velocity;
    point.acceleration = wp.acceleration;
    point.time_from_start = wp.timestamp;
    point.path_length = wp.path_length;
    result.trajectory.push_back(point);
  }

  return result;
}
```

#### 5. reset()

重置插件状态：

```cpp
void MyPlannerPlugin::reset() {
  // 重置统计信息
  total_plans_ = 0;
  successful_plans_ = 0;
}
```

#### 6. getStatistics()

返回统计信息：

```cpp
nlohmann::json MyPlannerPlugin::getStatistics() const {
  nlohmann::json stats;
  stats["total_plans"] = total_plans_;
  stats["successful_plans"] = successful_plans_;
  stats["success_rate"] = (total_plans_ > 0) 
      ? (double)successful_plans_ / total_plans_ : 0.0;
  return stats;
}
```

### 辅助函数

#### parseConfig()

解析 JSON 配置：

```cpp
algorithm::MyPlanner::Config MyPlannerPlugin::parseConfig(
    const nlohmann::json& json) const {
  
  algorithm::MyPlanner::Config config;

  if (json.contains("max_velocity")) {
    config.max_velocity = json["max_velocity"].get<double>();
  }
  if (json.contains("max_acceleration")) {
    config.max_acceleration = json["max_acceleration"].get<double>();
  }
  // ... 解析其他参数

  return config;
}
```

#### convertPose()

转换位姿数据：

```cpp
Eigen::Vector3d MyPlannerPlugin::convertPose(
    const navsim::planning::Pose2d& pose) const {
  return Eigen::Vector3d(pose.x, pose.y, pose.yaw);
}
```

---

## 配置 CMakeLists.txt

### 基本配置

脚手架工具生成的 `CMakeLists.txt` 已经包含基本配置，你只需要：

1. **添加源文件**
2. **添加依赖库**
3. **添加头文件路径**

### 示例

```cmake
# 插件名称
set(PLUGIN_NAME my_planner_plugin)

# 源文件
set(PLUGIN_SOURCES
    algorithm/my_planner.cpp
    adapter/my_planner_plugin.cpp
    adapter/register.cpp
)

# 头文件
set(PLUGIN_HEADERS
    algorithm/my_planner.hpp
    adapter/my_planner_plugin.hpp
)

# 创建共享库
add_library(${PLUGIN_NAME} SHARED ${PLUGIN_SOURCES})

# 链接依赖
target_link_libraries(${PLUGIN_NAME}
    PUBLIC
        navsim_plugin_framework
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

# 包含目录
target_include_directories(${PLUGIN_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
        ${CMAKE_CURRENT_SOURCE_DIR}/adapter
)
```

### 添加额外依赖

#### 添加 Boost

```cmake
find_package(Boost REQUIRED COMPONENTS system)

target_link_libraries(${PLUGIN_NAME}
    PUBLIC
        Boost::system
)
```

#### 添加 ESDF Builder

```cmake
target_link_libraries(${PLUGIN_NAME}
    PUBLIC
        esdf_builder_plugin
)
```

---

## 编译和测试

### 编译插件

```bash
# 1. 配置 CMake
cd build
cmake ..

# 2. 编译插件
make my_planner_plugin -j4

# 3. 检查生成的 .so 文件
ls -lh build/plugins/planning/my_planner/libmy_planner_plugin.so
```

### 测试插件

#### 1. 创建测试场景

创建 `scenarios/test_my_planner.json`：

```json
{
  "scenario_name": "test_my_planner",
  "description": "Test scenario for MyPlanner",
  
  "ego_state": {
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "twist": {"vx": 0.0, "vy": 0.0, "omega": 0.0}
  },
  
  "goal": {
    "pose": {"x": 10.0, "y": 0.0, "yaw": 0.0}
  },
  
  "planner_config": {
    "MyPlanner": {
      "max_velocity": 2.0,
      "max_acceleration": 2.0
    }
  }
}
```

#### 2. 运行测试

```bash
./build/navsim_local_debug \
    --planner MyPlanner \
    --scenario scenarios/test_my_planner.json
```

#### 3. 查看结果

```
=== NavSim Local Debug Tool ===
[1/5] Initializing plugin system...
[2/5] Loading plugins...
  Loading planner plugin: MyPlanner
  Successfully loaded 1 plugins
[3/5] Loading scenario...
  Scenario loaded successfully
[4/5] Running planner...
  Planner 'MyPlanner' succeeded in 0.123 ms
[5/5] Planning result:
  Success: yes
  Planner: MyPlanner
  Trajectory points: 100
  Computation time: 0.123 ms
=== Done ===
```

---

## 完整示例

参见 [StraightLine 插件](../plugins/planning/straight_line_planner/) 的完整实现。

这是一个使用脚手架工具生成并完成开发的真实插件，包含：
- ✅ 完整的算法层实现（直线路径 + 梯形速度曲线）
- ✅ 完整的适配层实现
- ✅ 完整的 CMakeLists.txt 配置
- ✅ 编译成功
- ✅ 功能验证通过

---

## 迁移现有算法

如果你已经有一个现有的算法实现，可以按照以下步骤迁移到插件系统：

### 步骤 1: 生成插件模板

```bash
python3 tools/navsim_create_plugin.py \
    --name MyExistingAlgorithm \
    --type planner \
    --output plugins/planning/my_existing_algorithm
```

### 步骤 2: 复制算法代码

```bash
cp path/to/existing/algorithm/*.{hpp,cpp} \
   plugins/planning/my_existing_algorithm/algorithm/
```

### 步骤 3: 调整算法接口

确保算法层：
- ✅ 只依赖 Eigen + STL
- ✅ 不包含 JSON 解析
- ✅ 使用 `Config` 结构体而不是 JSON

**修改前**：
```cpp
void plan(const nlohmann::json& config) {
  double max_vel = config["max_velocity"];
  // ...
}
```

**修改后**：
```cpp
struct Config {
  double max_velocity = 2.0;
};

void plan(const Config& config) {
  double max_vel = config.max_velocity;
  // ...
}
```

### 步骤 4: 实现适配层

在 `adapter/` 中实现平台接口，参考 [编写适配层代码](#编写适配层代码)。

### 步骤 5: 更新 CMakeLists.txt

添加所有源文件和依赖，参考 [配置 CMakeLists.txt](#配置-cmakeliststxt)。

### 步骤 6: 编译和测试

```bash
cd build
cmake ..
make my_existing_algorithm_plugin -j4
./navsim_local_debug --planner MyExistingAlgorithm --scenario scenarios/test.json
```

---

## 常见问题

### Q1: 编译错误：找不到头文件

**问题**：
```
fatal error: my_algorithm.hpp: No such file or directory
```

**解决方案**：
在 `CMakeLists.txt` 中添加头文件路径：
```cmake
target_include_directories(${PLUGIN_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
)
```

### Q2: 链接错误：undefined reference

**问题**：
```
undefined reference to `MyClass::myFunction()'
```

**解决方案**：
1. 确认 `.cpp` 文件已添加到 `PLUGIN_SOURCES`
2. 确认依赖库已添加到 `target_link_libraries`

### Q3: 插件加载失败

**问题**：
```
[DynamicPluginLoader] Failed to load plugin: MyPlanner
```

**解决方案**：
1. 检查 `.so` 文件是否存在
2. 检查插件名称是否正确（区分大小写）
3. 使用 `ldd` 检查依赖：
   ```bash
   ldd build/plugins/planning/my_planner/libmy_planner_plugin.so
   ```

### Q4: 如何调试插件？

**方法 1：使用 GDB**
```bash
gdb --args ./build/navsim_local_debug --planner MyPlanner --scenario scenarios/test.json
(gdb) break MyPlannerPlugin::plan
(gdb) run
```

**方法 2：添加日志**
```cpp
std::cout << "[MyPlanner] Debug info: " << value << std::endl;
```

**方法 3：使用 Valgrind**
```bash
valgrind --leak-check=full ./build/navsim_local_debug --planner MyPlanner
```

### Q5: 如何优化性能？

1. **使用 Release 模式编译**：
   ```bash
   cmake -DCMAKE_BUILD_TYPE=Release ..
   ```

2. **启用编译器优化**：
   ```cmake
   target_compile_options(${PLUGIN_NAME} PRIVATE -O3 -march=native)
   ```

3. **使用性能分析工具**：
   ```bash
   perf record ./build/navsim_local_debug --planner MyPlanner
   perf report
   ```

---

## 参考资料

- [插件系统说明书](PLUGIN_SYSTEM_GUIDE.md)
- [重构提案](../REFACTORING_PROPOSAL.md)
- [StraightLine 插件示例](../plugins/planning/straight_line_planner/)

---

**版本**: 1.0.0  
**最后更新**: 2025-10-18  
**作者**: NavSim Team


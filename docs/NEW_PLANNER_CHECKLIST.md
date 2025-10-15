# 新增规划器插件检查清单

## 📋 快速开始检查清单

### ✅ 第一步：创建文件结构

```bash
cd navsim-local/plugins/planning
mkdir -p my_planner/{include,src}
```

- [ ] 创建 `plugins/planning/my_planner/` 目录
- [ ] 创建 `include/` 子目录
- [ ] 创建 `src/` 子目录

### ✅ 第二步：创建必需文件

需要创建以下 4 个文件：

- [ ] `include/my_planner_plugin.hpp` - 插件头文件
- [ ] `include/my_planner_plugin_register.hpp` - 注册函数声明（可选）
- [ ] `src/my_planner_plugin.cpp` - 插件实现
- [ ] `src/register.cpp` - 注册代码
- [ ] `CMakeLists.txt` - 构建配置

### ✅ 第三步：实现插件接口

在 `my_planner_plugin.hpp` 中必须实现：

```cpp
class MyPlannerPlugin : public plugin::PlannerPluginInterface {
public:
  // ✅ 必须实现
  plugin::PlannerPluginMetadata getMetadata() const override;
  bool initialize(const nlohmann::json& config) override;
  bool plan(const planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           plugin::PlanningResult& result) override;
  std::pair<bool, std::string> isAvailable(
      const planning::PlanningContext& context) const override;
  
  // ❌ 可选实现
  void reset() override;
  nlohmann::json getStatistics() const override;
};
```

检查项：
- [ ] 继承 `plugin::PlannerPluginInterface`
- [ ] 实现 `getMetadata()` 方法
- [ ] 实现 `initialize()` 方法
- [ ] 实现 `plan()` 方法
- [ ] 实现 `isAvailable()` 方法
- [ ] （可选）实现 `reset()` 方法
- [ ] （可选）实现 `getStatistics()` 方法

### ✅ 第四步：编写注册代码

在 `src/register.cpp` 中：

```cpp
#include "my_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"

namespace navsim::plugins::planning {

void registerMyPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "MyPlanner",  // ⚠️ 插件名称，必须与配置文件一致
        []() -> std::shared_ptr<plugin::PlannerPluginInterface> {
          return std::make_shared<MyPlannerPlugin>();
        });
    registered = true;
  }
}

} // namespace

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

检查项：
- [ ] 实现 `registerMyPlannerPlugin()` 函数
- [ ] 调用 `PlannerPluginRegistry::getInstance().registerPlugin()`
- [ ] 插件名称与配置文件一致
- [ ] 导出 C 风格函数（用于动态加载）
- [ ] 添加静态初始化器（用于静态链接）

### ✅ 第五步：配置 CMake

创建 `CMakeLists.txt`：

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

修改 `plugins/planning/CMakeLists.txt`，添加：

```cmake
add_subdirectory(my_planner)
```

检查项：
- [ ] 创建 `CMakeLists.txt`
- [ ] 设置为 SHARED 库（支持动态加载）
- [ ] 包含所有源文件
- [ ] 链接 `navsim_plugin_framework`
- [ ] 在父目录 CMakeLists.txt 中添加 `add_subdirectory(my_planner)`

### ✅ 第六步：添加配置

在 `config/default.json` 中添加：

```json
{
  "planning": {
    "primary_planner": "MyPlanner",  // ⚠️ 必须与注册名称一致
    "fallback_planner": "StraightLinePlanner",
    "enable_fallback": true,
    "planners": {
      "MyPlanner": {
        "time_step": 0.1,
        "max_velocity": 5.0
        // 你的自定义参数...
      }
    }
  }
}
```

检查项：
- [ ] 在 `planning.planners` 中添加插件配置
- [ ] 插件名称与注册名称一致
- [ ] 设置 `primary_planner` 或 `fallback_planner`
- [ ] 添加所需的配置参数

### ✅ 第七步：编译和测试

```bash
cd navsim-local/build
cmake ..
make -j$(nproc)

# 检查插件是否编译成功
ls -lh plugins/planning/libmy_planner_plugin.so

# 运行测试
./navsim_algo ws://127.0.0.1:8080/ws demo --config=../config/default.json
```

检查项：
- [ ] 编译成功，无错误
- [ ] 生成 `.so` 文件
- [ ] 运行时日志显示插件已注册
- [ ] 运行时日志显示插件已加载
- [ ] 规划器正常工作

---

## 🔍 常见错误检查

### 错误1: 插件未注册

**症状**: 日志中没有 "Registered plugin: MyPlanner"

**检查**:
- [ ] 是否添加了静态初始化器？
- [ ] 是否在 CMakeLists.txt 中添加了 `register.cpp`？
- [ ] 插件名称是否拼写正确？

### 错误2: 插件未加载

**症状**: 日志显示 "Failed to create planner: MyPlanner"

**检查**:
- [ ] 配置文件中的插件名称是否与注册名称一致？
- [ ] 是否在 `planners` 中添加了配置？
- [ ] 插件是否成功注册到 Registry？

### 错误3: 规划器不可用

**症状**: 日志显示 "Planner is not available"

**检查**:
- [ ] `isAvailable()` 返回了什么？
- [ ] 必需的感知数据是否存在？
- [ ] 感知插件是否正确配置？

### 错误4: 编译错误

**症状**: 编译时报错

**检查**:
- [ ] 是否包含了正确的头文件？
- [ ] 是否链接了 `navsim_plugin_framework`？
- [ ] 是否使用了 C++17 标准？
- [ ] 命名空间是否正确？

---

## 📝 代码模板

### getMetadata() 模板

```cpp
plugin::PlannerPluginMetadata getMetadata() const override {
  return {
    .name = "MyPlanner",              // ⚠️ 必须与注册名称一致
    .version = "1.0.0",
    .description = "My custom planner",
    .type = "search",                 // search/optimization/geometric/hybrid
    .author = "Your Name",
    .can_be_fallback = false,         // 是否可作为降级规划器
    .required_perception = {"occupancy_grid"}  // 必需的感知数据
  };
}
```

### initialize() 模板

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
  
  std::cout << "[MyPlanner] Initialized successfully" << std::endl;
  return true;
}
```

### plan() 模板

```cpp
bool plan(const planning::PlanningContext& context,
         std::chrono::milliseconds deadline,
         plugin::PlanningResult& result) override {
  auto start_time = std::chrono::steady_clock::now();
  
  // 1. 获取数据
  const auto& ego = context.ego;
  const auto& goal = context.task.goal_pose;
  const auto& grid = context.occupancy_grid;
  
  // 2. 执行规划算法
  std::vector<plugin::TrajectoryPoint> trajectory;
  // TODO: 实现你的算法
  
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

### isAvailable() 模板

```cpp
std::pair<bool, std::string> isAvailable(
    const planning::PlanningContext& context) const override {
  // 检查必需的感知数据
  if (!context.occupancy_grid) {
    return {false, "Missing occupancy grid"};
  }
  
  // 检查其他条件
  // ...
  
  return {true, ""};
}
```

---

## 🎯 最佳实践

### 1. 命名规范

- [ ] 插件类名：`XxxPlannerPlugin`（驼峰命名）
- [ ] 注册名称：`XxxPlanner`（与类名一致，去掉 Plugin 后缀）
- [ ] 文件名：`xxx_planner_plugin.hpp/cpp`（下划线命名）
- [ ] 库名：`xxx_planner_plugin`（下划线命名）

### 2. 性能优化

- [ ] 在 `initialize()` 中分配资源，避免在 `plan()` 中重复分配
- [ ] 使用成员变量缓存可复用数据
- [ ] 注意 `deadline` 参数，避免超时
- [ ] 使用 `reset()` 清理状态，避免内存泄漏

### 3. 错误处理

- [ ] 在 `initialize()` 中验证配置参数
- [ ] 在 `isAvailable()` 中检查必需数据
- [ ] 在 `plan()` 中处理异常情况
- [ ] 提供清晰的错误信息

### 4. 日志输出

- [ ] 使用统一的日志格式：`[PluginName] Message`
- [ ] 在关键步骤输出日志
- [ ] 区分信息日志（std::cout）和错误日志（std::cerr）

### 5. 测试

- [ ] 编写单元测试
- [ ] 测试不同的配置参数
- [ ] 测试边界情况
- [ ] 测试降级机制

---

## 📚 参考资料

- **完整指南**: `docs/PLUGIN_SYSTEM_GUIDE_CN.md`
- **架构设计**: `docs/PLUGIN_ARCHITECTURE_DESIGN.md`
- **快速参考**: `docs/PLUGIN_QUICK_REFERENCE.md`
- **示例插件**: 
  - `plugins/planning/straight_line/` - 简单几何规划器
  - `plugins/planning/astar/` - A* 搜索规划器

---

## ✅ 完成检查

全部完成后，你应该能够：

- [ ] 编译成功，生成 `.so` 文件
- [ ] 运行时看到插件注册日志
- [ ] 运行时看到插件加载日志
- [ ] 规划器能够正常执行
- [ ] 配置参数能够正确读取
- [ ] 降级机制能够正常工作

**恭喜！你已经成功创建了一个新的规划器插件！** 🎉


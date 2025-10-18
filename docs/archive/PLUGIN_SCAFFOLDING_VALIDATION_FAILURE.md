# 插件脚手架工具"全新插件"验证失败报告

本文档记录了使用 `navsim_create_plugin.py` 创建全新插件（StraightPathPlanner）的验证过程，以及发现的**严重问题**。

---

## 🎯 验证目标

验证工具生成的代码是否可以"开箱即用"（不修改任何生成的代码）。

---

## 📝 验证步骤

### 步骤 1: 创建全新插件 ✅

```bash
python3 tools/navsim_create_plugin.py \
    --name StraightPathPlanner \
    --type planner \
    --output plugins/planning/straight_path_planner \
    --author "NavSim Team" \
    --description "Simple straight-line path planner for validation" \
    --verbose
```

**结果**: ✅ 插件创建成功，生成了所有必要的文件

---

### 步骤 2: 添加到构建系统 ✅

在 `plugins/planning/CMakeLists.txt` 中添加：
```cmake
add_subdirectory(straight_path_planner)
list(APPEND PLANNING_PLUGIN_LIBS straight_path_planner_plugin)
```

**结果**: ✅ CMake 配置成功

---

### 步骤 3: 编译插件（不修改任何代码） ❌

```bash
cd build
make straight_path_planner_plugin -j4
```

**结果**: ❌ **编译失败**，发现多个严重问题

---

## 🔴 发现的严重问题

### 问题 1: `algorithm/` 层违反设计原则 🔴

**问题描述**: `algorithm/straight_path_planner.hpp` 使用了 `nlohmann::json`

```cpp
// algorithm/straight_path_planner.hpp:34
static Config fromJson(const nlohmann::json& json);  // ❌ 违反设计原则
```

**错误信息**:
```
error: 'nlohmann' does not name a type
```

**设计原则**: 
> `algorithm/` 层应该是**纯算法层**，不依赖任何平台 API，只使用标准库和 Eigen。

**影响**: 
- ❌ 编译失败
- ❌ 违反三层架构设计原则
- ❌ 降低算法的可复用性

**正确做法**:
```cpp
// algorithm/ 层不应该知道 JSON
struct Config {
    double max_velocity = 2.0;
    double max_acceleration = 2.0;
    double step_size = 0.1;
    int max_iterations = 1000;
    
    // ❌ 不应该有 fromJson
    // static Config fromJson(const nlohmann::json& json);
};
```

JSON 解析应该在 `adapter/` 层完成：
```cpp
// adapter/ 层负责 JSON → Config 转换
bool StraightPathPlannerPlugin::initialize(const nlohmann::json& config) {
    algorithm::StraightPathPlanner::Config algo_config;
    
    if (config.contains("max_velocity")) {
        algo_config.max_velocity = config["max_velocity"].get<double>();
    }
    // ...
    
    planner_->setConfig(algo_config);
}
```

---

### 问题 2: `adapter/` 层接口与平台不匹配 🔴

#### 问题 2.1: `PlannerPluginMetadata` 字段错误

**生成的代码**:
```cpp
// adapter/straight_path_planner_plugin.cpp:17-19
metadata.requires_occupancy_grid = false;  // ❌ 字段不存在
metadata.requires_esdf_map = false;        // ❌ 字段不存在
metadata.requires_lane_lines = false;      // ❌ 字段不存在
```

**错误信息**:
```
error: 'struct navsim::plugin::PlannerPluginMetadata' has no member named 'requires_occupancy_grid'
error: 'struct navsim::plugin::PlannerPluginMetadata' has no member named 'requires_esdf_map'
error: 'struct navsim::plugin::PlannerPluginMetadata' has no member named 'requires_lane_lines'
```

**实际的 `PlannerPluginMetadata` 定义**:
```cpp
struct PlannerPluginMetadata : public PluginMetadata {
    std::string type;                              // ✅ 规划器类型
    std::vector<std::string> required_perception_data;  // ✅ 必需的感知数据
    bool can_be_fallback = false;                  // ✅ 是否可以作为降级规划器
};
```

**正确的代码**:
```cpp
navsim::plugin::PlannerPluginMetadata metadata;
metadata.name = "StraightPathPlanner";
metadata.version = "1.0.0";
metadata.author = "NavSim Team";
metadata.description = "Simple straight-line path planner for validation";
metadata.type = "geometric";  // ✅ 正确
metadata.required_perception_data = {};  // ✅ 正确（不需要感知数据）
metadata.can_be_fallback = true;  // ✅ 正确（可以作为降级规划器）
```

---

#### 问题 2.2: `plan()` 方法签名不匹配

**生成的代码**:
```cpp
// adapter/straight_path_planner_plugin.hpp:63-65
bool plan(
    const navsim::planning::PlanningContext& context,
    navsim::plugin::PlanningResult& result) override;  // ❌ 缺少 deadline 参数
```

**实际的接口定义**:
```cpp
virtual bool plan(
    const planning::PlanningContext& context,
    std::chrono::milliseconds deadline,  // ✅ 必需参数
    PlanningResult& result) = 0;
```

**错误信息**:
```
error: 'bool straight_path_planner::adapter::StraightPathPlannerPlugin::plan(...)' marked 'override', but does not override
```

**正确的代码**:
```cpp
bool plan(
    const navsim::planning::PlanningContext& context,
    std::chrono::milliseconds deadline,  // ✅ 添加 deadline 参数
    navsim::plugin::PlanningResult& result) override;
```

---

#### 问题 2.3: 缺少 `isAvailable()` 方法

**生成的代码**: ❌ 完全缺少此方法

**实际的接口定义**:
```cpp
virtual std::pair<bool, std::string> isAvailable(
    const planning::PlanningContext& context) const = 0;  // ✅ 必需方法
```

**错误信息**:
```
error: invalid new-expression of abstract class type 'straight_path_planner::adapter::StraightPathPlannerPlugin'
note: because the following virtual functions are pure within '...':
note:     'virtual std::pair<bool, std::__cxx11::basic_string<char> > navsim::plugin::PlannerPluginInterface::isAvailable(...)'
```

**正确的代码**:
```cpp
// adapter/straight_path_planner_plugin.hpp
std::pair<bool, std::string> isAvailable(
    const navsim::planning::PlanningContext& context) const override;

// adapter/straight_path_planner_plugin.cpp
std::pair<bool, std::string> StraightPathPlannerPlugin::isAvailable(
    const navsim::planning::PlanningContext& context) const {
    // 简单的规划器不需要任何感知数据
    return {true, ""};
}
```

---

#### 问题 2.4: `Trajectory` 类型不存在

**生成的代码**:
```cpp
// adapter/straight_path_planner_plugin.hpp:90
navsim::planning::Trajectory convertTrajectory(...);  // ❌ 类型不存在
```

**错误信息**:
```
error: 'Trajectory' in namespace 'navsim::planning' does not name a type
```

**实际情况**: 查看 `PlanningResult` 定义，轨迹数据存储在 `result.trajectory_points`

**正确的做法**: 直接填充 `result.trajectory_points`，不需要单独的 `Trajectory` 类型

---

## 📊 问题总结

| 问题 | 严重性 | 影响 | 状态 |
|------|--------|------|------|
| `algorithm/` 层使用 JSON | 🔴 高 | 编译失败 + 违反设计原则 | ❌ 未修复 |
| `PlannerPluginMetadata` 字段错误 | 🔴 高 | 编译失败 | ❌ 未修复 |
| `plan()` 签名不匹配 | 🔴 高 | 编译失败 | ❌ 未修复 |
| 缺少 `isAvailable()` 方法 | 🔴 高 | 编译失败 | ❌ 未修复 |
| `Trajectory` 类型不存在 | 🔴 高 | 编译失败 | ❌ 未修复 |

**结论**: 工具生成的代码**完全无法编译**，需要大量修改才能使用。

---

## 🎯 验证结论

### ❌ 验证失败

**原因**: 工具生成的代码与实际平台接口**严重不匹配**，无法编译。

**问题根源**: 模板代码**过时**或**从未与实际平台接口同步**。

### 📉 实际可用性评估

| 指标 | 预期 | 实际 | 差距 |
|------|------|------|------|
| **开箱即用** | 100% | 0% | -100% |
| **编译成功率** | 100% | 0% | -100% |
| **需要修改的文件** | 0 | 5+ | +5 |
| **需要修改的行数** | 0 | 50+ | +50 |

### 🔍 与 JpsPlanner 验证的对比

**JpsPlanner 验证**:
- ✅ 编译成功（修复 CMakeLists.txt 后）
- ✅ 运行正常
- ⚠️ 但我们**复制了旧插件的 `adapter/` 代码**，没有使用生成的代码

**StraightPathPlanner 验证**:
- ❌ 编译失败（多个严重错误）
- ❌ 无法运行
- ✅ 但我们**真正验证了生成的代码**

**结论**: JpsPlanner 验证**掩盖了模板的严重问题**，因为我们没有使用生成的 `adapter/` 代码。

---

## 🔧 需要修复的模板文件

### 1. `templates/planner_plugin/algorithm/{{PLUGIN_NAME_SNAKE}}.hpp`

**问题**: 使用了 `nlohmann::json`

**修复**: 移除 `fromJson()` 方法，保持 `Config` 为纯数据结构

---

### 2. `templates/planner_plugin/algorithm/{{PLUGIN_NAME_SNAKE}}.cpp`

**问题**: 实现了 `fromJson()` 方法

**修复**: 移除 `fromJson()` 实现

---

### 3. `templates/planner_plugin/adapter/{{PLUGIN_NAME_SNAKE}}_plugin.hpp`

**问题**: 
- `plan()` 签名缺少 `deadline` 参数
- 缺少 `isAvailable()` 方法
- `convertTrajectory()` 返回不存在的类型

**修复**:
```cpp
// 添加 deadline 参数
bool plan(
    const navsim::planning::PlanningContext& context,
    std::chrono::milliseconds deadline,
    navsim::plugin::PlanningResult& result) override;

// 添加 isAvailable() 方法
std::pair<bool, std::string> isAvailable(
    const navsim::planning::PlanningContext& context) const override;

// 移除 convertTrajectory()，直接填充 result.trajectory_points
```

---

### 4. `templates/planner_plugin/adapter/{{PLUGIN_NAME_SNAKE}}_plugin.cpp`

**问题**:
- `getMetadata()` 使用了不存在的字段
- `initialize()` 调用了不存在的 `fromJson()`
- `plan()` 签名不匹配
- 缺少 `isAvailable()` 实现
- `convertTrajectory()` 返回不存在的类型

**修复**: 重写整个文件以匹配实际平台接口

---

## 📈 改进优先级

### 🔴 紧急（必须立即修复）

1. **修复 `adapter/` 层模板** - 使其与实际平台接口匹配
2. **移除 `algorithm/` 层的 JSON 依赖** - 保持纯算法层
3. **添加 `isAvailable()` 方法** - 必需的接口方法
4. **修复 `plan()` 签名** - 添加 `deadline` 参数

### 🟡 重要（短期修复）

5. **添加实际平台接口的单元测试** - 防止模板再次过时
6. **创建模板验证脚本** - 自动验证生成的代码可以编译
7. **更新文档** - 说明模板的限制和已知问题

### 🟢 改进（长期优化）

8. **自动同步模板与平台接口** - 从平台接口自动生成模板
9. **提供多个模板变体** - 适应不同类型的规划器
10. **添加更多示例** - 展示如何使用生成的代码

---

## 🎯 下一步行动

1. **立即修复模板** - 使生成的代码可以编译
2. **重新验证** - 使用修复后的模板创建新插件
3. **更新文档** - 记录修复过程和经验教训
4. **添加 CI 测试** - 确保模板始终与平台接口同步

---

## 📚 相关文档

- [插件脚手架验证报告](PLUGIN_SCAFFOLDING_VALIDATION.md) - JpsPlanner 迁移验证（掩盖了问题）
- [插件脚手架改进建议](PLUGIN_SCAFFOLDING_IMPROVEMENTS.md) - 改进计划
- [开发工具指南](DEVELOPMENT_TOOLS.md) - 工具使用说明

---

## 🎉 总结

### 重要发现

1. **JpsPlanner 验证掩盖了问题**: 因为我们复制了旧插件的 `adapter/` 代码，没有真正验证生成的代码
2. **模板严重过时**: 生成的代码与实际平台接口完全不匹配
3. **设计原则被违反**: `algorithm/` 层使用了平台 API (JSON)

### 验证价值

虽然验证失败了，但这次验证**非常有价值**：
- ✅ 发现了模板的严重问题
- ✅ 明确了需要修复的内容
- ✅ 避免了用户使用有问题的工具

### 下一步

**立即修复模板**，然后重新进行"全新插件"验证。


# NavSim 插件系统说明书

## 目录

- [概述](#概述)
- [插件系统架构](#插件系统架构)
- [插件加载机制](#插件加载机制)
- [插件接口说明](#插件接口说明)
- [内置插件列表](#内置插件列表)
- [配置文件格式](#配置文件格式)
- [常见问题](#常见问题)

---

## 概述

NavSim 插件系统是一个灵活、可扩展的架构，允许用户动态加载和使用不同的感知和规划算法。插件系统采用**三层解耦架构**，将算法实现、平台适配和核心框架分离，确保代码的可维护性和可扩展性。

### 核心特性

- **动态加载**：支持运行时动态加载插件（`.so` 文件）
- **静态链接**：支持编译时静态链接内置插件
- **版本兼容性检查**：自动检查插件与平台的兼容性
- **统一路径解析**：支持短名称和完整路径两种加载方式
- **三层解耦架构**：算法层、适配层、平台层分离

---

## 插件系统架构

### 三层解耦架构

NavSim 插件系统采用三层架构设计：

```
┌─────────────────────────────────────────────────────────────┐
│                      Platform Layer                          │
│  (平台层 - 插件框架、数据结构、接口定义)                      │
│  - PlannerPluginInterface                                    │
│  - PerceptionPluginInterface                                 │
│  - PlanningContext                                           │
│  - PlanningResult                                            │
└─────────────────────────────────────────────────────────────┘
                            ▲
                            │ 实现接口
                            │
┌─────────────────────────────────────────────────────────────┐
│                      Adapter Layer                           │
│  (适配层 - 平台接口适配)                                      │
│  - 实现平台插件接口                                           │
│  - 转换平台数据结构 ↔ 算法数据结构                            │
│  - 处理 JSON 配置解析                                         │
│  - 错误处理和日志                                             │
└─────────────────────────────────────────────────────────────┘
                            ▲
                            │ 调用算法
                            │
┌─────────────────────────────────────────────────────────────┐
│                     Algorithm Layer                          │
│  (算法层 - 纯算法实现)                                        │
│  - 纯算法逻辑                                                 │
│  - 只依赖 Eigen + STL                                         │
│  - 无平台依赖                                                 │
│  - 无 JSON 解析                                               │
└─────────────────────────────────────────────────────────────┘
```

### 设计原则

1. **单一职责原则 (SRP)**
   - 感知插件负责感知，规划插件负责规划
   - 每个插件只做一件事

2. **开闭原则 (OCP)**
   - 可以添加新插件而不修改现有代码
   - 通过配置文件切换插件

3. **依赖倒置原则 (DIP)**
   - 规划器依赖抽象的 `PlanningContext`，而不是具体的感知插件
   - 通过接口而不是实现进行交互

4. **算法层纯净性**
   - 算法层不依赖平台 API
   - 算法层不解析 JSON
   - 算法层只使用 Eigen + STL

---

## 插件加载机制

### 加载方式

NavSim 支持两种插件加载方式：

#### 1. 动态加载（推荐）

通过 `dlopen/dlsym` 动态加载 `.so` 文件：

```bash
./navsim_local_debug --planner StraightLine --scenario scenarios/simple_corridor.json
```

**优点**：
- 无需重新编译平台
- 可以独立开发和测试插件
- 支持第三方插件

**插件查找路径**（按优先级）：
1. `$NAVSIM_PLUGIN_PATH` 环境变量指定的路径
2. `build/plugins/planning/{plugin_name}/lib{plugin_name}_plugin.so`
3. `build/plugins/perception/{plugin_name}/lib{plugin_name}_plugin.so`
4. `plugins/planning/{plugin_name}/lib{plugin_name}_plugin.so`
5. `plugins/perception/{plugin_name}/lib{plugin_name}_plugin.so`

#### 2. 静态链接

编译时链接内置插件：

```cmake
target_link_libraries(navsim_local_debug
    navsim_builtin_plugins  # 包含所有内置插件
)
```

**优点**：
- 启动速度快
- 无需管理 `.so` 文件
- 适合内置插件

**缺点**：
- 修改插件需要重新编译平台
- 无法动态添加插件

### 插件注册机制

NavSim 使用**双重注册机制**确保插件被正确注册：

#### 1. 动态注册（dlsym）

```cpp
extern "C" {
  void registerStraightLinePlugin() {
    straight_line::adapter::registerStraightLinePlugin();
  }
}
```

#### 2. 静态注册（静态初始化器）

```cpp
namespace {
  struct StraightLinePluginInitializer {
    StraightLinePluginInitializer() {
      straight_line::adapter::registerStraightLinePlugin();
    }
  };
  static StraightLinePluginInitializer g_straight_line_initializer;
}
```

### 版本兼容性检查

插件加载时会检查版本兼容性：

```cpp
navsim::plugin::PlannerPluginMetadata metadata = plugin->getMetadata();
// 检查 metadata.version 是否兼容
```

---

## 插件接口说明

### 规划器插件接口

所有规划器插件必须实现 `PlannerPluginInterface`：

```cpp
class PlannerPluginInterface {
public:
  // 获取插件元数据
  virtual PlannerPluginMetadata getMetadata() const = 0;

  // 初始化插件
  virtual bool initialize(const nlohmann::json& config) = 0;

  // 重置插件状态
  virtual void reset() = 0;

  // 检查插件是否可用
  virtual std::pair<bool, std::string> isAvailable(
      const PlanningContext& context) const = 0;

  // 执行规划
  virtual PlanningResult plan(
      const PlanningContext& context,
      std::chrono::milliseconds deadline) = 0;

  // 获取统计信息
  virtual nlohmann::json getStatistics() const = 0;
};
```

### 感知插件接口

所有感知插件必须实现 `PerceptionPluginInterface`：

```cpp
class PerceptionPluginInterface {
public:
  // 获取插件元数据
  virtual PerceptionPluginMetadata getMetadata() const = 0;

  // 初始化插件
  virtual bool initialize(const nlohmann::json& config) = 0;

  // 重置插件状态
  virtual void reset() = 0;

  // 处理感知数据
  virtual bool process(PlanningContext& context) = 0;

  // 获取统计信息
  virtual nlohmann::json getStatistics() const = 0;
};
```

### 插件元数据

#### 规划器元数据

```cpp
struct PlannerPluginMetadata {
  std::string name;                          // 插件名称
  std::string version;                       // 版本号
  std::string description;                   // 描述
  std::string author;                        // 作者
  std::string type;                          // 类型（geometric, search, optimization）
  std::vector<std::string> required_perception_data;  // 需要的感知数据
  bool can_be_fallback;                      // 是否可以作为备用规划器
};
```

#### 感知器元数据

```cpp
struct PerceptionPluginMetadata {
  std::string name;                          // 插件名称
  std::string version;                       // 版本号
  std::string description;                   // 描述
  std::string author;                        // 作者
  std::string type;                          // 类型（grid_map, esdf, pointcloud）
  std::vector<std::string> provides_data;    // 提供的数据类型
};
```

---

## 内置插件列表

### 规划器插件

| 插件名称 | 类型 | 描述 | 需要的感知数据 | 可作为备用 |
|---------|------|------|---------------|-----------|
| **StraightLine** | geometric | 直线路径规划器，支持梯形速度曲线 | 无 | ✅ |
| **JpsPlanner** | search | Jump Point Search 路径规划器 | esdf_map | ❌ |

### 感知器插件

| 插件名称 | 类型 | 描述 | 提供的数据 |
|---------|------|------|-----------|
| **GridMapBuilder** | grid_map | 栅格地图构建器 | grid_map |
| **ESDFBuilder** | esdf | ESDF 地图构建器 | esdf_map |

---

## 配置文件格式

### 场景配置文件

场景配置文件使用 JSON 格式，定义了规划任务的所有参数：

```json
{
  "scenario_name": "simple_corridor",
  "description": "Simple corridor scenario",
  
  "ego_state": {
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "twist": {"vx": 0.0, "vy": 0.0, "omega": 0.0}
  },
  
  "goal": {
    "pose": {"x": 20.0, "y": 0.0, "yaw": 0.0}
  },
  
  "planner_config": {
    "StraightLine": {
      "default_velocity": 2.0,
      "time_step": 0.1,
      "planning_horizon": 5.0,
      "use_trapezoidal_profile": true,
      "max_acceleration": 2.0
    },
    "JpsPlanner": {
      "safe_dis": 0.3,
      "max_vel": 2.0,
      "max_acc": 2.0,
      "max_omega": 1.0
    }
  }
}
```

### 插件配置参数

#### StraightLine 插件

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `default_velocity` | double | 2.0 | 默认速度 (m/s) |
| `time_step` | double | 0.1 | 时间步长 (s) |
| `planning_horizon` | double | 5.0 | 规划时域 (s) |
| `use_trapezoidal_profile` | bool | true | 是否使用梯形速度曲线 |
| `max_acceleration` | double | 2.0 | 最大加速度 (m/s²) |

#### JpsPlanner 插件

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `safe_dis` | double | 0.3 | 安全距离 (m) |
| `max_vel` | double | 2.0 | 最大速度 (m/s) |
| `max_acc` | double | 2.0 | 最大加速度 (m/s²) |
| `max_omega` | double | 1.0 | 最大角速度 (rad/s) |

---

## 常见问题

### Q1: 如何添加新插件？

参见 [插件开发指南](PLUGIN_DEVELOPMENT_GUIDE.md)。

### Q2: 插件加载失败怎么办？

**检查步骤**：
1. 确认插件 `.so` 文件存在
2. 检查插件路径是否正确
3. 查看错误日志
4. 确认插件版本兼容性

### Q3: 如何切换插件？

通过命令行参数或配置文件：

```bash
# 命令行
./navsim_local_debug --planner StraightLine

# 配置文件
{
  "planner": "StraightLine"
}
```

### Q4: 插件之间如何通信？

通过 `PlanningContext` 共享数据：

```cpp
// 感知插件写入数据
context.setCustomData("esdf_map", esdf_map);

// 规划插件读取数据
auto esdf_map = context.getCustomData<ESDFMap>("esdf_map");
```

### Q5: 如何调试插件？

1. **启用详细日志**：
   ```bash
   ./navsim_local_debug --planner StraightLine --verbose
   ```

2. **使用 GDB**：
   ```bash
   gdb --args ./navsim_local_debug --planner StraightLine
   ```

3. **查看统计信息**：
   ```cpp
   nlohmann::json stats = plugin->getStatistics();
   std::cout << stats.dump(2) << std::endl;
   ```

---

## 参考资料

- [插件开发指南](PLUGIN_DEVELOPMENT_GUIDE.md)
- [重构提案](../REFACTORING_PROPOSAL.md)
- [API 文档](API_REFERENCE.md)

---

**版本**: 1.0.0  
**最后更新**: 2025-10-18  
**作者**: NavSim Team


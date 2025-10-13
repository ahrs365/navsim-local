# 插件系统快速参考

## 🎯 核心概念

### 插件类型

| 类型 | 接口 | 用途 |
|------|------|------|
| **感知插件** | `PerceptionPluginInterface` | 处理感知数据，生成规划上下文 |
| **规划器插件** | `PlannerPluginInterface` | 执行轨迹规划算法 |

---

## 📝 插件接口速查

### 感知插件必须实现的方法

**重要**: 感知插件接收标准化的 `PerceptionInput`，而不是原始的 `proto::WorldTick`

```cpp
class MyPerceptionPlugin : public PerceptionPluginInterface {
public:
  // 1. 元信息
  Metadata getMetadata() const override;

  // 2. 初始化
  bool initialize(const nlohmann::json& config) override;

  // 3. 处理函数 (核心) - 注意输入是 PerceptionInput
  bool process(const PerceptionInput& input,
              planning::PlanningContext& context) override;

  // 4. 可选方法
  void reset() override;
  nlohmann::json getStatistics() const override;
};

// 5. 注册
REGISTER_PERCEPTION_PLUGIN(MyPerceptionPlugin)
```

**PerceptionInput 结构**:
```cpp
struct PerceptionInput {
  planning::EgoVehicle ego;                    // 自车状态
  planning::PlanningTask task;                 // 任务目标
  planning::BEVObstacles bev_obstacles;        // BEV障碍物(已解析)
  std::vector<planning::DynamicObstacle> dynamic_obstacles;  // 动态障碍物(已解析)
  const proto::WorldTick* raw_world_tick;      // 原始数据(可选)
};
```

### 规划器插件必须实现的方法

```cpp
class MyPlannerPlugin : public PlannerPluginInterface {
public:
  // 1. 元信息
  Metadata getMetadata() const override;
  
  // 2. 初始化
  bool initialize(const nlohmann::json& config) override;
  
  // 3. 规划函数 (核心)
  bool plan(const planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           planning::PlanningResult& result) override;
  
  // 4. 可用性检查
  std::pair<bool, std::string> isAvailable(
      const planning::PlanningContext& context) const override;
  
  // 5. 可选方法
  void reset() override;
  nlohmann::json getStatistics() const override;
};

// 6. 注册
REGISTER_PLANNER_PLUGIN(MyPlannerPlugin)
```

---

## 🔧 配置文件速查

### 感知插件配置

```json
{
  "perception": {
    "preprocessing": {
      "bev_extraction": {
        "detection_range": 50.0,
        "confidence_threshold": 0.5
      },
      "dynamic_prediction": {
        "prediction_horizon": 5.0,
        "time_step": 0.1
      }
    },
    "plugins": [
      {
        "name": "PluginName",
        "enabled": true,
        "priority": 1,
        "params": {
          "param1": value1,
          "param2": value2
        }
      }
    ]
  }
}
```

### 规划器插件配置

```json
{
  "planning": {
    "primary_planner": "PrimaryPlannerName",
    "fallback_planner": "FallbackPlannerName",
    "enable_fallback": true,
    "planners": {
      "PlannerName": {
        "param1": value1,
        "param2": value2
      }
    }
  }
}
```

---

## 📦 数据结构速查

### 输入数据 (感知插件)

**重要**: 感知插件接收的是标准化的 `PerceptionInput`，而不是原始的 `proto::WorldTick`

```cpp
// PerceptionInput 包含:
input.ego                     // 自车状态 (已转换)
input.task                    // 任务目标 (已转换)
input.bev_obstacles           // BEV障碍物 (已解析)
input.dynamic_obstacles       // 动态障碍物预测 (已解析)
input.raw_world_tick          // 原始数据 (可选，指针)
input.timestamp               // 时间戳
input.tick_id                 // Tick ID
```

**访问标准化数据**:
```cpp
// BEV 障碍物
const auto& circles = input.bev_obstacles.circles;
const auto& rectangles = input.bev_obstacles.rectangles;
const auto& polygons = input.bev_obstacles.polygons;

// 动态障碍物预测
for (const auto& obstacle : input.dynamic_obstacles) {
  const auto& current_pose = obstacle.current_pose;
  const auto& trajectories = obstacle.predicted_trajectories;
}
```

**访问原始数据 (可选)**:
```cpp
if (input.hasRawData()) {
  const auto& world_tick = *input.raw_world_tick;
  // 访问原始数据...
}
```

### 输出数据 (感知插件)

```cpp
// planning::PlanningContext 可填充:
context.occupancy_grid        // 栅格地图 (unique_ptr)
context.point_cloud_map       // 点云地图 (unique_ptr)
context.esdf_map              // ESDF距离场 (unique_ptr)
context.custom_data           // 自定义数据 (map)

// 注意: ego, task, dynamic_obstacles 已由前置处理层填充
```

### 输入数据 (规划器插件)

```cpp
// planning::PlanningContext (同上)
```

### 输出数据 (规划器插件)

```cpp
// planning::PlanningResult 需填充:
result.trajectory             // 轨迹点 (vector<TrajectoryPoint>)
result.control_cmd            // 控制指令 (可选)
result.status                 // 规划状态 (SUCCESS/FAILED/...)
result.cost                   // 代价信息
result.diagnostics            // 诊断信息
```

---

## 🛠️ 常用代码片段

### 读取配置参数

```cpp
bool initialize(const nlohmann::json& config) override {
  // 必需参数
  if (!config.contains("required_param")) {
    std::cerr << "Missing required parameter" << std::endl;
    return false;
  }
  
  // 可选参数 (带默认值)
  param1_ = config.value("param1", 1.0);
  param2_ = config.value("param2", "default");
  
  // 嵌套参数
  if (config.contains("nested")) {
    nested_param_ = config["nested"].value("key", 0);
  }
  
  return true;
}
```

### 检查超时

```cpp
bool plan(..., std::chrono::milliseconds deadline, ...) override {
  auto start = std::chrono::steady_clock::now();
  
  while (/* 迭代条件 */) {
    // 检查超时
    auto now = std::chrono::steady_clock::now();
    if (now - start > deadline) {
      result.status = PlanningResult::Status::TIMEOUT;
      return false;
    }
    
    // 迭代逻辑...
  }
  
  return true;
}
```

### 错误处理

```cpp
bool process(...) override {
  try {
    // 处理逻辑
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[" << getMetadata().name << "] Exception: " 
              << e.what() << std::endl;
    return false;
  }
}
```

### 日志输出

```cpp
// 信息日志
std::cout << "[" << getMetadata().name << "] Info message" << std::endl;

// 错误日志
std::cerr << "[" << getMetadata().name << "] ERROR: Error message" << std::endl;

// 警告日志
std::cerr << "[" << getMetadata().name << "] WARNING: Warning message" << std::endl;
```

### 访问自定义数据

```cpp
// 设置自定义数据
auto my_data = std::make_shared<MyDataType>();
context.setCustomData("my_key", my_data);

// 获取自定义数据
auto data = context.getCustomData<MyDataType>("my_key");
if (data) {
  // 使用 data
}
```

---

## 📋 开发检查清单

### 感知插件开发

- [ ] 实现 `getMetadata()` - 提供插件元信息
- [ ] 实现 `initialize()` - 读取配置参数
- [ ] 实现 `process()` - 核心处理逻辑
- [ ] 实现 `reset()` (可选) - 重置状态
- [ ] 实现 `getStatistics()` (可选) - 统计信息
- [ ] 添加 `REGISTER_PERCEPTION_PLUGIN()` 宏
- [ ] 添加到 CMakeLists.txt
- [ ] 创建配置文件示例
- [ ] 编写单元测试
- [ ] 更新文档

### 规划器插件开发

- [ ] 实现 `getMetadata()` - 提供插件元信息
- [ ] 实现 `initialize()` - 读取配置参数
- [ ] 实现 `plan()` - 核心规划逻辑
- [ ] 实现 `isAvailable()` - 可用性检查
- [ ] 实现 `reset()` (可选) - 重置状态
- [ ] 实现 `getStatistics()` (可选) - 统计信息
- [ ] 添加 `REGISTER_PLANNER_PLUGIN()` 宏
- [ ] 添加到 CMakeLists.txt
- [ ] 创建配置文件示例
- [ ] 编写单元测试
- [ ] 更新文档

---

## 🐛 常见问题

### Q1: 插件未被加载？

**检查**:
1. 是否添加了 `REGISTER_*_PLUGIN()` 宏？
2. 是否添加到 CMakeLists.txt？
3. 配置文件中插件名称是否正确？
4. 插件是否被 `enabled: true`？

### Q2: 配置参数读取失败？

**检查**:
1. JSON 格式是否正确？
2. 参数名称是否匹配？
3. 是否使用了 `config.value()` 提供默认值？

### Q3: 规划器不可用？

**检查**:
1. `isAvailable()` 返回了什么？
2. 是否缺少必需的感知数据？
3. 配置参数是否有效？

### Q4: 性能问题？

**优化**:
1. 避免在 `process()`/`plan()` 中分配大量内存
2. 使用成员变量缓存可复用数据
3. 注意截止时间 (`deadline`)
4. 使用性能分析工具

---

## 📚 更多资源

- **[详细设计文档](PLUGIN_ARCHITECTURE_DESIGN.md)** - 完整架构设计
- **[执行摘要](PLUGIN_ARCHITECTURE_SUMMARY.md)** - 核心要点总结
- **[插件开发指南](PLUGIN_DEVELOPMENT_GUIDE.md)** - 详细开发教程 (待创建)
- **[示例插件](../plugins/examples/)** - 参考实现

---

**提示**: 这是快速参考文档，详细信息请查看完整设计文档。


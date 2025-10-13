# NavSim-Local 插件化架构重构方案 - 执行摘要

## 📋 概述

本文档是 `PLUGIN_ARCHITECTURE_DESIGN.md` 的执行摘要，提供重构方案的核心要点。

---

## 🎯 重构目标

将 navsim-local 从硬编码架构重构为**插件化、可扩展、可配置**的架构：

1. ✅ **感知处理插件化** - 用户可自定义感知数据转换
2. ✅ **规划器插件化** - 用户可新增和适配不同规划算法
3. ✅ **配置驱动** - 通过配置文件控制插件加载和参数
4. ✅ **向后兼容** - 现有功能完全保留

---

## 🏗️ 核心架构

### 架构分层

```
Application Layer (main.cpp, ConfigLoader)
           ↓
Algorithm Manager (协调感知+规划)
           ↓
    ┌──────────────┴──────────────┐
    ↓                             ↓
Perception Plugin Manager    Planning Plugin Manager
    ↓                             ↓
Plugin Registry              Plugin Registry
    ↓                             ↓
Perception Plugins           Planner Plugins
```

### 核心组件

| 组件 | 职责 |
|------|------|
| **PluginInterface** | 定义插件抽象接口 |
| **PluginRegistry** | 插件注册和发现（单例+工厂） |
| **PluginManager** | 插件生命周期管理 |
| **ConfigLoader** | 配置文件加载（JSON） |

---

## 🔌 插件接口设计

### 感知插件接口

**架构分层**:
```
proto::WorldTick
    ↓
[公共前置处理层] - BEV提取 + 动态障碍物预测
    ↓
PerceptionInput (标准化数据)
    ↓
[感知插件层] - 构建地图表示
    ↓
PlanningContext
```

**插件接口**:
```cpp
// 标准化输入数据
struct PerceptionInput {
  planning::EgoVehicle ego;
  planning::PlanningTask task;
  planning::BEVObstacles bev_obstacles;  // 已解析
  std::vector<planning::DynamicObstacle> dynamic_obstacles;  // 已解析
  const proto::WorldTick* raw_world_tick;  // 可选
};

// 感知插件接口
class PerceptionPluginInterface {
public:
  virtual Metadata getMetadata() const = 0;
  virtual bool initialize(const nlohmann::json& config) = 0;
  virtual bool process(const PerceptionInput& input,
                      planning::PlanningContext& context) = 0;
  virtual void reset() {}
};

// 注册宏
REGISTER_PERCEPTION_PLUGIN(MyPlugin)
```

**插件示例**:
- `GridMapBuilderPlugin` - 构建栅格占据地图
- `ESDFBuilderPlugin` - 构建 ESDF 距离场
- `PointCloudMapBuilderPlugin` - 构建点云地图
- 用户自定义插件 - 构建其他地图表示

### 规划器插件接口

```cpp
class PlannerPluginInterface {
public:
  virtual Metadata getMetadata() const = 0;
  virtual bool initialize(const nlohmann::json& config) = 0;
  virtual bool plan(const planning::PlanningContext& context,
                   std::chrono::milliseconds deadline,
                   planning::PlanningResult& result) = 0;
  virtual std::pair<bool, std::string> isAvailable(
      const planning::PlanningContext& context) const = 0;
  virtual void reset() {}
};

// 注册宏
REGISTER_PLANNER_PLUGIN(MyPlanner)
```

---

## ⚙️ 配置系统

### 配置文件格式

**推荐**: JSON (已有依赖 nlohmann/json)

### 配置文件示例

```json
{
  "version": "1.0",
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
        "name": "GridMapBuilderPlugin",
        "enabled": true,
        "priority": 1,
        "params": {
          "resolution": 0.1,
          "map_width": 100.0
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
        "heuristic_weight": 1.0,
        "max_iterations": 10000
      }
    }
  }
}
```

### 配置优先级

1. 命令行参数（最高）
2. 环境变量
3. 配置文件
4. 代码默认值（最低）

---

## 📁 目录结构调整

```
navsim-local/
├── config/                    # 配置文件 (新增)
│   ├── default.json
│   └── examples/
├── include/
│   ├── plugin/                # 插件系统核心 (新增)
│   │   ├── perception_plugin_interface.hpp
│   │   ├── planner_plugin_interface.hpp
│   │   ├── plugin_registry.hpp
│   │   └── *_plugin_manager.hpp
│   └── config/                # 配置系统 (新增)
│       └── config_loader.hpp
├── src/
│   ├── plugin/                # 插件系统实现 (新增)
│   └── config/                # 配置系统实现 (新增)
├── plugins/                   # 插件实现 (新增)
│   ├── perception/
│   │   ├── grid_map_builder_plugin.cpp
│   │   ├── esdf_builder_plugin.cpp
│   │   └── ...
│   ├── planning/
│   │   ├── straight_line_planner_plugin.cpp
│   │   ├── astar_planner_plugin.cpp
│   │   └── ...
│   └── examples/              # 自定义插件示例
│       ├── custom_perception_plugin/
│       └── custom_planner_plugin/
└── docs/
    ├── PLUGIN_ARCHITECTURE_DESIGN.md      # 详细设计文档
    ├── PLUGIN_DEVELOPMENT_GUIDE.md        # 插件开发指南 (待创建)
    └── MIGRATION_GUIDE.md                 # 迁移指南 (待创建)
```

---

## 🔄 向后兼容性

### 兼容性保证

| 层面 | 策略 |
|------|------|
| **API 兼容** | 保留现有类和接口 |
| **配置兼容** | 提供默认配置 |
| **行为兼容** | 默认行为不变 |
| **编译兼容** | 无需修改 CMake |

### 迁移路径

```bash
# 阶段 1: 无感知迁移 (默认行为)
./build/navsim_algo ws://127.0.0.1:8080/ws demo

# 阶段 2: 使用配置文件
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config.json

# 阶段 3: 自定义插件
# 开发自定义插件并集成
```

---

## 🚀 如何添加新插件

### 3 步添加自定义插件

#### 步骤 1: 实现插件类

```cpp
// plugins/planning/my_planner.cpp
class MyPlannerPlugin : public PlannerPluginInterface {
  // 实现接口方法...
};

REGISTER_PLANNER_PLUGIN(MyPlannerPlugin)
```

#### 步骤 2: 添加到 CMake

```cmake
# plugins/planning/CMakeLists.txt
add_library(navsim_planning_plugins STATIC
    ...
    my_planner.cpp  # 新增
)
```

#### 步骤 3: 配置文件中启用

```json
{
  "planning": {
    "primary_planner": "MyPlannerPlugin",
    "planners": {
      "MyPlannerPlugin": {
        "my_param": 1.0
      }
    }
  }
}
```

### 编译运行

```bash
cd navsim-local
rm -rf build
cmake -B build -S .
cmake --build build
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=my_config.json
```

---

## 📅 实施计划

### 分阶段实施 (4-6 周)

| 阶段 | 时间 | 目标 | 交付物 |
|------|------|------|--------|
| **Phase 1** | 1-2 周 | 基础架构 | 插件系统核心、配置系统 |
| **Phase 2** | 1-2 周 | 插件迁移 | 所有插件实现、更新的 AlgorithmManager |
| **Phase 3** | 1 周 | 测试与文档 | 测试套件、开发指南、迁移指南 |
| **Phase 4** | 1-2 周 | 高级功能（可选） | 动态加载、热重载 |

### Phase 1 任务清单

- [ ] 定义插件接口
- [ ] 实现插件注册表
- [ ] 实现插件管理器
- [ ] 实现配置加载器
- [ ] 调整目录结构
- [ ] 编写单元测试

---

## ✅ 成功标准

- ✅ 所有现有功能正常工作
- ✅ 性能无明显退化 (< 5%)
- ✅ 可以通过配置文件切换插件
- ✅ 可以添加自定义插件无需修改核心代码
- ✅ 文档完整，示例清晰
- ✅ 所有测试通过

---

## 📊 优势总结

| 优势 | 说明 |
|------|------|
| **可扩展性** | 用户可轻松添加自定义插件 |
| **可配置性** | 通过配置文件灵活控制行为 |
| **可维护性** | 模块化设计，职责清晰 |
| **可测试性** | 插件独立测试，易于调试 |
| **向后兼容** | 现有功能完全保留 |

---

## 📚 相关文档

- **[详细设计文档](PLUGIN_ARCHITECTURE_DESIGN.md)** - 完整的架构设计方案
- **[插件开发指南](PLUGIN_DEVELOPMENT_GUIDE.md)** - 如何开发插件 (待创建)
- **[迁移指南](MIGRATION_GUIDE.md)** - 从旧版本迁移 (待创建)

---

## 📞 联系方式

- **GitHub Issues**: [ahrs-simulator/issues](https://github.com/ahrs365/ahrs-simulator/issues)
- **Email**: ahrs365@outlook.com

---

**最后更新**: 2025-10-13


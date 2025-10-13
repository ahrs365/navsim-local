# NavSim-Local 插件化重构方案 - 总览

**版本**: 2.0
**最新更新**: 2025-10-13
**状态**: 设计完成，待评审

---

## � 重要更新 (v2.0)

**感知插件架构已优化！**

我们重新设计了感知插件架构，将 BEV 提取和动态障碍物预测从插件层移至公共前置处理层。

**关键变化**:
- ✅ 感知插件现在接收标准化的 `PerceptionInput` 而不是原始的 `proto::WorldTick`
- ✅ BEV 提取和动态预测作为固定的前置处理步骤
- ✅ 感知插件专注于构建特定的地图表示（栅格地图、ESDF、点云等）
- ✅ 配置文件新增 `perception.preprocessing` 配置节

**详细说明**: 请查看 [感知插件架构更新说明](docs/PERCEPTION_ARCHITECTURE_UPDATE.md)

---

## �📋 文档索引

本次重构设计已完成，以下是所有相关文档：

### 主入口文档 🆕

0. **[插件系统主文档](PLUGIN_SYSTEM_README.md)** ⭐⭐⭐⭐⭐ **NEW**
   - **内容**: 插件化架构重构方案主入口文档（1500+ 行）
   - **包含**: 项目概述、架构设计、数据流向、工程结构、快速开始、开发指南、配置指南、使用场景、故障排查
   - **适合**: 所有用户，作为主要参考文档

### 核心设计文档

1. **[插件架构设计](docs/PLUGIN_ARCHITECTURE_DESIGN.md)** ⭐⭐⭐⭐⭐
   - **内容**: 完整的架构设计方案（1400+ 行）
   - **包含**: 架构分层、接口设计、插件管理、配置系统、目录结构、实施计划
   - **适合**: 架构师、技术负责人、需要深入了解设计的开发者
   - **更新**: v2.0 已更新感知插件接口设计

2. **[执行摘要](docs/PLUGIN_ARCHITECTURE_SUMMARY.md)** ⭐⭐⭐⭐
   - **内容**: 核心要点总结（300 行）
   - **包含**: 重构目标、核心架构、接口设计、配置系统、实施计划
   - **适合**: 项目经理、快速了解方案的开发者
   - **更新**: v2.0 已更新感知插件接口示例

3. **[快速参考](docs/PLUGIN_QUICK_REFERENCE.md)** ⭐⭐⭐⭐⭐
   - **内容**: 插件开发速查手册（250 行）
   - **包含**: 接口速查、配置速查、代码片段、开发检查清单、常见问题
   - **适合**: 插件开发者、日常开发参考
   - **更新**: v2.0 已更新感知插件输入/输出数据结构

4. **[感知插件架构详解](docs/PERCEPTION_PLUGIN_ARCHITECTURE.md)** ⭐⭐⭐⭐⭐ **NEW**
   - **内容**: 感知插件架构详细说明（300 行）
   - **包含**: 数据流图、设计原则、实现细节、配置示例、添加插件指南
   - **适合**: 感知插件开发者、需要理解感知架构的开发者

5. **[感知架构更新说明](docs/PERCEPTION_ARCHITECTURE_UPDATE.md)** ⭐⭐⭐⭐ **NEW**
   - **内容**: v2.0 架构更新详细说明（300 行）
   - **包含**: 更新动机、架构对比、主要变更、迁移指南
   - **适合**: 所有开发者，了解 v1.0 到 v2.0 的变化

6. **[PlanningContext 设计文档](docs/PLANNING_CONTEXT_DESIGN.md)** ⭐⭐⭐⭐⭐ **NEW**
   - **内容**: PlanningContext 详细设计（300 行）
   - **包含**: 数据结构定义、使用示例（标准数据 + 自定义数据）、配置示例、最佳实践
   - **适合**: 插件开发者，了解如何输出和读取数据

### 配置文件

4. **[默认配置模板](config/default.json.example)** ⭐⭐⭐
   - 完整的默认配置示例
   - 包含所有插件和参数说明
   - **更新**: v2.0 已添加 `perception.preprocessing` 配置节

5. **[配置示例](config/examples/)** ⭐⭐⭐
   - `astar_planner.json` - 使用 A* 规划器的配置
   - `minimal.json` - 最小化配置示例
   - **更新**: v2.0 已更新感知配置结构

6. **[配置文件说明](config/README.md)** ⭐⭐⭐
   - 配置文件使用指南
   - 参数说明和故障排查
   - **更新**: v2.0 已更新感知配置说明和参数表

### 架构图

7. **架构图 (Mermaid)**
   - 插件化架构设计图
   - 插件生命周期流程图

---

## 🎯 重构方案核心要点

### 设计目标

1. ✅ **感知处理插件化** - 用户可自定义感知数据转换
2. ✅ **规划器插件化** - 用户可新增和适配不同规划算法
3. ✅ **配置驱动** - 通过 JSON 配置文件控制插件
4. ✅ **向后兼容** - 现有功能完全保留

### 核心架构

```
Application Layer
    ↓
Algorithm Manager
    ↓
┌─────────────────┴─────────────────┐
↓                                   ↓
Perception Plugin Manager    Planning Plugin Manager
    ↓                                   ↓
Plugin Registry                    Plugin Registry
    ↓                                   ↓
Perception Plugins              Planner Plugins
```

### 插件接口

**感知插件**:
```cpp
class PerceptionPluginInterface {
  virtual Metadata getMetadata() const = 0;
  virtual bool initialize(const nlohmann::json& config) = 0;
  virtual bool process(const proto::WorldTick&, PlanningContext&) = 0;
};
REGISTER_PERCEPTION_PLUGIN(MyPlugin)
```

**规划器插件**:
```cpp
class PlannerPluginInterface {
  virtual Metadata getMetadata() const = 0;
  virtual bool initialize(const nlohmann::json& config) = 0;
  virtual bool plan(const PlanningContext&, deadline, result) = 0;
  virtual std::pair<bool, string> isAvailable(const PlanningContext&) = 0;
};
REGISTER_PLANNER_PLUGIN(MyPlanner)
```

### 配置系统

**格式**: JSON (使用已有的 nlohmann/json)

**示例**:
```json
{
  "perception": {
    "plugins": [
      {"name": "GridBuilderPlugin", "enabled": true, "params": {...}}
    ]
  },
  "planning": {
    "primary_planner": "AStarPlannerPlugin",
    "planners": {
      "AStarPlannerPlugin": {...}
    }
  }
}
```

### 目录结构

```
navsim-local/
├── config/                    # 配置文件 (新增)
├── include/
│   ├── plugin/                # 插件系统核心 (新增)
│   └── config/                # 配置系统 (新增)
├── src/
│   ├── plugin/                # 插件系统实现 (新增)
│   └── config/                # 配置系统实现 (新增)
├── plugins/                   # 插件实现 (新增)
│   ├── perception/
│   ├── planning/
│   └── examples/              # 自定义插件示例
└── docs/
    ├── PLUGIN_ARCHITECTURE_DESIGN.md
    ├── PLUGIN_ARCHITECTURE_SUMMARY.md
    └── PLUGIN_QUICK_REFERENCE.md
```

---

## 📅 实施计划

### Phase 1: 基础架构 (1-2 周)

**目标**: 建立插件系统核心框架

**任务清单**:

#### 1.1 核心数据结构
- [ ] 创建 `include/plugin/perception_input.hpp`
  - [ ] 定义 `PerceptionInput` 结构（包含 ego, task, bev_obstacles, dynamic_obstacles）
- [ ] 创建 `include/planning/planning_context.hpp`
  - [ ] 定义 `PlanningContext` 类（固定字段 + custom_data）
  - [ ] 实现模板函数 `setCustomData<T>()` 和 `getCustomData<T>()`
  - [ ] 实现 `hasCustomData()` 方法
- [ ] 创建 `include/planning/planning_result.hpp`
  - [ ] 定义 `PlanningResult` 结构

#### 1.2 插件接口定义
- [ ] 创建 `include/plugin/perception_plugin_interface.hpp`
  - [ ] 定义 `PerceptionPluginInterface` 抽象类
  - [ ] 定义 `Metadata` 结构
  - [ ] 接口方法：`getMetadata()`, `initialize()`, `process()`, `reset()`
- [ ] 创建 `include/plugin/planner_plugin_interface.hpp`
  - [ ] 定义 `PlannerPluginInterface` 抽象类
  - [ ] 接口方法：`getMetadata()`, `initialize()`, `plan()`, `isAvailable()`, `reset()`

#### 1.3 插件注册机制
- [ ] 创建 `include/plugin/plugin_registry.hpp`
  - [ ] 实现 `PerceptionPluginRegistry` 单例类
  - [ ] 实现 `PlannerPluginRegistry` 单例类
  - [ ] 定义注册宏 `REGISTER_PERCEPTION_PLUGIN()` 和 `REGISTER_PLANNER_PLUGIN()`
- [ ] 创建 `src/plugin/plugin_registry.cpp`
  - [ ] 实现注册表的工厂方法

#### 1.4 插件管理器
- [ ] 创建 `include/plugin/perception_plugin_manager.hpp`
  - [ ] 定义 `PerceptionPluginManager` 类
  - [ ] 方法：`loadPlugins()`, `initialize()`, `process()`, `reset()`
- [ ] 创建 `src/plugin/perception_plugin_manager.cpp`
  - [ ] 实现插件加载和执行逻辑（按优先级排序）
- [ ] 创建 `include/plugin/planner_plugin_manager.hpp`
  - [ ] 定义 `PlannerPluginManager` 类
  - [ ] 方法：`loadPlanners()`, `initialize()`, `plan()`, `reset()`
  - [ ] 实现降级机制（primary → fallback）
- [ ] 创建 `src/plugin/planner_plugin_manager.cpp`
  - [ ] 实现规划器选择和降级逻辑

#### 1.5 前置处理层 (v2.0 新增)
- [ ] 创建 `include/perception/preprocessing.hpp`
  - [ ] 定义 `BEVExtractor` 类
  - [ ] 定义 `DynamicObstaclePredictor` 类
  - [ ] 定义 `BasicDataConverter` 类
- [ ] 创建 `src/perception/bev_extractor.cpp`
  - [ ] 实现 BEV 障碍物提取逻辑
- [ ] 创建 `src/perception/dynamic_predictor.cpp`
  - [ ] 实现动态障碍物预测逻辑
- [ ] 创建 `src/perception/basic_converter.cpp`
  - [ ] 实现基础数据转换逻辑

#### 1.6 配置系统
- [ ] 创建 `include/plugin/config_loader.hpp`
  - [ ] 定义 `ConfigLoader` 类
  - [ ] 方法：`load()`, `validate()`, `getPerceptionConfig()`, `getPlanningConfig()`
- [ ] 创建 `src/plugin/config_loader.cpp`
  - [ ] 实现 JSON 配置文件加载
  - [ ] 实现配置优先级处理（命令行 > 环境变量 > 文件 > 默认值）
  - [ ] 实现配置验证

#### 1.7 单元测试
- [ ] 创建 `tests/test_perception_input.cpp`
- [ ] 创建 `tests/test_planning_context.cpp`
  - [ ] 测试 `setCustomData()` 和 `getCustomData()`
  - [ ] 测试类型安全
- [ ] 创建 `tests/test_plugin_registry.cpp`
- [ ] 创建 `tests/test_config_loader.cpp`

**交付物**:
- ✅ 插件系统核心代码（include/plugin/, src/plugin/）
- ✅ 前置处理层代码（include/perception/, src/perception/）
- ✅ 配置系统代码
- ✅ 单元测试（覆盖率 > 80%）
- ✅ CMakeLists.txt 更新

**验收标准**:
- 所有单元测试通过
- 代码符合 C++17 标准
- 无编译警告

---

### Phase 2: 插件迁移 (1-2 周)

**目标**: 将现有功能迁移为插件

**任务清单**:

#### 2.1 感知插件实现
- [ ] 创建 `plugins/perception/grid_map_builder_plugin.hpp`
  - [ ] 实现 `GridMapBuilderPlugin` 类
  - [ ] 从 `PerceptionInput` 构建栅格地图
  - [ ] 输出到 `context.occupancy_grid`
- [ ] 创建 `plugins/perception/grid_map_builder_plugin.cpp`
  - [ ] 实现地图构建逻辑
  - [ ] 注册插件：`REGISTER_PERCEPTION_PLUGIN(GridMapBuilderPlugin)`
- [ ] 创建 `plugins/perception/esdf_builder_plugin.hpp`
  - [ ] 实现 `ESDFBuilderPlugin` 类
  - [ ] 从 `PerceptionInput` 构建 ESDF 距离场
  - [ ] 输出到 `context.setCustomData("esdf_map", esdf)`
- [ ] 创建 `plugins/perception/esdf_builder_plugin.cpp`
  - [ ] 实现 ESDF 计算逻辑
  - [ ] 注册插件

#### 2.2 规划器插件实现
- [ ] 创建 `plugins/planning/straight_line_planner_plugin.hpp`
  - [ ] 实现 `StraightLinePlannerPlugin` 类
  - [ ] 几何规划器（降级方案）
- [ ] 创建 `plugins/planning/straight_line_planner_plugin.cpp`
  - [ ] 实现直线规划逻辑
  - [ ] 注册插件：`REGISTER_PLANNER_PLUGIN(StraightLinePlannerPlugin)`
- [ ] 创建 `plugins/planning/astar_planner_plugin.hpp`
  - [ ] 实现 `AStarPlannerPlugin` 类
  - [ ] 需要 `occupancy_grid`
- [ ] 创建 `plugins/planning/astar_planner_plugin.cpp`
  - [ ] 实现 A* 搜索逻辑
  - [ ] 注册插件
- [ ] 创建 `plugins/planning/optimization_planner_plugin.hpp`
  - [ ] 实现 `OptimizationPlannerPlugin` 类
  - [ ] 优化规划器
- [ ] 创建 `plugins/planning/optimization_planner_plugin.cpp`
  - [ ] 实现优化逻辑
  - [ ] 注册插件

#### 2.3 适配 AlgorithmManager
- [ ] 修改 `include/algorithm_manager.hpp`
  - [ ] 添加 `PerceptionPluginManager` 成员
  - [ ] 添加 `PlannerPluginManager` 成员
  - [ ] 添加前置处理模块成员
- [ ] 修改 `src/algorithm_manager.cpp`
  - [ ] 在 `initialize()` 中加载插件
  - [ ] 在 `processWorldTick()` 中调用前置处理层
  - [ ] 在 `processWorldTick()` 中调用感知插件
  - [ ] 在 `processWorldTick()` 中调用规划器插件

#### 2.4 配置文件
- [ ] 创建 `config/default.json`（从 default.json.example 复制）
  - [ ] 包含 `perception.preprocessing` 配置
  - [ ] 包含感知插件配置
  - [ ] 包含规划器配置

#### 2.5 CMake 配置
- [ ] 创建 `plugins/perception/CMakeLists.txt`
- [ ] 创建 `plugins/planning/CMakeLists.txt`
- [ ] 更新主 `CMakeLists.txt`
  - [ ] 添加插件系统库
  - [ ] 添加前置处理库
  - [ ] 链接所有插件

#### 2.6 集成测试
- [ ] 创建 `tests/test_perception_plugins.cpp`
  - [ ] 测试 GridMapBuilderPlugin
  - [ ] 测试 ESDFBuilderPlugin
- [ ] 创建 `tests/test_planner_plugins.cpp`
  - [ ] 测试 StraightLinePlannerPlugin
  - [ ] 测试 AStarPlannerPlugin
- [ ] 创建 `tests/test_algorithm_manager_integration.cpp`
  - [ ] 端到端测试

**交付物**:
- ✅ 所有插件实现（perception + planning）
- ✅ 更新的 AlgorithmManager
- ✅ 默认配置文件
- ✅ 集成测试

**验收标准**:
- 所有插件正常工作
- 集成测试通过
- 性能无明显退化（< 5%）

---

### Phase 3: 测试与文档 (1 周)

**目标**: 完善测试和文档

**任务清单**:

#### 3.1 测试
- [ ] 端到端测试
  - [ ] 测试默认配置运行
  - [ ] 测试自定义配置运行
  - [ ] 测试插件切换
- [ ] 性能测试
  - [ ] 基准测试（重构前 vs 重构后）
  - [ ] 性能分析（感知、规划、总体）
- [ ] 向后兼容性测试
  - [ ] 无配置文件运行
  - [ ] 与旧版本行为对比

#### 3.2 文档
- [ ] 验证所有设计文档与实现一致
  - [ ] PLUGIN_SYSTEM_README.md
  - [ ] PLUGIN_ARCHITECTURE_DESIGN.md
  - [ ] PERCEPTION_PLUGIN_ARCHITECTURE.md
  - [ ] PLANNING_CONTEXT_DESIGN.md
- [ ] 创建示例插件
  - [ ] `plugins/examples/custom_perception_plugin.cpp`
  - [ ] `plugins/examples/custom_planner_plugin.cpp`
- [ ] 更新主 README.md
  - [ ] 添加插件系统说明
  - [ ] 添加快速开始指南

#### 3.3 代码审查
- [ ] 代码风格检查
- [ ] 代码审查（peer review）
- [ ] 性能优化

**交付物**:
- ✅ 完整测试套件（单元测试 + 集成测试 + 性能测试）
- ✅ 示例插件
- ✅ 更新的文档

**验收标准**:
- 所有测试通过
- 测试覆盖率 > 80%
- 性能退化 < 5%
- 文档完整

---

### Phase 4: 高级功能 (可选, 1-2 周)

**任务**:
- [ ] 运行时动态加载插件（.so/.dll）
- [ ] 插件热重载
- [ ] 插件依赖管理
- [ ] 配置文件 Schema 验证（JSON Schema）
- [ ] 插件性能监控和统计

---

## ✅ 成功标准

- ✅ 所有现有功能正常工作
- ✅ 性能无明显退化 (< 5%)
- ✅ 可以通过配置文件切换插件
- ✅ 可以添加自定义插件无需修改核心代码
- ✅ 文档完整，示例清晰
- ✅ 所有测试通过

---

## 🚀 快速开始

### 对于架构师/技术负责人

1. 阅读 **[插件架构设计](docs/PLUGIN_ARCHITECTURE_DESIGN.md)**
2. 评审设计方案
3. 提供反馈和建议

### 对于项目经理

1. 阅读 **[执行摘要](docs/PLUGIN_ARCHITECTURE_SUMMARY.md)**
2. 了解实施计划和时间表
3. 评估资源需求

### 对于开发者

1. 阅读 **[快速参考](docs/PLUGIN_QUICK_REFERENCE.md)**
2. 查看配置文件示例
3. 准备开始实施

---

## 📊 优势总结

| 优势 | 说明 |
|------|------|
| **可扩展性** | 用户可轻松添加自定义插件，无需修改核心代码 |
| **可配置性** | 通过 JSON 配置文件灵活控制行为 |
| **可维护性** | 模块化设计，职责清晰，易于维护 |
| **可测试性** | 插件独立测试，易于调试 |
| **向后兼容** | 现有功能完全保留，平滑迁移 |

---

## 🔄 向后兼容性

### 无感知迁移

用户无需任何修改，程序使用默认配置运行：

```bash
# 重构前
./build/navsim_algo ws://127.0.0.1:8080/ws demo

# 重构后 (完全相同)
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

### 渐进式迁移

用户可选择使用配置文件：

```bash
# 使用配置文件
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config.json
```

---

## 🛠️ 如何添加新插件

### 3 步添加自定义插件

1. **实现插件类**
   ```cpp
   class MyPlugin : public PerceptionPluginInterface {
     // 实现接口...
   };
   REGISTER_PERCEPTION_PLUGIN(MyPlugin)
   ```

2. **添加到 CMake**
   ```cmake
   add_library(navsim_perception_plugins STATIC
       ...
       my_plugin.cpp  # 新增
   )
   ```

3. **配置文件中启用**
   ```json
   {
     "perception": {
       "plugins": [
         {"name": "MyPlugin", "enabled": true, "params": {...}}
       ]
     }
   }
   ```

---

## 📞 下一步行动

1. **评审设计方案** - 收集团队反馈
2. **创建 GitHub Issue/Project** - 跟踪实施进度
3. **开始 Phase 1 实施** - 建立基础架构
4. **持续迭代** - 根据实际使用情况优化

---

## 📚 相关资源

### 内部文档

- [插件架构设计](docs/PLUGIN_ARCHITECTURE_DESIGN.md) - 完整设计文档
- [执行摘要](docs/PLUGIN_ARCHITECTURE_SUMMARY.md) - 核心要点
- [快速参考](docs/PLUGIN_QUICK_REFERENCE.md) - 开发速查
- [配置说明](config/README.md) - 配置文件指南

### 待创建文档

- `PLUGIN_DEVELOPMENT_GUIDE.md` - 插件开发详细指南
- `MIGRATION_GUIDE.md` - 从旧版本迁移指南
- `API_REFERENCE.md` - API 参考文档

### 外部参考

- [C++ Plugin Architecture](https://www.codeproject.com/Articles/1191047/Plugin-Architecture-in-Cplusplus)
- [nlohmann/json Documentation](https://json.nlohmann.me/)
- [Factory Pattern in C++](https://refactoring.guru/design-patterns/factory-method/cpp/example)

---

## 📧 联系方式

如有问题或建议，请联系：

- **GitHub Issues**: [ahrs-simulator/issues](https://github.com/ahrs365/ahrs-simulator/issues)
- **Email**: ahrs365@outlook.com

---

**文档创建日期**: 2025-10-13  
**方案状态**: ✅ 设计完成，待评审  
**下一步**: 团队评审 → 开始实施



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

**任务**:
- [ ] 定义插件接口
- [ ] 实现插件注册表
- [ ] 实现插件管理器
- [ ] 实现配置加载器
- [ ] 调整目录结构
- [ ] 编写单元测试

**交付物**:
- 插件系统核心代码
- 配置系统代码
- 单元测试
- 基础文档

### Phase 2: 插件迁移 (1-2 周)

**任务**:
- [ ] 迁移感知处理器为插件
  - [ ] GridBuilderPlugin
  - [ ] BEVExtractorPlugin
  - [ ] DynamicPredictorPlugin
- [ ] 迁移规划器为插件
  - [ ] StraightLinePlannerPlugin
  - [ ] AStarPlannerPlugin
  - [ ] OptimizationPlannerPlugin
- [ ] 适配 AlgorithmManager
- [ ] 修改 main.cpp

**交付物**:
- 所有插件实现
- 更新的 AlgorithmManager
- 默认配置文件
- 集成测试

### Phase 3: 测试与文档 (1 周)

**任务**:
- [ ] 端到端测试
- [ ] 性能测试
- [ ] 向后兼容性测试
- [ ] 编写插件开发指南
- [ ] 编写迁移指南
- [ ] 更新 README

**交付物**:
- 完整测试套件
- 插件开发指南
- 迁移指南
- 示例插件

### Phase 4: 高级功能 (可选, 1-2 周)

**任务**:
- [ ] 运行时动态加载插件
- [ ] 插件热重载
- [ ] 插件依赖管理
- [ ] 配置文件 Schema 验证

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



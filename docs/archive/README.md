# 归档文档说明

本目录包含已归档的历史文档。这些文档记录了项目开发过程中的分析、验证和改进过程，具有历史参考价值，但不再作为主要文档维护。

## 📁 归档文档列表

### 架构分析和设计

| 文档 | 描述 | 归档原因 |
|------|------|----------|
| `ARCHITECTURE_ANALYSIS.md` | 插件架构分析：现有设计 vs 提议的新架构 | 历史分析文档，结论已整合到 `ARCHITECTURE.md` |
| `PLUGIN_LOADING_MECHANISM.md` | 插件加载和注册机制详解 | 内容已整合到 `ARCHITECTURE.md` |
| `PLUGIN_SYSTEM_GUIDE.md` | 插件系统说明书 | 内容已整合到 `PLUGIN_DEVELOPMENT.md` 和 `ARCHITECTURE.md` |

### 插件开发指南

| 文档 | 描述 | 归档原因 |
|------|------|----------|
| `PLUGIN_DEVELOPMENT_GUIDE.md` | 旧版插件开发指南 | 已被新版 `PLUGIN_DEVELOPMENT.md` 替代 |
| `LOCAL_DEBUG_MODE.md` | 旧版本地调试模式文档 | 已被 `GETTING_STARTED.md` 替代 |

### 脚手架工具验证

| 文档 | 描述 | 归档原因 |
|------|------|----------|
| `PLUGIN_SCAFFOLDING_IMPROVEMENTS.md` | 插件脚手架工具改进建议 | 改进已完成，历史记录 |
| `PLUGIN_SCAFFOLDING_VALIDATION.md` | 插件脚手架工具验证报告（JPS Planner） | 验证已完成，历史记录 |
| `PLUGIN_SCAFFOLDING_VALIDATION_FAILURE.md` | 脚手架工具验证失败报告 | 问题已修复，历史记录 |
| `PLUGIN_SCAFFOLDING_VALIDATION_SUCCESS.md` | 脚手架工具验证成功报告 | 验证已完成，历史记录 |

### 其他

| 文档 | 描述 | 归档原因 |
|------|------|----------|
| `AVAILABLE_PLUGINS.md` | 可用插件列表 | 内容已整合到主 `README.md` |

## 📚 当前活跃文档

如果您正在查找文档，请参考以下当前维护的文档：

### 核心文档（位于 `docs/` 目录）

1. **[README.md](../README.md)** - 项目概述和快速开始
2. **[GETTING_STARTED.md](../GETTING_STARTED.md)** - 详细的快速开始指南
3. **[PLUGIN_DEVELOPMENT.md](../PLUGIN_DEVELOPMENT.md)** - 插件开发完整指南
4. **[DEVELOPMENT_TOOLS.md](../DEVELOPMENT_TOOLS.md)** - 开发工具使用指南
5. **[ARCHITECTURE.md](../ARCHITECTURE.md)** - 架构设计文档

### 文档导航

- **新用户**：从 [README.md](../README.md) 和 [GETTING_STARTED.md](../GETTING_STARTED.md) 开始
- **插件开发者**：参考 [PLUGIN_DEVELOPMENT.md](../PLUGIN_DEVELOPMENT.md)
- **使用开发工具**：参考 [DEVELOPMENT_TOOLS.md](../DEVELOPMENT_TOOLS.md)
- **了解架构**：参考 [ARCHITECTURE.md](../ARCHITECTURE.md)

## 🗂️ 归档策略

文档被归档的原因通常包括：

1. **内容已过时**：与当前代码实现不一致
2. **内容已整合**：精华内容已整合到新文档中
3. **历史记录**：记录开发过程，但不再需要日常参考
4. **重复内容**：与其他文档重复

## 📝 归档日期

本批次文档归档于：**2025-10-18**

## ⚠️ 注意事项

- 归档文档**不再更新**，可能包含过时信息
- 归档文档仅供**历史参考**
- 如需最新信息，请参考当前活跃文档
- 归档文档中的代码示例可能无法直接运行

## 🔍 查找信息

如果您在归档文档中找到有用的信息，但在当前文档中找不到：

1. 检查是否已整合到其他文档
2. 查看 Git 提交历史了解变更原因
3. 如有疑问，请提交 Issue

## 📧 反馈

如果您认为某个归档文档应该恢复或更新，请：

1. 提交 Issue 说明原因
2. 提供具体的使用场景
3. 建议如何改进当前文档

---

**维护者注意**：归档文档应定期审查，确保不包含敏感信息或误导性内容。


# NavSim Local 文档目录

本目录包含 NavSim 本地算法工程的技术文档。

---

## 📚 文档列表

### 1. [WebSocket 集成方案](./websocket-integration-plan.md)

**完整的实施方案和开发计划**

- 技术方案概述（架构、技术选型）
- 详细实现方案（消息格式、Bridge 设计、线程模型）
- 5 个开发阶段（Phase 1-5）
- 关键技术细节（消息过滤、延迟补偿、错误处理）
- 验收标准和时间估算

**适合**：开发人员详细阅读，按阶段实施

---

### 2. [修订总结](./REVISION_SUMMARY.md)

**方案修订说明和关键变更**

- 8 个已修正的问题（Topic 命名、字段映射、延迟补偿等）
- 修订前后对照表
- 实施检查清单
- 保留的亮点

**适合**：快速了解方案变更，确认关键修正点

---

## 🎯 快速开始

### 如果您是第一次阅读：

1. **先看** [修订总结](./REVISION_SUMMARY.md) - 了解关键修正点（5 分钟）
2. **再看** [完整方案](./websocket-integration-plan.md) - 理解详细实施计划（20 分钟）
3. **确认** 待确认问题（方案第十章）
4. **开始** Phase 1 实施

### 如果您已经熟悉项目：

直接查看 [完整方案](./websocket-integration-plan.md) 的以下章节：

- **零、关键修订说明** - 与初版的重要差异
- **二、详细实现方案** - 消息格式映射和 Bridge 设计
- **三、开发计划** - Phase 1-5 任务清单
- **附录 A** - 关键修订对照表

---

## ✅ 关键修正点速查

| 项目 | 正确做法 |
|------|---------|
| **Topic 格式** | `room/<room_id>/world_tick`（不带前导 `/`） |
| **发送消息** | 只发 `plan`，不发 `ego_cmd` |
| **字段命名** | `theta` 不是 `yaw`，单位：弧度 |
| **速度和曲率** | `v` 单位 m/s，`kappa` 单位 1/m |
| **plan 必须字段** | `x, y, theta, kappa, s, t, v`（7 个字段） |
| **连接 URL** | 命令行传入 `url` 和 `room_id`，Bridge 拼接 `ws://host/ws?room=<id>` |
| **队列策略** | SPSC 容量 8，入队覆盖旧元素，Planner 只消费最新 |
| **延迟补偿** | 收到 tick 时计算 Δ 并前滚，使用标量速度 v |
| **心跳机制** | 每 5s 发送，Topic: `room/<id>/control/heartbeat` |
| **日志级别** | WARN（schema_ver 不匹配、字段缺失）/ ERROR（JSON 解析失败） |
| **JSON 精度** | 使用 std::fixed，避免科学计数法 |
| **wss:// 支持** | ixwebsocket + OpenSSL |

---

## 📋 待确认问题

在开始实施前，请确认：

1. 是否先实现最小可用版本（忽略 obstacles/map）？
2. 命令行参数格式：`./navsim_algo <url> <room_id>` 是否可以？
3. 日志级别需求？
4. SPSC 队列容量 8 是否合适？
5. 心跳发送频率 5 秒是否合适？

---

## 🚀 实施流程

```
Phase 1: CMake 集成 (30 分钟)
   ↓
Phase 2: Bridge 基础框架 (1 小时)
   ↓
Phase 3: JSON ↔ Protobuf 转换 (1.5 小时)
   ↓
Phase 4: main.cpp 集成 (1 小时)
   ↓
Phase 5: 端到端测试与优化 (1 小时)
   ↓
总计: 5 小时
```

---

## 📞 联系方式

如有疑问，请参考：

- [任务书](../../任务书.md)
- [服务端实现](../../navsim-online/server/main.py)
- [前端实现](../../navsim-online/web/index.html)

---

**文档版本**: v1.1  
**最后更新**: 2025-01-XX


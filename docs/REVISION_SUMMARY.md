# WebSocket 集成方案修订总结

> **修订日期**: 2025-01-XX  
> **修订原因**: 根据现有协议和服务端实现调整方案

---

## 📝 修订概述

根据审阅反馈，原方案在以下几个关键点与现有系统不一致，已全部修正：

---

## ✅ 已修正的问题

### 1. Topic 命名规范

**问题**：初版使用了错误的 Topic 格式

| 项目 | 初版（❌ 错误） | 修订版（✅ 正确） |
|------|---------------|-----------------|
| world_tick | `/room/demo/world_tick` | `room/demo/world_tick` |
| plan | `/room/demo/plan_update` | `room/demo/plan` |
| heartbeat | 未定义 | `room/demo/control/heartbeat` |

**修正原因**：
- 服务端 Topic 不带前导 `/`
- 消息名称是 `plan` 而非 `plan_update`
- 心跳需要统一命名规范

---

### 2. 消息发送策略

**问题**：初版计划同时发送 `plan_update` 和 `ego_cmd`

**修正**：
- ✅ 只发送 `plan`（服务端当前只订阅此消息）
- ❌ 暂不发送 `ego_cmd`（服务端未订阅）
- 📝 预留 `ego_cmd` 接口，待服务端支持后再启用

---

### 3. 字段映射修正

**问题**：初版字段命名与现有协议不一致

| 字段类型 | 初版（❌） | 修订版（✅） | 说明 |
|---------|----------|------------|------|
| 角度 | `yaw` | `theta` | 统一使用 theta |
| 速度 | `vx, vy, omega` | `v, kappa` | 标量速度 + 曲率 |
| Schema | `schema` | `schema_ver` | 版本化管理 |
| 状态 | `status` | 移除 | 服务端不需要 |

**转换逻辑**：
```cpp
// JSON → Protobuf
ego.pose.yaw = json["ego"]["theta"];
ego.twist.vx = json["ego"]["v"] * cos(json["ego"]["theta"]);
ego.twist.omega = json["ego"]["v"] * json["ego"]["kappa"];

// Protobuf → JSON
json["points"][i]["theta"] = trajectory[i].yaw;
json["points"][i]["v"] = 0.8;  // 暂时填常数
json["points"][i]["kappa"] = 0.0;  // 暂时填 0
```

---

### 4. 连接 URL 格式

**问题**：初版未明确 URL 与 Topic 的关系

**修正**：
- 连接 URL: `ws://host:port/ws?room=<room_id>`
- Topic 路径: `room/<room_id>/world_tick`
- **关键**：room 参数在 URL query，不在 Topic 路径中重复

**示例**：
```cpp
// 正确
bridge.connect("ws://127.0.0.1:8080/ws", "demo");
// URL: ws://127.0.0.1:8080/ws?room=demo
// Topic: room/demo/world_tick

// 错误
bridge.connect("ws://127.0.0.1:8080/ws/room/demo", "demo");
```

---

### 5. 消息队列优化

**问题**：初版使用单槽覆盖，在 burst 时频繁覆盖

**修正**：
- ❌ 初版：单槽覆盖（容量 1）
- ✅ 修订：SPSC 队列（容量 4-8）
- 📊 统计：`dropped_ticks`（队列满时丢弃旧帧）

**优势**：
- 减少频繁覆盖写
- 应对短时 burst（30→60 Hz 抖动）
- 仍然"取最新"，但更平滑

---

### 6. 延迟补偿机制

**问题**：初版未考虑网络延迟

**新增**：
```cpp
void Bridge::Impl::compensate_delay(proto::WorldTick* tick, double delay_sec) {
  // 计算延迟
  double delay = now() - tick->stamp();
  
  // 预测起点前滚
  double theta = tick->ego().pose().yaw();
  double v = tick->ego().twist().vx();
  
  tick->mutable_ego()->mutable_pose()->set_x(
    tick->ego().pose().x() + v * cos(theta) * delay
  );
  tick->mutable_ego()->mutable_pose()->set_y(
    tick->ego().pose().y() + v * sin(theta) * delay
  );
  
  // 记录延迟
  if (delay > 0.1) {
    std::cerr << "[Bridge] High delay: " << (delay * 1000) << "ms" << std::endl;
  }
}
```

**作用**：
- 补偿网络延迟（典型 20-50ms）
- 提高动态障碍预测精度
- 记录高延迟警告（>100ms）

---

### 7. 心跳机制完善

**问题**：初版心跳内容未定义

**修正**：
```json
{
  "topic": "room/demo/control/heartbeat",
  "data": {
    "schema_ver": "1.0.0",
    "stamp": 1738200000.0,
    "ws_rx": 1234,
    "ws_tx": 567,
    "dropped_ticks": 5,
    "loop_hz": 19.8,
    "compute_ms_p50": 25.3
  }
}
```

**发送频率**：5 秒

---

### 8. 错误兜底策略

**新增**：静止计划发布路径

```cpp
// 规划失败时，发送静止计划
proto::PlanUpdate fallback_plan;
fallback_plan.set_tick_id(latest_tick_id);
fallback_plan.set_stamp(current_time());

auto* pt = fallback_plan.add_trajectory();
pt->set_x(ego_x);
pt->set_y(ego_y);
pt->set_yaw(ego_theta);
pt->set_t(0.0);

bridge.publish(fallback_plan);  // v=0，至少 1 个点
```

**作用**：
- 保证始终有计划输出
- 前端能对齐 tick_id
- 避免"无响应"状态

---

## 🎯 保留的亮点

以下设计在初版中已经正确，保持不变：

1. ✅ **技术选型**：ixwebsocket + nlohmann/json
2. ✅ **线程模型**：WebSocket 线程 + 规划线程
3. ✅ **开发阶段**：Phase 1-5 清晰可执行
4. ✅ **时间估算**：5 小时总计，现实可行
5. ✅ **风险应对**：覆盖 OpenSSL/zlib/字段不匹配等问题

---

## 📋 实施检查清单

在开始实施前，务必确认：

- [ ] **Topic 格式**：`room/<id>/world_tick`（不带前导 `/`）
- [ ] **发送消息**：只发 `plan`，不发 `ego_cmd`
- [ ] **字段命名**：`theta` 不是 `yaw`，单位为弧度
- [ ] **速度和曲率**：`v` 单位 m/s，`kappa` 单位 1/m
- [ ] **plan 必须字段**：`x, y, theta, kappa, s, t, v`（7 个字段）
- [ ] **连接 URL**：命令行传入 `url` 和 `room_id`，Bridge 拼接为 `ws://host/ws?room=<id>`
- [ ] **队列容量**：8（SPSC 队列，入队覆盖旧元素，Planner 只消费最新）
- [ ] **延迟补偿**：已实现，使用标量速度 v
- [ ] **心跳机制**：每 5s 发送，包含 `ws_rx, ws_tx, dropped_ticks, loop_hz, compute_ms_p50`
- [ ] **静止计划**：已实现，包含所有必须字段
- [ ] **日志级别**：WARN/ERROR 区分清晰
- [ ] **JSON 精度**：使用 std::fixed，避免科学计数法
- [ ] **schema_ver**：在 data 内，缺失或不匹配记录 WARN
- [ ] **断线处理**：publish(plan) 断线时直接丢弃，不阻塞

---

## 🚀 下一步行动

1. **审阅修订后的方案**：`websocket-integration-plan.md`
2. **确认剩余问题**：
   - 是否先实现最小可用版本（忽略 obstacles/map）？
   - 命令行参数格式？
   - 日志级别需求？
3. **开始实施**：Phase 1 → Phase 5

---

## 📚 相关文档

- [完整实施方案](./websocket-integration-plan.md)
- [任务书](../../任务书.md)
- [服务端实现](../../navsim-online/server/main.py)

---

**修订完成，等待确认后开始实施！** 🎉


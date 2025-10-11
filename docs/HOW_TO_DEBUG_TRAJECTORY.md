# 如何调试轨迹不显示问题

## 🎯 问题

**您的问题**: "本地算法发送的轨迹，是先发送到服务器，然后再转发到web客户端吗？我还是没有在页面上看到。这个轨迹到底发给谁了，如何确认接收到了？"

---

## ✅ 答案

### 1. 消息流向

**是的，消息流向是**:

```
本地算法 → 服务器 → 广播 → 所有客户端（包括前端和本地算法自己）
```

**详细流程**:

```
1. 本地算法生成轨迹
   ↓
2. 本地算法调用 plan_to_json()
   ↓
3. 本地算法通过 WebSocket 发送 JSON 到服务器
   ↓
4. 服务器接收消息
   ↓
5. 服务器调用 handle_client_payload()（检查 topic）
   ↓
6. 服务器调用 echo()（广播消息）
   ↓
7. 服务器广播到所有连接的客户端
   ↓
8. 前端接收消息
   ↓
9. 前端调用 interpretMessage()
   ↓
10. 前端检查 topic.endsWith('/plan_update')
    ↓
11. 如果匹配，调用 handlePlanUpdate()
    ↓
12. 前端调用 updateTrajectory()
    ↓
13. 前端显示绿色轨迹线
```

---

## 🔍 快速诊断（3 步）

### 步骤 1: 验证本地算法

**运行验证脚本**:

```bash
cd navsim-local
bash verify_plan_update.sh
```

**预期输出**:
```
=== 验证 plan_update 消息 ===

1. 检查编译状态...
   ✅ 代码已修改为 plan_update
   ✅ 代码使用 trajectory 字段
   ✅ 代码使用 yaw 字段

2. 检查编译时间...
   ✅ 可执行文件是最新的

3. 测试消息格式...
   ✅ 本地算法发送了 plan
   ✅ Topic 包含 plan_update

=== 验证完成 ===
```

**如果有 ❌**: 按照提示修复问题

---

### 步骤 2: 检查前端话题控制台

**操作**:
1. 打开浏览器: http://127.0.0.1:8000/index.html
2. 点击"连接 WebSocket"按钮
3. 查看右侧"话题控制台"
4. 查找包含 `plan_update` 的话题

**预期**:
- ✅ 话题列表中有 `room/demo/plan_update` 或 `/room/demo/plan_update`
- ✅ 点击话题，可以看到消息内容
- ✅ 消息包含 `trajectory` 字段

**如果没有看到**:
- ❌ 前端没有接收到消息
- 继续步骤 3

---

### 步骤 3: 运行浏览器诊断

**操作**:
1. 打开浏览器控制台（F12 → Console）
2. 复制 `docs/BROWSER_DIAGNOSIS.js` 的内容
3. 粘贴到控制台并回车
4. 运行 `diagnoseTrajectory()`

**预期输出**:
```
=== NavSim 轨迹诊断 ===

1. 连接状态
   Connected: ✅
   Socket: ✅

2. 轨迹线对象
   Exists: ✅
   Visible: ✅
   Points: 190

3. 轨迹回放状态
   Active: ✅
   Points: 190

4. 话题日志
   Total topics: 5
   Plan topics: 1
   Plan topics found:
     - room/demo/plan_update: 10 messages

...
```

**根据输出找出问题**

---

## 🛠️ 常见问题和解决方案

### 问题 1: 话题控制台没有 plan_update

**症状**: 话题控制台中没有 `plan_update` 相关话题

**可能原因**:
1. 本地算法未启动
2. 本地算法未连接到服务器
3. 本地算法未发送 plan
4. Topic 名称错误

**解决方案**:

**检查本地算法日志**:
```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

**应该看到**:
```
[Bridge] Connected successfully
[Bridge] Received world_tick #1, delay=0.9ms
[Planner] Computed plan with 190 points in 0.0 ms
[Bridge] Sent plan with 190 points, compute_ms=0.1ms
```

**如果没有 "Sent plan"**:
- 检查 Planner 是否生成了轨迹
- 检查是否连接到服务器

---

### 问题 2: 话题控制台有 plan，但不是 plan_update

**症状**: 话题控制台显示 `room/demo/plan` 而不是 `room/demo/plan_update`

**原因**: 没有重新编译最新代码

**解决方案**:
```bash
cd navsim-local
cmake --build build
```

---

### 问题 3: 话题控制台有 plan_update，但没有轨迹线

**症状**: 话题控制台显示 `room/demo/plan_update`，但 3D 场景中没有绿色轨迹线

**可能原因**:
1. 数据格式不匹配（使用 `points` 而不是 `trajectory`）
2. 点的字段不匹配（使用 `theta` 而不是 `yaw`）
3. 前端渲染问题

**解决方案**:

**在浏览器控制台运行**:
```javascript
diagnoseTrajectory()
```

**查看输出**:
```
4. 话题日志
   Plan topics found:
     - room/demo/plan_update: 10 messages
       Last message: {schema_ver: "1.0.0", trajectory: Array(190), ...}
```

**检查**:
- ✅ 消息包含 `trajectory` 字段（不是 `points`）
- ✅ `trajectory` 是数组
- ✅ 数组有元素

**如果消息包含 `points` 而不是 `trajectory`**:
- 代码没有正确修改
- 重新检查 `src/bridge.cpp` 中的 `plan_to_json()` 函数

---

### 问题 4: 数据格式正确，但仍然没有轨迹线

**症状**: 消息格式正确，但 `state.trajectoryLine.visible` 是 `false`

**解决方案**:

**在浏览器控制台运行**:
```javascript
testTrajectoryRendering()
```

**这会创建一个测试轨迹并尝试渲染**

**如果测试轨迹可见**:
- 说明渲染功能正常
- 问题在于实际数据

**如果测试轨迹也不可见**:
- 可能是渲染问题
- 尝试刷新页面

---

## 📊 完整的调试流程图

```
开始
  ↓
运行 verify_plan_update.sh
  ↓
所有检查通过？
  ├─ 否 → 修复问题 → 重新编译
  └─ 是 ↓
启动完整系统
  ↓
查看话题控制台
  ↓
有 plan_update 话题？
  ├─ 否 → 检查本地算法日志
  └─ 是 ↓
点击话题查看内容
  ↓
包含 trajectory 字段？
  ├─ 否 → 代码未正确修改
  └─ 是 ↓
运行 diagnoseTrajectory()
  ↓
trajectoryLine.visible = true？
  ├─ 否 → 运行 testTrajectoryRendering()
  └─ 是 ↓
检查 3D 场景
  ↓
看到绿色轨迹线？
  ├─ 否 → 调整相机角度/缩放
  └─ 是 ↓
成功！✅
```

---

## 🎯 最快的诊断方法

### 方法 1: 一键诊断（推荐）

```bash
# 终端
cd navsim-local
bash verify_plan_update.sh
```

```javascript
// 浏览器控制台
diagnoseTrajectory()
```

### 方法 2: 手动检查

**检查清单**:

- [ ] 本地算法已重新编译
- [ ] 本地算法日志显示 "Sent plan"
- [ ] 前端已连接（右上角绿色）
- [ ] 话题控制台有 `plan_update` 话题
- [ ] 消息包含 `trajectory` 字段
- [ ] `trajectory` 是数组且有元素
- [ ] 数组元素包含 `yaw` 字段（不是 `theta`）
- [ ] `state.trajectoryLine.visible` 是 `true`
- [ ] 3D 场景中可以看到绿色曲线

---

## 📝 总结

### 消息流向

```
本地算法 → 服务器 → 广播 → 前端
```

### 如何确认接收到了

**本地算法端**:
- 查看日志: `[Bridge] Sent plan with X points`

**服务器端**:
- 添加调试输出（见 `DEBUG_TRAJECTORY_NOT_SHOWING.md`）

**前端端**:
- 查看话题控制台
- 运行 `diagnoseTrajectory()`
- 查看浏览器网络面板（F12 → Network → WS）

### 最可能的问题

1. **没有重新编译** - 运行 `cmake --build build`
2. **Topic 名称错误** - 应该是 `plan_update` 不是 `plan`
3. **数据字段错误** - 应该是 `trajectory` 不是 `points`
4. **点字段错误** - 应该是 `yaw` 不是 `theta`
5. **前端未连接** - 点击"连接 WebSocket"按钮

---

## 🚀 下一步

1. **运行验证脚本**: `bash verify_plan_update.sh`
2. **启动系统**: 服务器 + 本地算法 + 前端
3. **运行诊断**: 浏览器控制台中运行 `diagnoseTrajectory()`
4. **查看结果**: 根据输出找出问题

---

**如果仍然有问题，请提供**:
1. `verify_plan_update.sh` 的输出
2. 本地算法的日志
3. `diagnoseTrajectory()` 的输出
4. 话题控制台的截图

---

**文档结束**


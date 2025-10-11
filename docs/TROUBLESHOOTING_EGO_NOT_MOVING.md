# 故障排查：自车不移动

## 🔍 问题描述

**症状**: 自车模型没有实时显示/移动，停留在原点

---

## ✅ 解决方案

### 最常见原因：仿真未启动

**问题**: 仿真默认是**暂停**状态，需要手动启动

**解决方法**:

1. **打开前端**: `http://127.0.0.1:8000/index.html`
2. **点击 "连接 WebSocket"** 按钮
3. **点击 "Start"** 按钮 ⭐ **这是关键步骤！**

**验证**: 
- 前端应该显示 "仿真运行中"
- 话题控制台应该持续收到 `world_tick`
- 自车应该开始移动

---

## 🔧 完整检查清单

### 1. 检查服务器是否运行

```bash
curl http://127.0.0.1:8080
```

**预期**: 返回 `{"detail":"Not Found"}` （正常，说明服务器在运行）

**如果失败**: 
```bash
cd navsim-online
bash run_navsim.sh
```

---

### 2. 检查本地算法是否运行

```bash
ps aux | grep navsim_algo
```

**预期**: 看到 `./build/navsim_algo ws://127.0.0.1:8080/ws demo`

**如果失败**:
```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

---

### 3. 检查前端是否连接

**打开浏览器**: `http://127.0.0.1:8000/index.html`

**检查**:
- [ ] 连接状态显示 "已连接"
- [ ] 话题控制台有消息
- [ ] 3D 场景显示自车（蓝色箭头）

**如果未连接**:
- 点击 "连接 WebSocket" 按钮
- 检查 URL 是否正确: `ws://127.0.0.1:8080/ws`
- 检查 Room ID 是否正确: `demo`

---

### 4. ⭐ 检查仿真是否启动（最重要！）

**症状**: 连接成功，但自车不移动

**原因**: 仿真默认是暂停状态

**解决**: 
1. 在前端界面找到 "Start" 按钮
2. 点击 "Start" 按钮
3. 观察自车是否开始移动

**验证**:
- 话题控制台的 `world_tick` 消息应该持续更新
- `tick_id` 应该递增
- `ego.pose.x` 和 `ego.pose.y` 应该变化

---

### 5. 检查服务器是否收到轨迹

**查看服务器终端**，应该看到：

```
[Room demo] Received trajectory with 50 points, duration: 5.00s
[Room demo] Tracking: tick=20, ego=(0.50, 0.00, 0.00), v=1.00, elapsed=0.50s
[Room demo] Tracking: tick=40, ego=(1.00, 0.00, 0.00), v=1.00, elapsed=1.00s
```

**如果没有**:
- 检查本地算法是否发送了 `plan_update`
- 查看本地算法终端，应该看到 `Sent plan with N points`

---

### 6. 检查前端是否更新自车位置

**打开浏览器控制台** (F12)，应该看到：

```
[Ego Update] tick=20, ego=(0.50, 0.00, 0.00)
[Ego Update] tick=40, ego=(1.00, 0.00, 0.00)
[Ego Update] tick=60, ego=(1.50, 0.00, 0.00)
```

**如果没有**:
- 刷新页面 (Ctrl+Shift+R 强制刷新)
- 清除浏览器缓存
- 检查是否有 JavaScript 错误

---

## 🐛 调试步骤

### 步骤 1: 运行调试脚本

```bash
cd navsim-local
bash debug_closed_loop.sh
```

这会运行 5 秒测试并检查各个环节。

---

### 步骤 2: 检查服务器日志

**查看服务器终端**，寻找以下信息：

```
✅ 正常日志:
  INFO:     127.0.0.1:XXXXX - "WebSocket /ws" [accepted]
  [Room demo] Received trajectory with 50 points, duration: 5.00s
  [Room demo] Tracking: tick=20, ego=(0.50, 0.00, 0.00), v=1.00

❌ 异常日志:
  ERROR: ...
  WARNING: ...
```

---

### 步骤 3: 检查浏览器控制台

**打开浏览器控制台** (F12)，寻找：

```
✅ 正常日志:
  [Ego Update] tick=20, ego=(0.50, 0.00, 0.00)
  WebSocket connected

❌ 异常日志:
  WebSocket connection failed
  Uncaught TypeError: ...
```

---

### 步骤 4: 检查话题控制台

**在前端界面**:

1. 点击 "话题控制台" 标签
2. 点击 `/room/demo/world_tick`
3. 查看最新消息

**检查**:
- `tick_id` 是否递增？
- `ego.pose.x` 和 `ego.pose.y` 是否变化？
- `ego.twist.vx` 是否 > 0？

**示例**:
```json
{
  "tick_id": 123,
  "ego": {
    "pose": {"x": 1.5, "y": 0.0, "yaw": 0.0},  // ✅ x 在变化
    "twist": {"vx": 1.0, "vy": 0.0, "omega": 0.0}  // ✅ vx > 0
  }
}
```

---

## 🎯 常见问题

### Q1: 自车显示但不移动

**原因**: 仿真未启动

**解决**: 点击前端的 "Start" 按钮

---

### Q2: 自车移动但很慢

**原因**: 轨迹速度较低

**检查**: 
- 查看 `ego.twist.vx` 的值
- 查看轨迹点之间的距离

**调整**: 修改本地算法的轨迹生成逻辑

---

### Q3: 自车抖动

**原因**: 前端回放与服务器冲突

**解决**: 
- 确保前端代码已更新
- 强制刷新浏览器 (Ctrl+Shift+R)
- 清除浏览器缓存

---

### Q4: 自车突然停止

**原因**: 轨迹完成

**检查**: 
- 服务器日志是否显示 "Tracking complete"
- 轨迹时长是否太短

**解决**: 
- 等待本地算法发送新轨迹
- 增加轨迹时长

---

### Q5: 自车位置不准确

**原因**: 时间同步问题

**检查**:
- 轨迹点的 `t` 字段是否正确
- 时间是否单调递增

**调试**:
```javascript
// 在浏览器控制台
console.log(state.displayTick.ego.pose);
```

---

## 📊 诊断流程图

```
自车不移动
    ↓
服务器运行？
    ├─ 否 → 启动服务器
    └─ 是 ↓
本地算法运行？
    ├─ 否 → 启动本地算法
    └─ 是 ↓
前端已连接？
    ├─ 否 → 点击"连接 WebSocket"
    └─ 是 ↓
仿真已启动？⭐
    ├─ 否 → 点击"Start"按钮
    └─ 是 ↓
收到 plan_update？
    ├─ 否 → 检查本地算法
    └─ 是 ↓
ego.pose 变化？
    ├─ 否 → 检查服务器日志
    └─ 是 ↓
前端显示更新？
    ├─ 否 → 刷新浏览器
    └─ 是 → ✅ 正常！
```

---

## 🔧 快速修复

### 一键重启所有服务

```bash
# 终端 1: 重启服务器
cd navsim-online
pkill -f uvicorn
bash run_navsim.sh

# 终端 2: 重启本地算法
cd navsim-local
pkill -f navsim_algo
./build/navsim_algo ws://127.0.0.1:8080/ws demo

# 浏览器: 强制刷新
# Ctrl+Shift+R
```

---

## 📝 检查清单总结

使用此清单逐项检查：

- [ ] 服务器运行中
- [ ] 本地算法运行中
- [ ] 前端已连接
- [ ] ⭐ **仿真已启动（点击 Start 按钮）**
- [ ] 服务器收到 plan_update
- [ ] 服务器日志显示 "Tracking"
- [ ] ego.pose 在变化
- [ ] 前端控制台显示 "[Ego Update]"
- [ ] 3D 场景中自车移动

---

## 🎉 成功标志

如果一切正常，你应该看到：

✅ **服务器终端**:
```
[Room demo] Received trajectory with 50 points, duration: 5.00s
[Room demo] Tracking: tick=20, ego=(0.50, 0.00, 0.00), v=1.00
[Room demo] Tracking: tick=40, ego=(1.00, 0.00, 0.00), v=1.00
```

✅ **浏览器控制台**:
```
[Ego Update] tick=20, ego=(0.50, 0.00, 0.00)
[Ego Update] tick=40, ego=(1.00, 0.00, 0.00)
```

✅ **3D 场景**:
- 蓝色箭头（自车）沿绿色轨迹线移动
- 自车不停留在原点

---

**更新时间**: 2025-09-30  
**最常见问题**: 忘记点击 "Start" 按钮启动仿真


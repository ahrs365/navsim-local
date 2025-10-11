# NavSim 快速开始指南

## 🚀 5 分钟快速上手

### 步骤 1: 启动服务器（终端 1）

```bash
cd navsim-online
bash run_navsim.sh
```

**预期输出**:
```
[run_navsim] 启动 WebSocket 服务: http://0.0.0.0:8080/ws
INFO:     Uvicorn running on http://0.0.0.0:8080 (Press CTRL+C to quit)
INFO:     Application startup complete.
[run_navsim] 启动静态前端: http://127.0.0.1:8000/index.html
```

✅ 服务器已启动！

---

### 步骤 2: 打开前端页面

**浏览器访问**: http://127.0.0.1:8000/index.html

**你会看到**:
- 左侧：控制面板
- 中间：Three.js 3D 场景（灰色地面）
- 右侧：话题控制台
- 顶部：连接状态（红色"未连接"）

**此时场景是静止的**，因为还没有连接到 WebSocket。

---

### 步骤 3: 点击"连接 WebSocket"按钮

**位置**: 左侧面板顶部

**默认配置**:
- WebSocket URL: `ws://127.0.0.1:8080/ws`
- Room ID: `demo`

**点击"连接"按钮**

**预期变化**:
1. 顶部状态变为绿色"已连接"
2. 场景中出现：
   - 🚗 蓝色小车（自车）
   - 🎯 红色球体（目标点）
   - 📦 黄色方块（障碍物，如果有）
3. 小车开始移动！
4. 右侧控制台开始显示消息

✅ 前端已连接！

---

### 步骤 4: 启动本地算法（终端 2）

```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

**预期输出**:
```
=== NavSim Local Algorithm ===
WebSocket URL: ws://127.0.0.1:8080/ws
Room ID: demo
===============================
[Bridge] Connecting to ws://127.0.0.1:8080/ws?room=demo
[Bridge] WebSocket connection opened
[Bridge] Connected successfully
[Bridge] Started, waiting for world_tick messages...
[Main] Waiting for world_tick messages... (Press Ctrl+C to exit)
[Bridge] Received world_tick #123, delay=8.5ms
[Planner] Computed plan with 190 points in 0.1 ms
[Bridge] Sent plan with 190 points, compute_ms=0.1ms
```

**前端变化**:
- 场景中出现 **绿色曲线**（规划轨迹）
- 小车沿着绿色曲线移动
- 右侧控制台显示 `plan` 消息

✅ 本地算法已连接！

---

## 🎮 交互操作

### 设置起点

1. 点击左侧"放置起点"按钮
2. 在 3D 场景中点击一个位置
3. 拖动鼠标确定朝向（yaw）
4. 松开鼠标
5. 小车会瞬移到新起点

### 设置终点

1. 点击左侧"放置终点"按钮
2. 在 3D 场景中点击一个位置
3. 拖动鼠标确定朝向
4. 松开鼠标
5. 目标点移动到新位置
6. 规划轨迹会更新

### 放置障碍物

1. 勾选障碍类型（静态圆形/矩形/动态圆形/矩形）
2. 点击"放置"按钮
3. 在场景中点击确定中心点
4. 拖动确定尺寸/朝向
5. 松开鼠标
6. 障碍物出现在场景中

### 查看消息

1. 右侧控制台显示所有话题
2. 点击话题名称查看详情
3. 可以过滤话题（输入框）
4. 可以清空控制台

---

## 📊 观察指标

### 连接状态（顶部）

- 🟢 **已连接**: WebSocket 连接正常
- 🔴 **未连接**: WebSocket 未连接或断开

### 契约检查（顶部）

- 🟢 **契约正常**: world_tick 格式正确
- 🟡 **等待数据**: 尚未接收 world_tick
- 🔴 **契约异常**: world_tick 格式错误

### 性能图表（中间下方）

- 蓝线：自车速度（m/s）
- 红线：目标偏差（m）
- 实时更新（20Hz）

### 话题控制台（右侧）

**常见话题**:
- `/room/demo/world_tick` - 世界状态（20Hz）
- `/room/demo/plan` - 规划轨迹（本地算法发送）
- `/room/demo/control/heartbeat` - 心跳（每 5 秒）
- `/room/demo/control/start` - 起点设置
- `/room/demo/control/goal` - 终点设置

---

## 🔍 故障排查

### 问题 1: 前端无法连接

**症状**: 点击"连接"后，状态仍为"未连接"

**检查**:
1. 服务器是否运行？`curl http://127.0.0.1:8080`
2. URL 是否正确？`ws://127.0.0.1:8080/ws`
3. 浏览器控制台是否有错误？（F12）

### 问题 2: 场景中没有小车

**症状**: 连接成功，但场景是空的

**检查**:
1. 右侧控制台是否收到 `world_tick`？
2. 浏览器控制台是否有 JavaScript 错误？
3. 尝试刷新页面（F5）

### 问题 3: 没有绿色轨迹

**症状**: 小车在移动，但没有绿色轨迹

**原因**: 本地算法未启动或未连接

**解决**:
1. 启动本地算法：`./build/navsim_algo ws://127.0.0.1:8080/ws demo`
2. 检查本地算法日志是否有错误
3. 检查右侧控制台是否收到 `plan` 消息

### 问题 4: 本地算法连接失败

**症状**: `[Bridge] ERROR: WebSocket connection failed`

**检查**:
1. 服务器是否运行？
2. URL 是否正确？
3. 防火墙是否阻止连接？

---

## 🎯 完整工作流程

```
1. 启动服务器
   ↓
2. 打开前端页面
   ↓
3. 点击"连接 WebSocket"按钮
   ↓
4. 服务器开始生成 world_tick（20Hz）
   ↓
5. 前端接收 world_tick 并渲染场景
   ↓
6. 启动本地算法
   ↓
7. 本地算法接收 world_tick
   ↓
8. 本地算法运行规划器
   ↓
9. 本地算法发送 plan
   ↓
10. 服务器广播 plan
    ↓
11. 前端接收 plan 并渲染绿色轨迹
    ↓
12. 循环（每 50ms）
```

---

## 📚 进阶操作

### 多房间测试

**终端 1**: 启动服务器
```bash
cd navsim-online && bash run_navsim.sh
```

**终端 2**: 连接到房间 A
```bash
cd navsim-local && ./build/navsim_algo ws://127.0.0.1:8080/ws roomA
```

**终端 3**: 连接到房间 B
```bash
cd navsim-local && ./build/navsim_algo ws://127.0.0.1:8080/ws roomB
```

**浏览器 1**: 连接到房间 A
- URL: `ws://127.0.0.1:8080/ws`
- Room: `roomA`

**浏览器 2**: 连接到房间 B
- URL: `ws://127.0.0.1:8080/ws`
- Room: `roomB`

两个房间完全独立！

### 性能测试

```bash
# 短时间测试（5 秒）
cd navsim-local
bash test_e2e.sh

# 长时间测试（1 分钟）
bash test_long_run.sh 60

# 长时间测试（30 分钟）
bash test_long_run.sh 1800
```

### 查看日志

**服务器日志**: 终端 1 的输出

**本地算法日志**: 终端 2 的输出

**前端日志**: 浏览器控制台（F12）

---

## 🎉 恭喜！

你已经成功运行了 NavSim 系统！

**下一步**:
- 尝试修改规划算法（`navsim-local/src/planner.cpp`）
- 尝试添加更多障碍物
- 尝试不同的起点和终点
- 查看详细文档（`docs/WORKFLOW_EXPLANATION.md`）

---

**快速开始指南结束**


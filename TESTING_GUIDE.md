# 静态障碍物显示 - 完整测试指南

## 🎯 测试目标

验证修复后的系统能够正确显示静态障碍物。

---

## 📋 前置条件

1. ✅ navsim-local 已编译（`cmake --build build -j$(nproc)`）
2. ✅ navsim-online 已安装依赖（`pip install fastapi uvicorn websockets`）

---

## 🧪 完整测试流程

### 步骤 1：启动 navsim-online

```bash
cd navsim-online
python3 -m server.main
```

**预期输出**：
```
[Room demo] Room created
WebSocket server started on ws://0.0.0.0:8080/ws
INFO:     Uvicorn running on http://0.0.0.0:8080 (Press CTRL+C to quit)
```

**✅ 检查点**：看到 "WebSocket server started" 消息

---

### 步骤 2：打开 Web 界面

在浏览器中打开：`http://localhost:8080`

**预期**：
- 看到仿真场景（灰色背景）
- 左侧有控制面板
- 场景中有绿色圆形（自车）和红色圆形（目标点）

**✅ 检查点**：Web 界面正常显示

---

### 步骤 3：放置静态障碍物

在 Web 界面左侧控制面板：

1. **勾选"静态圆形"** 复选框
2. **点击"放置"** 按钮（按钮变为高亮状态）
3. **在场景中点击 3-5 个位置**（每次点击会出现一个蓝色圆形）
4. **点击"提交地图"** 按钮

**预期**：
- 场景中出现多个蓝色圆形（静态障碍物）
- 控制台输出：`[Room demo] Map updated, version=X`

**✅ 检查点**：场景中显示蓝色圆形障碍物

**⚠️ 重要**：此时**不要点击"开始"按钮**！

---

### 步骤 4：启动 navsim-local

在**新的终端**中：

```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json
```

**预期输出**：
```
[Bridge] Connecting to ws://127.0.0.1:8080/ws?room=demo
[Bridge] WebSocket connection opened
[Bridge] Connected successfully
[Main] ⏸️  Waiting for simulation to start...
[Main] Please click the 'Start' button in the Web interface
[Viz] Frame 1, Ego: (0.0, 0.0), Trajectory: 0 points, BEV circles: 0
[AlgorithmManager] WARN: Failed to process, sending fallback
```

**预期可视化窗口**：
- 窗口标题：`NavSim Visualizer`
- 左上角状态：`⏸️ Waiting for START button`
- 场景中：只有绿色圆形（自车）和红色圆形（目标点）
- **没有橙色圆形**（因为还没开始，没有收到静态地图）

**✅ 检查点 1**：日志显示 "⏸️  Waiting for simulation to start..."
**✅ 检查点 2**：可视化窗口显示 "⏸️ Waiting for START button"
**✅ 检查点 3**：日志显示 "BEV circles: 0"（正常，因为还没开始）

---

### 步骤 5：点击"开始"按钮

回到 **Web 界面**，点击左侧控制面板的 **"开始"** 按钮。

**预期 navsim-online 输出**：
```
[Room demo] 仿真已开始 (sim_running=True), will send static map in next tick
```

**预期 navsim-local 输出**：
```
[Bridge] ✅ Simulation STARTED - algorithm will now process ticks
[Main] ✅ Simulation STARTED - algorithm will now process ticks
[Bridge] Received world_tick #XXX, delay=X.Xms
[BEVExtractor] ========== Extract called ==========
[BEVExtractor] WorldTick tick_id: XXX
[BEVExtractor] Has static_map: 1  ← 关键！应该是 1
[BEVExtractor] StaticMap circles: 3  ← 您放置的障碍物数量
[BEVExtractor] ========== Extract result ==========
[BEVExtractor] Extracted circles: 3  ← 成功提取！
[AlgorithmManager] ========== Perception Input Check ==========
[AlgorithmManager] BEV obstacles in perception_input:
[AlgorithmManager]   Circles: 3  ← 传递给算法
[ImGuiVisualizer] drawBEVObstacles called:
[ImGuiVisualizer]   Input circles: 3  ← 传递给可视化
[Viz] Frame XXX, Ego: (X.X, X.X), Trajectory: XX points, BEV circles: 3
[Viz]   Drawing 3 BEV circles  ← 正在绘制！
```

**预期可视化窗口**：
- 左上角状态：`✅ Processing`
- 场景中：
  - 🟢 **绿色圆形 + 箭头** - 自车
  - 🔴 **红色圆形** - 目标点
  - 🟠 **橙色圆形（3-5 个）** - 静态障碍物 ✅✅✅
  - 🔵 **青色线条** - 规划轨迹

**✅ 检查点 1**：日志显示 "[Bridge] ✅ Simulation STARTED"
**✅ 检查点 2**：日志显示 "[BEVExtractor] Has static_map: 1"
**✅ 检查点 3**：日志显示 "[BEVExtractor] Extracted circles: X"（X > 0）
**✅ 检查点 4**：日志显示 "[Viz]   Drawing X BEV circles"
**✅ 检查点 5**：**可视化窗口中看到橙色圆形！**

---

## 🐛 故障排查

### 问题 1：日志显示 "Has static_map: 0"

**原因**：navsim-online 没有发送静态地图

**解决方案**：
1. 确认您在 Web 界面放置了障碍物并点击了"提交地图"
2. 确认您点击了"开始"按钮
3. 检查 navsim-online 的日志，应该看到 "will send static map in next tick"

### 问题 2：日志显示 "Extracted circles: 0"

**原因**：静态地图中没有圆形障碍物

**解决方案**：
1. 在 Web 界面确认勾选了"静态圆形"（不是"静态多边形"）
2. 确认点击了"放置"按钮
3. 确认在场景中点击了几个位置
4. 确认点击了"提交地图"按钮

### 问题 3：可视化窗口中看不到橙色圆形

**原因**：可能是绘制问题或者障碍物在视野外

**解决方案**：
1. 检查日志中 "[Viz]   Drawing X BEV circles" 的数量
2. 检查日志中 "[Viz]     First circle: world=(X, Y) -> screen=(X, Y)"
3. 尝试在 Web 界面的自车附近放置障碍物（距离 < 10 米）
4. 在可视化窗口中使用鼠标滚轮缩放，查看是否能看到障碍物

### 问题 4：一直显示 "⏸️ Waiting for START button"

**原因**：没有收到仿真开始信号

**解决方案**：
1. 确认在 Web 界面点击了"开始"按钮
2. 检查 navsim-online 的日志，应该看到 "仿真已开始"
3. 检查 navsim-local 的日志，应该看到 "[Bridge] ✅ Simulation STARTED"
4. 如果没有，可能是 WebSocket 消息没有正确传递，尝试重启两个程序

---

## 📸 预期效果截图说明

### Web 界面（放置障碍物后）

```
┌─────────────────────────────────────────┐
│ 控制面板                                 │
│ ☑ 静态圆形                               │
│ [放置] [提交地图]                        │
│ [开始] [暂停] [重置]                     │
└─────────────────────────────────────────┘
         场景
    🔵 🔵 🔵  ← 蓝色圆形（静态障碍物）
       🟢     ← 绿色圆形（自车）
         🔴   ← 红色圆形（目标点）
```

### navsim-local 可视化窗口（开始后）

```
┌─────────────────────────────────────────┐
│ NavSim Visualizer                       │
│ Status: ✅ Processing                   │
└─────────────────────────────────────────┘
         场景
    🟠 🟠 🟠  ← 橙色圆形（静态障碍物）✅
       🟢➡    ← 绿色圆形+箭头（自车）
      ╱🔴     ← 红色圆形（目标点）
     ╱        ← 青色线条（轨迹）
    🔵
```

---

## ✅ 成功标志

如果您看到以下所有内容，说明修复成功：

1. ✅ navsim-local 启动时显示 "⏸️  Waiting for simulation to start..."
2. ✅ 可视化窗口显示 "⏸️ Waiting for START button"
3. ✅ 点击"开始"后，日志显示 "[Bridge] ✅ Simulation STARTED"
4. ✅ 日志显示 "[BEVExtractor] Has static_map: 1"
5. ✅ 日志显示 "[BEVExtractor] Extracted circles: X"（X > 0）
6. ✅ 日志显示 "[Viz]   Drawing X BEV circles"
7. ✅ **可视化窗口中看到橙色圆形（静态障碍物）**

---

## 🎉 测试完成

如果所有检查点都通过，恭喜！静态障碍物显示问题已成功修复！

如果有任何问题，请提供：
1. navsim-online 的完整日志
2. navsim-local 的完整日志（从启动到点击"开始"后的 20 行）
3. 可视化窗口的截图

---

**测试指南版本**：1.0  
**最后更新**：2025-10-14


# 简单的浏览器调试方法

## 问题

话题控制台看不到 `plan_update` 话题，说明前端没有接收到消息。

---

## 🔍 快速诊断方法

### 方法 1: 检查 WebSocket 网络面板（最简单）

1. **打开浏览器开发者工具**
   - 按 `F12` 或右键点击页面 → "检查"

2. **切换到 Network（网络）标签**

3. **点击 WS（WebSocket）过滤器**
   - 在 Network 标签顶部，点击 "WS" 按钮

4. **查看 WebSocket 连接**
   - 应该看到一个 WebSocket 连接（例如 `ws?room=demo`）
   - 点击这个连接

5. **查看 Messages（消息）标签**
   - 在右侧面板中，点击 "Messages" 标签
   - 这里会显示所有收发的消息

6. **查找 plan_update 消息**
   - 在消息列表中，查找包含 `plan_update` 的消息
   - 绿色箭头 ↓ 表示接收的消息
   - 红色箭头 ↑ 表示发送的消息

**预期结果**:
- ✅ 应该看到很多包含 `"topic":"room/demo/plan_update"` 的消息
- ✅ 消息应该包含 `"trajectory":[...]`

**如果看不到 plan_update 消息**:
- ❌ 前端没有接收到消息
- 可能原因：
  1. 前端未连接（右上角显示"未连接"）
  2. 服务器未运行
  3. 本地算法未运行

---

### 方法 2: 检查连接状态

**查看右上角的连接状态**:
- ✅ 绿色 "已连接" - 正常
- ❌ 红色 "未连接" - 需要点击"连接 WebSocket"按钮

**如果显示"未连接"**:
1. 点击"连接 WebSocket"按钮
2. 等待几秒
3. 查看是否变为"已连接"

---

### 方法 3: 检查话题控制台

**查看右侧"话题控制台"**:
- 应该显示 "X 个话题"（X > 0）
- 话题列表中应该有：
  - `room/demo/world_tick` 或 `/room/demo/world_tick`
  - `room/demo/plan_update` 或 `/room/demo/plan_update`

**如果话题列表为空**:
- ❌ 前端没有接收到任何消息
- 检查连接状态

**如果只有 world_tick，没有 plan_update**:
- ❌ 本地算法可能未运行
- 或者消息被过滤了

---

## 🛠️ 常见问题

### 问题 1: 前端显示"未连接"

**症状**: 右上角显示红色"未连接"

**解决**:
1. 确认服务器正在运行
   ```bash
   cd navsim-online
   bash run_navsim.sh
   ```

2. 点击前端的"连接 WebSocket"按钮

3. 检查 WebSocket URL 是否正确
   - 默认: `ws://127.0.0.1:8080/ws`
   - Room ID: `demo`

---

### 问题 2: 话题控制台为空

**症状**: 话题控制台显示 "0 个话题"

**可能原因**:
1. 前端未连接
2. 服务器未运行
3. 没有客户端连接到服务器（服务器不会生成 world_tick）

**解决**:
1. 确认前端已连接（右上角绿色）
2. 启动本地算法
   ```bash
   cd navsim-local
   ./build/navsim_algo ws://127.0.0.1:8080/ws demo
   ```

---

### 问题 3: 有 world_tick 但没有 plan_update

**症状**: 话题控制台有 `world_tick` 但没有 `plan_update`

**可能原因**:
1. 本地算法未运行
2. 本地算法未发送 plan
3. 消息被过滤了

**解决**:
1. 检查本地算法日志
   - 应该看到 `[DEBUG] Sending plan:` 和 `[Bridge] Sent plan with X points`

2. 检查话题过滤器
   - 在话题控制台顶部，有一个"过滤话题"输入框
   - 确保输入框为空，或者输入 `plan` 来过滤

3. 清空话题控制台并重新连接
   - 点击"清空"按钮
   - 断开并重新连接

---

### 问题 4: WebSocket 消息中有 plan_update，但话题控制台没有

**症状**: 
- Network → WS → Messages 中可以看到 `plan_update` 消息
- 但话题控制台中没有

**可能原因**: 前端代码的 topic 匹配逻辑有问题

**调试**:

在浏览器控制台（F12 → Console）中运行：

```javascript
// 测试 topic 匹配
const testTopic = "room/demo/plan_update";
console.log("Topic:", testTopic);
console.log("endsWith('/plan_update'):", testTopic.endsWith('/plan_update'));
console.log("endsWith('/world_tick'):", testTopic.endsWith('/world_tick'));
```

**预期输出**:
```
Topic: room/demo/plan_update
endsWith('/plan_update'): true
endsWith('/world_tick'): false
```

---

## 📊 完整的检查清单

### 服务器端

- [ ] 服务器正在运行
  ```bash
  cd navsim-online && bash run_navsim.sh
  ```

### 本地算法端

- [ ] 本地算法正在运行
  ```bash
  cd navsim-local && ./build/navsim_algo ws://127.0.0.1:8080/ws demo
  ```

- [ ] 本地算法日志显示发送 plan
  ```
  [DEBUG] Sending plan:
    Topic: room/demo/plan_update
    Data keys: ... trajectory ...
    Trajectory points: 190
  [Bridge] Sent plan with 190 points
  ```

### 前端端

- [ ] 浏览器打开 http://127.0.0.1:8000/index.html

- [ ] 点击"连接 WebSocket"按钮

- [ ] 右上角显示"已连接"（绿色）

- [ ] Network → WS → Messages 中可以看到消息

- [ ] 话题控制台显示话题（> 0 个话题）

- [ ] 话题列表中有 `plan_update`

- [ ] 3D 场景中显示绿色轨迹线

---

## 🎯 最可能的问题

根据您的描述（"话题控制台看不到 plan_update 的话题"），最可能的原因是：

### 原因 1: 前端未连接

**检查**: 右上角是否显示"已连接"（绿色）

**解决**: 点击"连接 WebSocket"按钮

---

### 原因 2: 本地算法未运行或未连接

**检查**: 本地算法日志是否显示 `[Bridge] Connected successfully`

**解决**: 
```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

---

### 原因 3: 服务器未广播消息

**检查**: Network → WS → Messages 中是否有消息

**解决**: 重启服务器

---

## 🚀 推荐的调试步骤

### 步骤 1: 检查 WebSocket 消息（最重要）

1. F12 → Network → WS
2. 点击 WebSocket 连接
3. 查看 Messages 标签
4. 查找 `plan_update` 消息

**如果看到 plan_update 消息**:
- ✅ 服务器和本地算法都正常
- ❌ 问题在前端代码
- 继续步骤 2

**如果看不到 plan_update 消息**:
- ❌ 前端没有接收到消息
- 检查连接状态
- 检查本地算法是否运行

---

### 步骤 2: 检查连接状态

1. 查看右上角连接状态
2. 如果是"未连接"，点击"连接 WebSocket"按钮
3. 等待变为"已连接"

---

### 步骤 3: 检查话题控制台

1. 查看话题控制台是否有话题
2. 查找 `plan_update` 话题
3. 点击话题查看消息内容

---

### 步骤 4: 检查 3D 场景

1. 查看 3D 场景中是否有绿色轨迹线
2. 如果没有，可能是渲染问题
3. 尝试旋转视角、缩放

---

## 📝 请告诉我

请按照以下步骤操作，并告诉我结果：

1. **检查连接状态**
   - 右上角显示什么？（"已连接" 还是 "未连接"）

2. **检查 WebSocket 消息**
   - F12 → Network → WS → Messages
   - 能看到 `plan_update` 消息吗？
   - 如果能，复制一条消息的内容

3. **检查话题控制台**
   - 显示多少个话题？
   - 有哪些话题？（列出前 5 个）

4. **检查本地算法日志**
   - 是否显示 `[DEBUG] Sending plan:`？
   - 是否显示 `Topic: room/demo/plan_update`？

---

**根据您的回答，我可以精确定位问题所在！** 🎯

---

**文档结束**


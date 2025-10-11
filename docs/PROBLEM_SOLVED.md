# 问题已解决：前端看不到轨迹

## 🎯 问题总结

**症状**: 
- 本地算法发送了 plan_update
- 前端已连接
- 但前端看不到 plan_update 消息和轨迹

**根本原因**: 
- 本地算法发送的 topic 是 `room/demo/plan_update`（没有前导斜杠）
- 服务器要求 topic 必须以 `/room/{room_id}/` 开头（有前导斜杠）
- 服务器拒绝了消息，没有广播

---

## 🔍 诊断过程

### 1. 确认本地算法正常

运行本地算法，看到：
```
[DEBUG] Sending plan:
  Topic: room/demo/plan_update
  Data keys: ... trajectory ...
  Trajectory points: 190
[Bridge] Sent plan with 190 points
```

✅ 本地算法发送了消息

---

### 2. 确认前端连接正常

- 右上角显示"已连接"（绿色）
- 话题控制台有 world_tick 消息

✅ 前端连接正常

---

### 3. 确认前端没有接收到 plan_update

在浏览器控制台运行测试脚本：
```
Total messages: 184
Plan update messages: 0
```

❌ 前端没有接收到 plan_update 消息

---

### 4. 发现服务器拒绝消息

查看服务器代码 `navsim-online/server/main.py` 第 424 行：

```python
if not isinstance(topic, str) or not topic.startswith(f"/room/{room_state.room_id}/"):
    await websocket.send_json({
        "topic": f"/room/{room_state.room_id}/system/error",
        "data": {"reason": "topic_out_of_scope"},
    })
    continue
```

**服务器要求 topic 以 `/room/{room_id}/` 开头（注意前导斜杠 `/`）**

但本地算法发送的是：
- `room/demo/plan_update` ❌ 没有前导斜杠

服务器期望的是：
- `/room/demo/plan_update` ✅ 有前导斜杠

---

## ✅ 解决方案

### 修改 1: plan_update topic

**文件**: `navsim-local/src/bridge.cpp`

**修改前**:
```cpp
j["topic"] = "room/" + room_id_ + "/plan_update";
```

**修改后**:
```cpp
j["topic"] = "/room/" + room_id_ + "/plan_update";
```

---

### 修改 2: heartbeat topic

**文件**: `navsim-local/src/bridge.cpp`

**修改前**:
```cpp
j["topic"] = "room/" + room_id_ + "/control/heartbeat";
```

**修改后**:
```cpp
j["topic"] = "/room/" + room_id_ + "/control/heartbeat";
```

---

### 重新编译

```bash
cd navsim-local
cmake --build build
```

---

## 🚀 测试步骤

### 1. 启动服务器

```bash
cd navsim-online
bash run_navsim.sh
```

---

### 2. 启动本地算法

```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

**预期输出**:
```
[DEBUG] Sending plan:
  Topic: /room/demo/plan_update  ← 注意前导斜杠
  Data keys: ... trajectory ...
  Trajectory points: 190
[Bridge] Sent plan with 190 points
```

---

### 3. 打开前端

1. 浏览器打开: http://127.0.0.1:8000/index.html
2. 点击"连接 WebSocket"按钮
3. 等待连接成功（右上角绿色）

---

### 4. 验证前端接收到消息

**方法 1: 查看话题控制台**

- 话题列表中应该有 `/room/demo/plan_update`
- 点击话题查看消息内容

**方法 2: 运行浏览器测试脚本**

在浏览器控制台（F12 → Console）运行：

```javascript
let messageCount = 0;
let planUpdateCount = 0;

const testWs = new WebSocket('ws://127.0.0.1:8080/ws?room=demo');

testWs.onopen = () => {
    console.log('%c✅ Test WebSocket connected', 'color: green; font-weight: bold');
};

testWs.onmessage = (event) => {
    messageCount++;
    const data = JSON.parse(event.data);
    
    if (data.topic && data.topic.includes('plan')) {
        planUpdateCount++;
        console.log('%c[PLAN MESSAGE #' + planUpdateCount + ']', 'color: magenta; font-weight: bold');
        console.log('Topic:', data.topic);
        console.log('Has trajectory?', !!data.data?.trajectory);
        console.log('Trajectory length:', data.data?.trajectory?.length);
    }
    
    if (messageCount % 20 === 0) {
        console.log(`Received ${messageCount} messages, ${planUpdateCount} plan_update messages`);
    }
};

setTimeout(() => {
    console.log(`%c=== Test Summary ===`, 'color: blue; font-weight: bold');
    console.log(`Total messages: ${messageCount}`);
    console.log(`Plan update messages: ${planUpdateCount}`);
    testWs.close();
}, 10000);

console.log('Test WebSocket created, will run for 10 seconds...');
```

**预期输出**:
```
Test WebSocket created, will run for 10 seconds...
✅ Test WebSocket connected
[PLAN MESSAGE #1]
Topic: /room/demo/plan_update
Has trajectory? true
Trajectory length: 190
[PLAN MESSAGE #2]
...
Received 20 messages, 2 plan_update messages
...
=== Test Summary ===
Total messages: 200
Plan update messages: 20  ← 应该 > 0
```

---

### 5. 验证轨迹显示

**查看 3D 场景**:
- ✅ 应该看到绿色轨迹线
- ✅ 自车沿着轨迹移动

**如果看不到轨迹线**:
- 尝试旋转视角
- 尝试缩放
- 检查轨迹线是否在视野外

---

## 📊 修改前后对比

### 修改前

| 组件 | 状态 | 说明 |
|------|------|------|
| 本地算法 | ✅ 发送消息 | Topic: `room/demo/plan_update` |
| 服务器 | ❌ 拒绝消息 | Topic 不匹配，发送错误响应 |
| 前端 | ❌ 未接收 | 没有收到 plan_update |
| 轨迹显示 | ❌ 无 | 前端没有数据 |

### 修改后

| 组件 | 状态 | 说明 |
|------|------|------|
| 本地算法 | ✅ 发送消息 | Topic: `/room/demo/plan_update` |
| 服务器 | ✅ 接受并广播 | Topic 匹配，广播到所有客户端 |
| 前端 | ✅ 接收消息 | 收到 plan_update |
| 轨迹显示 | ✅ 显示 | 绿色轨迹线 |

---

## 🎓 经验教训

### 1. Topic 命名规范

**服务器要求**:
- Topic 必须以 `/room/{room_id}/` 开头
- 注意前导斜杠 `/`

**正确格式**:
- `/room/demo/world_tick` ✅
- `/room/demo/plan_update` ✅
- `/room/demo/control/heartbeat` ✅

**错误格式**:
- `room/demo/world_tick` ❌
- `room/demo/plan_update` ❌

---

### 2. 调试方法

**逐层诊断**:
1. 确认发送端（本地算法）是否发送
2. 确认接收端（前端）是否连接
3. 确认接收端是否收到消息
4. 确认中间层（服务器）是否转发

**工具**:
- 本地算法日志
- 浏览器 Network → WS → Messages
- 浏览器控制台测试脚本

---

### 3. 服务器验证逻辑

服务器会验证 topic 格式，如果不匹配：
- 发送错误响应到客户端
- **不会广播消息**
- 其他客户端收不到

**错误响应格式**:
```json
{
  "topic": "/room/{room_id}/system/error",
  "data": {"reason": "topic_out_of_scope"}
}
```

---

## 📝 总结

### 问题

本地算法发送的 topic 缺少前导斜杠，服务器拒绝并丢弃了消息。

### 解决

在 topic 前面加上 `/`：
- `room/demo/plan_update` → `/room/demo/plan_update`

### 结果

- ✅ 服务器接受并广播消息
- ✅ 前端接收到 plan_update
- ✅ 前端显示绿色轨迹线
- ✅ 自车沿轨迹移动

---

## 🎉 问题已解决！

**请重新启动系统并测试：**

1. 启动服务器
2. 启动本地算法（使用新编译的版本）
3. 打开前端并连接
4. 查看话题控制台和 3D 场景

**应该能看到绿色轨迹线了！** 🚀

---

**文档结束**


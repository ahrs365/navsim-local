# 调试指南：轨迹不显示问题

## 🎯 目标

确认 plan_update 消息的完整流向，找出为什么前端没有显示轨迹。

---

## 📊 消息流向

```
本地算法 → 服务器 → 广播 → 前端
```

**是的，消息先发送到服务器，然后服务器广播到所有客户端（包括前端）。**

---

## 🔍 逐步调试

### 步骤 1: 确认本地算法是否发送了消息

**查看本地算法日志**:

```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

**预期输出**:
```
[Bridge] Sent plan with 190 points, compute_ms=0.1ms
```

**如果看到这条日志**: ✅ 本地算法已发送消息

**如果没有看到**: ❌ 本地算法没有生成或发送 plan

---

### 步骤 2: 添加调试输出到本地算法

**修改 `navsim-local/src/bridge.cpp`**:

在 `publish()` 函数中添加调试输出：

```cpp
void Bridge::publish(const proto::PlanUpdate& plan, double compute_ms) {
  if (!impl_->connected_) {
    std::cerr << "[Bridge] WARN: Not connected, dropping plan" << std::endl;
    impl_->dropped_ticks_++;
    return;
  }

  impl_->update_compute_ms(compute_ms);

  // 转换为 JSON
  nlohmann::json j = impl_->plan_to_json(plan, compute_ms);

  // 【添加调试输出】
  std::cout << "[DEBUG] Sending plan_update:" << std::endl;
  std::cout << "  Topic: " << j["topic"] << std::endl;
  std::cout << "  Trajectory points: " << j["data"]["trajectory"].size() << std::endl;
  std::cout << "  Full JSON (first 500 chars): " << j.dump().substr(0, 500) << std::endl;

  // 发送
  std::string msg = j.dump();
  impl_->ws_.send(msg);
  impl_->ws_tx_++;

  std::cout << "[Bridge] Sent plan with " << plan.trajectory_size() << " points, compute_ms="
            << std::fixed << std::setprecision(1) << compute_ms << "ms" << std::endl;
}
```

**重新编译**:
```bash
cd navsim-local
cmake --build build
```

**运行并查看输出**:
```bash
./build/navsim_algo ws://127.0.0.1:8080/ws demo
```

**预期输出**:
```
[DEBUG] Sending plan_update:
  Topic: room/demo/plan_update
  Trajectory points: 190
  Full JSON (first 500 chars): {"topic":"room/demo/plan_update","data":{"schema_ver":"1.0.0","tick_id":123,"trajectory":[{"x":0.0,"y":0.0,"yaw":0.0,"t":0.0},...
[Bridge] Sent plan with 190 points, compute_ms=0.1ms
```

**检查**:
- ✅ Topic 是否为 `room/demo/plan_update`（不是 `room/demo/plan`）
- ✅ Trajectory 是否有点（不是 0）
- ✅ JSON 格式是否正确

---

### 步骤 3: 确认服务器是否接收到消息

**查看服务器日志**:

服务器默认不打印接收到的消息。我们需要添加调试输出。

**修改 `navsim-online/server/main.py`**:

找到 `websocket_endpoint` 函数，添加调试输出：

```python
@app.websocket("/ws")
async def websocket_endpoint(
    websocket: WebSocket, room_state: RoomState = Depends(get_room)
):
    await room_state.register(websocket)
    try:
        while True:
            message = await websocket.receive_text()
            payload = json.loads(message)
            topic = payload.get("topic", "")
            data = payload.get("data")
            
            # 【添加调试输出】
            if "plan" in topic:
                print(f"[DEBUG] Server received: topic={topic}")
                print(f"[DEBUG] Data keys: {list(data.keys()) if isinstance(data, dict) else 'not a dict'}")
                if isinstance(data, dict) and "trajectory" in data:
                    print(f"[DEBUG] Trajectory points: {len(data['trajectory'])}")
            
            # 验证 topic
            if not topic.startswith(f"/room/{room_state.room_id}/") and not topic.startswith(f"room/{room_state.room_id}/"):
                await websocket.send_json({
                    "topic": f"/room/{room_state.room_id}/system/error",
                    "data": {"reason": "topic_out_of_scope", "received": topic}
                })
                continue
            
            await room_state.handle_client_payload(topic, data)
            await room_state.echo(topic, data)
    except WebSocketDisconnect:
        pass
    finally:
        await room_state.unregister(websocket)
        await room_manager.cleanup_if_empty(room_state)
```

**重启服务器**:
```bash
cd navsim-online
bash run_navsim.sh
```

**预期输出**:
```
[DEBUG] Server received: topic=room/demo/plan_update
[DEBUG] Data keys: ['schema_ver', 'tick_id', 'stamp', 'n_points', 'compute_ms', 'trajectory', 'summary']
[DEBUG] Trajectory points: 190
```

**检查**:
- ✅ 服务器是否接收到消息
- ✅ Topic 是否正确
- ✅ Data 是否包含 trajectory 字段

---

### 步骤 4: 确认前端是否接收到消息

**打开浏览器控制台（F12）**:

**方法 1: 查看话题控制台**

1. 打开前端页面: http://127.0.0.1:8000/index.html
2. 点击"连接 WebSocket"按钮
3. 查看右侧"话题控制台"
4. 查找 `room/demo/plan_update` 或 `/room/demo/plan_update`

**预期**:
- ✅ 话题列表中应该有 `plan_update` 相关的话题
- ✅ 点击话题，查看消息内容

**如果没有看到**:
- ❌ 前端没有接收到消息

**方法 2: 查看浏览器控制台**

在浏览器控制台（F12 → Console）中输入：

```javascript
// 临时添加调试
const originalOnMessage = state.socket.onmessage;
state.socket.onmessage = (event) => {
    const parsed = JSON.parse(event.data);
    if (parsed.topic && parsed.topic.includes('plan')) {
        console.log('[DEBUG] Frontend received:', parsed.topic);
        console.log('[DEBUG] Data:', parsed.data);
        console.log('[DEBUG] Has trajectory?', Array.isArray(parsed.data?.trajectory));
        console.log('[DEBUG] Trajectory length:', parsed.data?.trajectory?.length);
    }
    originalOnMessage(event);
};
```

**预期输出**:
```
[DEBUG] Frontend received: room/demo/plan_update
[DEBUG] Data: {schema_ver: "1.0.0", tick_id: 123, trajectory: Array(190), ...}
[DEBUG] Has trajectory? true
[DEBUG] Trajectory length: 190
```

**方法 3: 查看网络面板**

1. 打开浏览器网络面板（F12 → Network → WS）
2. 点击 WebSocket 连接
3. 查看"Messages"标签
4. 查找包含 `plan_update` 的消息

**预期**:
- ✅ 应该看到 `{"topic":"room/demo/plan_update","data":{...}}` 消息

---

### 步骤 5: 确认前端是否调用了 handlePlanUpdate

**在浏览器控制台中添加调试**:

```javascript
// 临时覆盖 handlePlanUpdate 函数
const originalHandlePlanUpdate = handlePlanUpdate;
window.handlePlanUpdate = function(data, topic) {
    console.log('[DEBUG] handlePlanUpdate called!');
    console.log('[DEBUG] Topic:', topic);
    console.log('[DEBUG] Data:', data);
    console.log('[DEBUG] Trajectory:', data?.trajectory);
    console.log('[DEBUG] Trajectory length:', data?.trajectory?.length);
    
    // 调用原函数
    return originalHandlePlanUpdate(data, topic);
};
```

**预期输出**:
```
[DEBUG] handlePlanUpdate called!
[DEBUG] Topic: room/demo/plan_update
[DEBUG] Data: {schema_ver: "1.0.0", trajectory: Array(190), ...}
[DEBUG] Trajectory: Array(190)
[DEBUG] Trajectory length: 190
```

**如果没有输出**:
- ❌ `handlePlanUpdate` 没有被调用
- 可能是 topic 不匹配

---

### 步骤 6: 检查 topic 匹配逻辑

**在浏览器控制台中测试**:

```javascript
// 测试 topic 匹配
const testTopics = [
    'room/demo/plan_update',
    '/room/demo/plan_update',
    'room/demo/plan',
    '/room/demo/plan'
];

testTopics.forEach(topic => {
    const matches = topic.endsWith('/plan_update');
    console.log(`Topic: "${topic}" → endsWith('/plan_update'): ${matches}`);
});
```

**预期输出**:
```
Topic: "room/demo/plan_update" → endsWith('/plan_update'): true ✅
Topic: "/room/demo/plan_update" → endsWith('/plan_update'): true ✅
Topic: "room/demo/plan" → endsWith('/plan_update'): false ❌
Topic: "/room/demo/plan" → endsWith('/plan_update'): false ❌
```

**检查**:
- ✅ 本地算法发送的 topic 是否以 `/plan_update` 结尾

---

### 步骤 7: 检查轨迹渲染

**在浏览器控制台中检查**:

```javascript
// 检查轨迹线对象
console.log('Trajectory line:', state.trajectoryLine);
console.log('Trajectory visible:', state.trajectoryLine?.visible);
console.log('Trajectory geometry:', state.trajectoryLine?.geometry);
console.log('Trajectory draw range:', state.trajectoryLine?.geometry?.drawRange);

// 检查轨迹回放状态
console.log('Trajectory playback:', state.trajectoryPlayback);
```

**预期输出**:
```
Trajectory line: Line {geometry: BufferGeometry, material: LineBasicMaterial, ...}
Trajectory visible: true ✅
Trajectory geometry: BufferGeometry {attributes: {...}, drawRange: {start: 0, count: 190}}
Trajectory draw range: {start: 0, count: 190}
Trajectory playback: {trajectory: Array(190), startTime: 12345.678}
```

**如果 visible 是 false**:
- ❌ 轨迹线被隐藏了

---

## 🛠️ 快速诊断脚本

**在浏览器控制台中运行**:

```javascript
// 完整的诊断脚本
function diagnoseTrajectory() {
    console.log('=== Trajectory Diagnosis ===');
    
    // 1. 检查连接状态
    console.log('1. Connection:');
    console.log('   Connected:', state.connected);
    console.log('   Socket:', state.socket);
    
    // 2. 检查轨迹线对象
    console.log('2. Trajectory Line:');
    console.log('   Exists:', !!state.trajectoryLine);
    console.log('   Visible:', state.trajectoryLine?.visible);
    console.log('   Points:', state.trajectoryLine?.geometry?.drawRange?.count);
    
    // 3. 检查轨迹回放
    console.log('3. Trajectory Playback:');
    console.log('   Active:', !!state.trajectoryPlayback);
    console.log('   Points:', state.trajectoryPlayback?.trajectory?.length);
    
    // 4. 检查话题日志
    console.log('4. Topic Logs:');
    const planTopics = Array.from(state.topicLogs.keys()).filter(t => t.includes('plan'));
    console.log('   Plan topics:', planTopics);
    planTopics.forEach(topic => {
        const logs = state.topicLogs.get(topic);
        console.log(`   ${topic}: ${logs?.length || 0} messages`);
    });
    
    // 5. 添加消息监听
    console.log('5. Adding message listener...');
    const originalOnMessage = state.socket?.onmessage;
    if (state.socket && originalOnMessage) {
        state.socket.onmessage = (event) => {
            const parsed = JSON.parse(event.data);
            if (parsed.topic && parsed.topic.includes('plan')) {
                console.log('[PLAN MESSAGE]', parsed.topic, parsed.data);
            }
            originalOnMessage(event);
        };
        console.log('   Listener added ✅');
    } else {
        console.log('   No socket or onmessage ❌');
    }
    
    console.log('=== End Diagnosis ===');
}

// 运行诊断
diagnoseTrajectory();
```

---

## 📋 检查清单

### 本地算法端

- [ ] 编译了最新的代码（包含 plan_update 修复）
- [ ] 日志显示 `[Bridge] Sent plan with X points`
- [ ] Topic 是 `room/demo/plan_update`（不是 `room/demo/plan`）
- [ ] 数据包含 `trajectory` 字段（不是 `points`）

### 服务器端

- [ ] 服务器正在运行
- [ ] 服务器接收到消息（添加调试输出后可见）
- [ ] 服务器广播消息到所有客户端

### 前端端

- [ ] 前端已连接（右上角显示"已连接"绿色）
- [ ] 话题控制台显示 `plan_update` 消息
- [ ] 浏览器控制台没有错误
- [ ] `handlePlanUpdate` 被调用
- [ ] `state.trajectoryLine.visible` 是 `true`
- [ ] `state.trajectoryPlayback` 不是 `null`

---

## 🎯 最可能的问题

### 问题 1: 没有重新编译

**症状**: 本地算法仍然发送 `room/demo/plan` 而不是 `room/demo/plan_update`

**解决**:
```bash
cd navsim-local
cmake --build build
```

### 问题 2: Topic 前缀不一致

**症状**: 本地算法发送 `room/demo/plan_update`，但前端期望 `/room/demo/plan_update`

**检查**: 前端的 `endsWith('/plan_update')` 应该匹配两种格式

### 问题 3: 数据格式不匹配

**症状**: 本地算法发送 `points` 字段，但前端期望 `trajectory` 字段

**检查**: 确认修改后的代码使用 `data["trajectory"]`

### 问题 4: 前端未连接

**症状**: 前端页面显示"未连接"

**解决**: 点击"连接 WebSocket"按钮

---

## 📝 总结

**消息流向**:
```
本地算法 → 服务器 → 广播 → 前端
```

**调试步骤**:
1. 确认本地算法发送了消息
2. 确认服务器接收到消息
3. 确认前端接收到消息
4. 确认前端调用了 handlePlanUpdate
5. 确认轨迹线被渲染

**最快的诊断方法**:
1. 查看本地算法日志
2. 查看前端话题控制台
3. 在浏览器控制台运行 `diagnoseTrajectory()`

---

**文档结束**


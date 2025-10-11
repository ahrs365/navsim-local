# 闭环仿真系统实现文档

## 📋 概述

本文档描述了 NavSim 从**开环可视化**到**闭环仿真**的改造方案和实现细节。

---

## 🔄 系统架构对比

### 开环系统（改造前）

```
┌─────────────────────────────────────────────────────────────┐
│                    服务器 (Python)                           │
│  - 生成 world_tick (20Hz)                                   │
│  - ego.pose 固定不变 (0, 0, 0)                              │
│  - 不处理 plan_update                                       │
└────────────────┬────────────────────────────────────────────┘
                 │ WebSocket (world_tick)
                 ├──────────────┐
                 ▼              ▼
┌────────────────────────┐  ┌──────────────────────────┐
│   本地算法 (C++)        │  │   前端 (JavaScript)       │
│  - 接收 world_tick     │  │  - 接收 world_tick       │
│  - 规划轨迹            │  │  - 显示起点标记          │
│  - 发送 plan_update    │  │  - 前端轨迹回放 (60Hz)   │
└────────────┬───────────┘  └──────────────────────────┘
             │ WebSocket (plan_update)
             └──────────────────────────────────────────────┐
                                                            │
                                    ❌ 服务器不处理，只回显  │
                                                            ▼
                                                    (无闭环反馈)
```

**问题**：
- ❌ 自车位置不更新，始终在原点
- ❌ 下一帧 world_tick 基于旧的自车位置
- ❌ 无法模拟真实的闭环控制
- ❌ 前端回放与服务器状态不一致

---

### 闭环系统（改造后）

```
┌─────────────────────────────────────────────────────────────┐
│                    服务器 (Python)                           │
│  - 生成 world_tick (20Hz)                                   │
│  - ego.pose 动态更新 ✅                                      │
│  - 接收 plan_update ✅                                       │
│  - 轨迹跟踪 + 运动模型积分 ✅                                 │
└────────────────┬────────────────────────────────────────────┘
                 │ WebSocket (world_tick)
                 ├──────────────┐
                 ▼              ▼
┌────────────────────────┐  ┌──────────────────────────┐
│   本地算法 (C++)        │  │   前端 (JavaScript)       │
│  - 接收 world_tick     │  │  - 接收 world_tick       │
│  - 规划轨迹            │  │  - 显示自车位置 ✅        │
│  - 发送 plan_update    │  │  - 显示轨迹线 ✅          │
└────────────┬───────────┘  └──────────────────────────┘
             │ WebSocket (plan_update)
             └──────────────────────────────────────────────┐
                                                            │
                                    ✅ 服务器处理并更新状态  │
                                                            ▼
                                                    ┌──────────────┐
                                                    │ 轨迹跟踪模块  │
                                                    │ - 时间插值   │
                                                    │ - 位置更新   │
                                                    │ - 速度计算   │
                                                    └──────────────┘
```

**优势**：
- ✅ 自车真正沿轨迹移动
- ✅ 下一帧 world_tick 基于新的自车位置
- ✅ 真实的闭环仿真
- ✅ 前端显示与服务器状态一致

---

## 🔧 核心实现

### 1. 服务器端改动

#### 1.1 添加轨迹跟踪状态

**文件**: `navsim-online/server/main.py`

```python
@dataclass
class RoomState:
    # ... 原有字段 ...
    
    # Closed-loop trajectory tracking
    current_trajectory: List[Dict[str, Any]] = field(default_factory=list)
    trajectory_start_time: float = 0.0
    trajectory_received_time: float = 0.0
```

#### 1.2 启用自车积分

```python
async def _generate_tick(self) -> None:
    async with self.lock:
        # ...
        self._apply_pending_events()
        self._integrate_ego(dt)  # ✅ 启用自车积分
        self._integrate_dynamic(dt)
        # ...
```

#### 1.3 实现轨迹跟踪

```python
def _integrate_ego(self, dt: float) -> None:
    """Update ego state using trajectory tracking (closed-loop)."""
    
    if self.current_trajectory and len(self.current_trajectory) > 0:
        self._track_trajectory(dt)
    else:
        # Fallback: 简单运动模型（无规划器时）
        # ...

def _track_trajectory(self, dt: float) -> None:
    """Track the current trajectory using time-based interpolation."""
    now = time.time()
    elapsed = now - self.trajectory_start_time
    
    # 时间插值找到当前位置
    # 更新 ego_pose 和 ego_twist
    # ...
```

#### 1.4 处理 plan_update 消息

```python
async def handle_client_payload(self, topic: str, data: Any) -> None:
    # ... 原有处理 ...
    
    elif topic.endswith("/plan_update"):
        # ✅ 处理轨迹更新
        if isinstance(data, dict):
            self._handle_plan_update(data)

def _handle_plan_update(self, data: Dict[str, Any]) -> None:
    """Process plan_update and update trajectory tracking."""
    trajectory = data.get("trajectory", [])
    # 清洗轨迹点
    # 更新 current_trajectory
    # 记录开始时间
    # ...
```

---

### 2. 前端改动

#### 2.1 禁用前端轨迹回放

**文件**: `navsim-online/web/index.html`

```javascript
function handleWorldTick(tick, topic) {
    // ...
    
    // ✅ 闭环模式：总是使用 world_tick 的 ego.pose
    // 停止前端轨迹回放，让服务器控制自车位置
    state.trajectoryPlayback = null;
    state.egoPose = { ...state.startPose };
    updateEgoFromPose();
    
    // ...
}
```

#### 2.2 只显示轨迹线

```javascript
function handlePlanUpdate(data, topic) {
    // ...
    
    // ✅ 闭环模式：只显示轨迹线，不启动前端回放
    updateTrajectory(sanitized);
    // 不再启动 trajectoryPlayback
    
    // ...
}
```

---

## 📊 数据流详解

### 完整闭环流程

```
时刻 t=0:
  服务器: ego = (0, 0, 0)
    ↓
  world_tick → 本地算法
    ↓
  规划轨迹: [(0,0,0,t=0), (1,0,0,t=1), (2,0,0,t=2), ...]
    ↓
  plan_update → 服务器
    ↓
  服务器保存轨迹，开始跟踪

时刻 t=0.5s:
  服务器: 插值计算 ego = (0.5, 0, 0)
    ↓
  world_tick → 本地算法 + 前端
    ↓
  前端显示: 自车在 (0.5, 0, 0)
    ↓
  本地算法: 基于新位置重新规划

时刻 t=1.0s:
  服务器: 插值计算 ego = (1.0, 0, 0)
    ↓
  world_tick → 本地算法 + 前端
    ↓
  ... (循环)
```

---

## 🧪 测试方法

### 快速测试

```bash
# 终端 1: 启动服务器
cd navsim-online
bash run_navsim.sh

# 终端 2: 启动本地算法
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo

# 浏览器: 打开前端观察
# http://127.0.0.1:8000/index.html
# 点击"连接 WebSocket"按钮
# 点击"Start"按钮启动仿真
```

### 验证要点

1. **自车移动** ✅
   - 观察前端 3D 场景
   - 自车应该沿着绿色轨迹线移动
   - 不应该停留在原点

2. **位置更新** ✅
   - 查看话题控制台的 `world_tick`
   - `ego.pose.x` 和 `ego.pose.y` 应该持续变化
   - 不应该始终是 (0, 0)

3. **轨迹跟踪** ✅
   - 服务器终端应该输出：
     ```
     [Room demo] Received trajectory with N points, duration: X.XXs
     ```

4. **闭环反馈** ✅
   - 自车移动后，下一帧 `world_tick` 应该基于新位置
   - 本地算法应该基于新位置重新规划

---

## 🎯 关键特性

### 轨迹跟踪算法

- **时间插值**: 根据 `trajectory[i].t` 字段进行时间同步
- **线性插值**: 位置和角度在相邻点之间线性插值
- **速度计算**: 根据轨迹点间距和时间差计算速度
- **角速度计算**: 根据角度变化和时间差计算角速度

### 降级策略

- **无轨迹时**: 使用简单运动模型（正弦波漫游）
- **轨迹完成后**: 停在终点，清空轨迹
- **轨迹未开始**: 停在起点

---

## 📝 配置选项

### 服务器端

可以在 `RoomState` 中调整：

```python
# 轨迹跟踪参数（未来可扩展）
trajectory_tracking_gain: float = 1.0  # 跟踪增益
trajectory_lookahead: float = 0.1      # 前瞻时间
```

### 前端

可以在 `state` 中调整：

```python
// 是否启用前端回放（闭环模式下应为 false）
enableFrontendPlayback: false
```

---

## 🔍 故障排查

### 问题 1: 自车不移动

**症状**: 自车停留在原点

**检查**:
1. 服务器是否收到 `plan_update`？
   - 查看服务器终端输出
2. 仿真是否启动？
   - 前端点击"Start"按钮
3. 轨迹是否有效？
   - 检查 `plan_update.trajectory` 是否为空

### 问题 2: 自车抖动

**症状**: 自车位置跳跃

**原因**: 前端回放与服务器冲突

**解决**: 确保前端代码已更新，`trajectoryPlayback` 被设置为 `null`

### 问题 3: 轨迹不匹配

**症状**: 自车不沿轨迹线移动

**原因**: 时间同步问题

**检查**: 轨迹点的 `t` 字段是否正确

---

## 📚 相关文档

- [任务书](../../任务书.md) - 项目总体目标
- [README](../README.md) - 本地算法使用说明
- [服务器 README](../../navsim-online/server/README.md) - 服务器使用说明

---

**更新时间**: 2025-09-30
**状态**: ✅ 已实现并测试


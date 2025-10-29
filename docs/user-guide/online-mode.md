# 在线模式使用指南

NavSim Local 支持**在线模式**，可以连接到 `navsim-online` WebSocket 服务器，实现实时仿真和 Web 可视化。

## 🎯 在线模式 vs 离线模式

| 特性 | 离线模式 | 在线模式 |
|------|---------|---------|
| **可执行文件** | `navsim_local_debug` | `navsim_algo` |
| **场景来源** | JSON 文件 | WebSocket 服务器 |
| **可视化** | 控制台输出 / ImGui | Web 浏览器 |
| **交互性** | 无 | 实时编辑场景 |
| **配置方式** | 命令行参数 | JSON 配置文件 |
| **适用场景** | 批量测试、算法开发 | 实时演示、调试 |

## 🏗️ 系统架构

```
┌─────────────────┐      WebSocket      ┌──────────────────┐
│  Web Browser    │ ←─────────────────→ │  navsim-online   │
│  (可视化界面)    │      (20Hz)         │  (Python Server) │
└─────────────────┘                     └──────────────────┘
                                               ↕ WebSocket
                                              (20Hz)
                                        ┌──────────────────┐
                                        │  navsim_algo     │
                                        │  (C++ 算法程序)  │
                                        └──────────────────┘
```

**数据流**：
1. **Web → Server**: 用户在浏览器中设置起点、终点、障碍物
2. **Server → navsim_algo**: 服务器发送 `world_tick`（仿真状态）
3. **navsim_algo → Server**: 算法程序发送 `plan_update`（规划结果）
4. **Server → Web**: 服务器广播数据到浏览器进行可视化

## 🚀 快速开始

### 步骤 1：启动 navsim-online 服务器

在**新终端**中：

```bash
cd navsim-online
bash run_navsim.sh
```

**输出示例**：

```
[2025-10-18 10:30:00] 启动 WebSocket 服务: http://127.0.0.1:8080/ws
[2025-10-18 10:30:01] 启动静态前端: http://127.0.0.1:8000/index.html
按 Ctrl+C 结束两个服务
```

**说明**：
- WebSocket 服务运行在 `ws://127.0.0.1:8080/ws`
- Web 界面运行在 `http://127.0.0.1:8000/index.html`

### 步骤 2：编译 navsim_algo

在**新终端**中：

```bash
cd navsim-local
mkdir -p build && cd build
cmake .. -DBUILD_PLUGINS=ON
make -j$(nproc)
```

**验证编译**：

```bash
ls -lh navsim_algo
# 应该看到 navsim_algo 可执行文件
```

### 步骤 3：启动算法程序

```bash
./navsim_algo ws://127.0.0.1:8080/ws demo --config=../config/default.json
```

**参数说明**：
- `ws://127.0.0.1:8080/ws` - WebSocket 服务器地址
- `demo` - 房间 ID（多个客户端可以使用不同房间）
- `--config=../config/default.json` - 配置文件路径

**输出示例**：

```
=== NavSim Local Algorithm ===
WebSocket URL: ws://127.0.0.1:8080/ws
Room ID: demo
Config File: ../config/default.json
===============================

Loading config from: ../config/default.json
✓ Primary planner: AstarPlanner
✓ Fallback planner: StraightLinePlanner
✓ Perception plugins: GridMapBuilder

Connecting to WebSocket server...
✓ Connected to ws://127.0.0.1:8080/ws?room=demo

Waiting for world_tick...
```

### 步骤 4：打开 Web 界面

在浏览器中访问：

```
http://127.0.0.1:8000/index.html
```

**界面说明**：

1. **连接状态**：右上角显示 WebSocket 连接状态
2. **3D 场景**：中央显示仿真场景（地面、自车、障碍物、轨迹）
3. **控制面板**：左侧工具栏，用于设置起点、终点、添加障碍物
4. **仿真控制**：顶部按钮（开始、暂停、重置）

### 步骤 5：运行仿真

1. **设置起点**：
   - 点击左侧 "放置起点" 按钮
   - 在 3D 场景中点击设置位置
   - 拖拽鼠标确定朝向

2. **设置终点**：
   - 点击左侧 "放置终点" 按钮
   - 在 3D 场景中点击设置位置

3. **添加障碍物**（可选）：
   - 选择障碍物类型（圆形/矩形）
   - 点击场景放置
   - 拖拽确定大小

4. **开始仿真**：
   - 点击顶部 "▶ 开始" 按钮
   - 观察自车沿规划轨迹移动

## ⚙️ 配置文件

### 配置文件位置

```
navsim-local/config/default.json
```

### 配置文件结构

```json
{
  "algorithm": {
    "primary_planner": "AstarPlanner",
    "fallback_planner": "StraightLinePlanner",
    "enable_planner_fallback": true,
    "max_computation_time_ms": 25.0,
    "verbose_logging": true
  },
  "perception": {
    "preprocessing": {
      "bev_extraction": {
        "enabled": true,
        "static_obstacle_inflation": 0.2,
        "dynamic_obstacle_inflation": 0.3
      }
    },
    "plugins": [
      {
        "name": "GridMapBuilder",
        "enabled": true,
        "priority": 100,
        "params": {
          "resolution": 0.1,
          "map_width": 30.0,
          "map_height": 30.0,
          "obstacle_cost": 100,
          "inflation_radius": 0.0
        }
      }
    ]
  },
  "planning": {
    "AstarPlanner": {
      "time_step": 0.1,
      "heuristic_weight": 1.2,
      "step_size": 0.5,
      "max_iterations": 10000,
      "goal_tolerance": 0.5,
      "default_velocity": 1.5
    },
    "StraightLinePlanner": {
      "default_velocity": 1.5,
      "time_step": 0.1,
      "planning_horizon": 5.0
    }
  }
}
```

### 关键配置项

#### 算法配置 (`algorithm`)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `primary_planner` | string | "AstarPlanner" | 主规划器名称 |
| `fallback_planner` | string | "StraightLinePlanner" | 降级规划器 |
| `enable_planner_fallback` | bool | true | 启用降级机制 |
| `max_computation_time_ms` | double | 25.0 | 最大计算时间（毫秒） |
| `verbose_logging` | bool | true | 详细日志 |

> 可视化界面固定启用，无需额外配置字段。

#### 感知配置 (`perception.plugins`)

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `name` | string | - | 插件名称 |
| `enabled` | bool | true | 是否启用 |
| `priority` | int | 100 | 优先级（数值越大越先执行） |
| `params` | object | {} | 插件参数 |

#### 规划器配置 (`planning.<PlannerName>`)

每个规划器可以有自己的参数配置。

**A* 规划器参数**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `heuristic_weight` | double | 1.2 | 启发式权重（> 1 加速搜索） |
| `step_size` | double | 0.5 | 搜索步长（米） |
| `max_iterations` | int | 10000 | 最大迭代次数 |
| `goal_tolerance` | double | 0.5 | 目标容差（米） |

### 修改配置

1. **编辑配置文件**：

```bash
cd navsim-local/config
vim default.json
```

2. **重启算法程序**：

```bash
# 按 Ctrl+C 停止当前程序
# 重新启动
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/default.json
```

**注意**：配置文件修改后需要重启 `navsim_algo`，无需重启服务器。

## 🎨 Web 界面功能

### 场景编辑

| 工具 | 操作 | 说明 |
|------|------|------|
| **放置起点** | 点击 → 拖拽 | 设置起点位置和朝向 |
| **放置终点** | 点击 → 拖拽 | 设置终点位置和朝向 |
| **静态圆形障碍物** | 点击 → 拖拽 | 设置圆心和半径 |
| **静态矩形障碍物** | 点击 → 拖拽 | 设置中心、尺寸和朝向 |
| **动态障碍物** | 点击 → 拖拽 → 输入速度 | 设置运动障碍物 |

### 仿真控制

| 按钮 | 功能 | 快捷键 |
|------|------|--------|
| ▶ 开始 | 启动仿真 | - |
| ⏸ 暂停 | 暂停仿真 | - |
| ⏭ 单步 | 单步执行 | - |
| 🔄 重置 | 重置场景 | - |

### 视图控制

| 操作 | 功能 |
|------|------|
| **鼠标左键拖拽** | 旋转视角 |
| **鼠标右键拖拽** | 平移视角 |
| **鼠标滚轮** | 缩放视角 |
| **自动跟随** | 相机跟随自车 |

## 🔧 高级功能

### 启用 ImGui 桌面可视化

在线模式也可以同时启用 ImGui 桌面可视化：

```bash
# 1. 安装 SDL2
sudo apt-get install libsdl2-dev

# 2. 重新编译（启用可视化）
cd navsim-local/build
cmake .. -DENABLE_VISUALIZATION=ON -DBUILD_PLUGINS=ON
make -j$(nproc)

# 3. 运行（可视化已默认启用，无需额外开关）
./navsim_algo ws://127.0.0.1:8080/ws demo --config=../config/default.json
```

**效果**：
- Web 浏览器显示 3D 场景
- 桌面窗口显示 ImGui 调试界面（栅格地图、ESDF、参数调节）

### 多房间支持

可以同时运行多个独立的仿真实例：

```bash
# 终端 1：房间 A
./navsim_algo ws://127.0.0.1:8080/ws roomA --config=../config/default.json

# 终端 2：房间 B
./navsim_algo ws://127.0.0.1:8080/ws roomB --config=../config/default.json
```

在浏览器中连接不同房间：
- 房间 A: `http://127.0.0.1:8000/index.html?room=roomA`
- 房间 B: `http://127.0.0.1:8000/index.html?room=roomB`

## 🔍 故障排查

### 问题 1：无法连接到 WebSocket 服务器

**错误信息**：
```
Error: Failed to connect to ws://127.0.0.1:8080/ws
```

**解决方法**：
1. 检查 `navsim-online` 服务器是否运行：
   ```bash
   curl http://127.0.0.1:8080/
   ```
2. 检查端口是否被占用：
   ```bash
   lsof -i :8080
   ```
3. 重启服务器：
   ```bash
   cd navsim-online
   bash run_navsim.sh
   ```

### 问题 2：Web 界面无法加载

**解决方法**：
1. 检查静态服务器是否运行：
   ```bash
   curl http://127.0.0.1:8000/index.html
   ```
2. 手动启动静态服务器：
   ```bash
   cd navsim-online/web
   python3 -m http.server 8000
   ```

### 问题 3：规划器无响应

**现象**：设置起点终点后，自车不移动

**解决方法**：
1. 检查 `navsim_algo` 是否运行
2. 查看 `navsim_algo` 终端输出是否有错误
3. 确认已点击 "▶ 开始" 按钮
4. 使用 `--verbose` 查看详细日志

## 📚 下一步

- **自定义配置**：修改 `config/default.json` 调整参数
- **创建插件**：查看 [插件开发指南](../developer-guide/plugin-development.md)
- **性能优化**：查看 [开发工具指南](../developer-guide/development-tools.md)

---

**遇到问题？** 提交 [Issue](https://github.com/ahrs365/ahrs-simulator/issues)

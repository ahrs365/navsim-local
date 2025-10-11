# NavSim Local Algorithm

本地轨迹规划算法，通过 WebSocket 与 NavSim Online 服务器实时通信。

## ✨ 特性

- ✅ **实时通信**: WebSocket 连接，20Hz 接收频率
- ✅ **自动重连**: 指数回退策略（0.5s → 5s）
- ✅ **低延迟**: P95 延迟 29.8ms（目标 120ms）
- ✅ **高稳定性**: 0% 丢弃率，无崩溃
- ✅ **心跳机制**: 每 5 秒发送统计信息
- ✅ **延迟补偿**: 自动计算并补偿网络延迟
- ✅ **完整测试**: 提供端到端测试脚本
- ✅ **闭环仿真**: 自车根据规划轨迹真正移动 🆕

## 📋 依赖

- **C++17** 或更高版本
- **CMake** 3.10 或更高版本
- **Protobuf** 3.x
- **ixwebsocket** (已包含在 third_party/)
- **nlohmann/json** (已包含在 third_party/)

## 🔧 构建

```bash
cd navsim-local
rm -rf build
cmake -B build -S .
cmake --build build
```

## 🚀 运行

### 快速开始（推荐）

```bash
# 终端 1: 启动服务器
cd navsim-online
bash run_navsim.sh

# 终端 2: 启动本地算法
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo

# 浏览器（可选）: 打开前端观察
# http://127.0.0.1:8000/index.html
# 点击"连接 WebSocket"按钮
```

**重要**: 前端是**可选的**！本地算法可以独立运行，前端只是用于可视化。

### 运行场景

#### 场景 1: 无头运行（无前端）

```bash
# 只启动服务器和本地算法
cd navsim-online && bash run_navsim.sh  # 终端 1
cd navsim-local && ./build/navsim_algo ws://127.0.0.1:8080/ws demo  # 终端 2
```

**适用**: 批量测试、性能测试、服务器部署

#### 场景 2: 可视化运行（有前端）

```bash
# 启动服务器和本地算法
cd navsim-online && bash run_navsim.sh  # 终端 1
cd navsim-local && ./build/navsim_algo ws://127.0.0.1:8080/ws demo  # 终端 2

# 打开浏览器
# http://127.0.0.1:8000/index.html
# 点击"连接 WebSocket"按钮
```

**适用**: 开发调试、演示、可视化规划结果

#### 场景 3: 只观察仿真（无本地算法）

```bash
# 只启动服务器
cd navsim-online && bash run_navsim.sh  # 终端 1

# 打开浏览器
# http://127.0.0.1:8000/index.html
# 点击"连接 WebSocket"按钮
```

**适用**: 调试前端、观察世界状态、手动设置起点/终点

## 📖 使用说明

### 命令行参数

```bash
./build/navsim_algo <ws_url> <room_id>
```

**参数说明**:
- `ws_url`: WebSocket 服务器地址（例如: `ws://127.0.0.1:8080/ws`）
- `room_id`: 房间 ID（例如: `demo`）

**示例**:
```bash
# 本地测试
./build/navsim_algo ws://127.0.0.1:8080/ws demo

# 远程服务器
./build/navsim_algo ws://192.168.1.100:8080/ws room1

# 支持 TLS（wss://）
./build/navsim_algo wss://example.com/ws production
```

### 退出程序

按 `Ctrl+C` 优雅退出，程序会打印统计信息：

```
=== Statistics ===
WebSocket RX: 2406
WebSocket TX: 1201
Dropped ticks: 0
==================
```

## 🧪 测试

### 短时间测试（5 秒）

```bash
bash test_e2e.sh
```

**测试项目**:
- ✅ 编译状态
- ✅ 服务器连接
- ✅ 消息接收/发送
- ✅ 延迟检查
- ✅ 错误检查
- ✅ 统计信息

### 长时间测试

```bash
# 1 分钟测试
bash test_long_run.sh 60

# 30 分钟测试
bash test_long_run.sh 1800
```

**测试项目**:
- ✅ 稳定性
- ✅ 性能指标
- ✅ 心跳机制
- ✅ 错误率

## 📊 性能指标

| 指标 | 值 | 说明 |
|------|-----|------|
| **P95 延迟** | 29.8 ms | 远低于目标 120ms |
| **平均延迟** | 18.88 ms | 优秀 |
| **接收频率** | 40.10 Hz | 高于预期 20 Hz |
| **丢弃率** | 0% | 完美 |
| **计算时间** | 0.10 ms | 非常快 |

## 📁 项目结构

```
navsim-local/
├── CMakeLists.txt          # CMake 配置
├── README.md               # 本文档
├── include/
│   ├── bridge.hpp          # WebSocket 通信接口
│   └── planner.hpp         # 规划器接口
├── src/
│   ├── bridge.cpp          # WebSocket 通信实现
│   ├── planner.cpp         # 规划器实现
│   └── main.cpp            # 主程序
├── proto/
│   ├── world_tick.proto    # 世界状态消息定义
│   ├── plan_update.proto   # 规划结果消息定义
│   └── ego_cmd.proto       # 控制命令消息定义
├── third_party/
│   ├── ixwebsocket/        # WebSocket 库
│   └── nlohmann/           # JSON 库
├── test_e2e.sh             # 端到端测试脚本
├── test_long_run.sh        # 长时间测试脚本
└── docs/
    ├── websocket-integration-plan.md  # 实施方案
    ├── FINAL_COMPLETION_REPORT.md     # 最终完成报告
    └── ...                            # 其他文档
```

## 🔍 故障排查

### 问题 1: 连接失败

**症状**: `[Bridge] ERROR: WebSocket connection failed`

**解决方案**:
1. 检查服务器是否运行: `curl http://127.0.0.1:8080`
2. 检查 URL 是否正确: `ws://127.0.0.1:8080/ws`
3. 检查防火墙设置

### 问题 2: 编译失败

**症状**: `fatal error: ixwebsocket/IXWebSocket.h: No such file or directory`

**解决方案**:
1. 确保子模块已初始化: `git submodule update --init --recursive`
2. 清理并重新构建: `rm -rf build && cmake -B build -S . && cmake --build build`

### 问题 3: 高延迟

**症状**: 延迟 > 100ms

**解决方案**:
1. 检查网络连接
2. 检查服务器负载
3. 查看日志中的 WARN 信息

## 📚 文档

### 核心文档

- **[闭环仿真快速开始](../CLOSED_LOOP_QUICKSTART.md)** 🆕 - 闭环仿真使用指南
- **[闭环仿真实现文档](docs/CLOSED_LOOP_SIMULATION.md)** 🆕 - 详细技术文档
- **[闭环仿真实现总结](docs/CLOSED_LOOP_IMPLEMENTATION_SUMMARY.md)** 🆕 - 实现总结

### 历史文档

- **[实施方案](docs/websocket-integration-plan.md)** - 完整的实施方案和设计文档
- **[最终完成报告](docs/FINAL_COMPLETION_REPORT.md)** - 项目完成总结和性能指标
- **[Phase 1-3 总结](docs/PHASE1_2_3_COMPLETION_SUMMARY.md)** - Phase 1-3 开发总结
- **[Phase 4-5 完成报告](docs/PHASE4_PHASE5_COMPLETION.md)** - Phase 4-5 开发总结

---

**项目状态**: ✅ 完成并投入使用（含闭环仿真）
**最后更新**: 2025-09-30

# 快速开始指南

本指南将帮助您快速编译、运行 NavSim Local 离线仿真器。

## 📋 前置要求

### 系统要求

- **操作系统**: Linux (Ubuntu 20.04+) 或 macOS
- **编译器**: GCC 9+ 或 Clang 10+ (支持 C++17)
- **CMake**: 3.16+

### 依赖库

#### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libprotobuf-dev \
    protobuf-compiler \
    python3 \
    python3-pip
```

#### macOS

```bash
brew install cmake eigen protobuf
```

## 🔨 编译项目

### 1. 克隆仓库

```bash
git clone https://github.com/ahrs365/ahrs-simulator.git
cd ahrs-simulator/navsim-local
```

### 2. 初始化子模块

```bash
git submodule update --init --recursive
```

### 3. 编译

```bash
mkdir -p build && cd build
cmake .. -DBUILD_PLUGINS=ON
make -j$(nproc)
```

**编译选项说明**：

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `BUILD_PLUGINS` | ON | 编译内置插件 |
| `ENABLE_VISUALIZATION` | OFF | 启用 ImGui 桌面可视化 |
| `CMAKE_BUILD_TYPE` | Release | 编译类型（Release/Debug） |

### 4. 验证编译

```bash
# 检查可执行文件
ls -lh navsim_local_debug

# 检查插件
ls -lh plugins/planning/*.so
ls -lh plugins/perception/*.so
```

## 🚀 运行仿真

### 基本用法

```bash
./navsim_local_debug \
  --scenario <场景文件> \
  --planner <规划器名称> \
  [--perception <感知插件名称>]
```

### 示例 1：使用直线规划器

最简单的示例，不需要感知插件：

```bash
./navsim_local_debug \
  --scenario ../scenarios/simple_corridor.json \
  --planner StraightLinePlanner
```

**输出示例**：

```
=== NavSim Local ===
Scenario: simple_corridor.json
Planner: StraightLinePlanner
===================

Loading scenario...
✓ Loaded 5 static obstacles
✓ Start: (0.0, 0.0, 0.0)
✓ Goal: (10.0, 0.0, 0.0)

Planning...
✓ Planning succeeded
✓ Path points: 21
✓ Planning time: 0.15 ms

=== Result ===
Status: SUCCESS
Path length: 10.0 m
Total time: 0.15 ms
==============
```

### 示例 2：使用 A* 规划器

A* 规划器需要栅格地图，因此需要指定感知插件：

```bash
./navsim_local_debug \
  --scenario ../scenarios/simple_corridor.json \
  --planner AstarPlanner \
  --perception GridMapBuilder
```

**输出示例**：

```
=== NavSim Local ===
Scenario: simple_corridor.json
Planner: AstarPlanner
Perception: GridMapBuilder
===================

Loading scenario...
✓ Loaded 5 static obstacles

Building perception...
✓ GridMapBuilder initialized
✓ Grid size: 300x300 (30m x 30m, 0.1m/cell)
✓ Obstacles marked: 1250 cells

Planning...
✓ Planning succeeded
✓ Path points: 42
✓ Planning time: 3.2 ms

=== Result ===
Status: SUCCESS
Path length: 10.5 m
Total time: 3.35 ms
==============
```

### 示例 3：使用 JPS 规划器

JPS 规划器需要 ESDF 距离场：

```bash
./navsim_local_debug \
  --scenario ../scenarios/complex_obstacles.json \
  --planner JpsPlanner \
  --perception EsdfBuilder
```

## 📂 场景文件格式

场景文件使用 JSON 格式，定义起点、终点和障碍物。

### 最小示例

```json
{
  "name": "simple_test",
  "start": {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  },
  "goal": {
    "x": 10.0,
    "y": 0.0,
    "yaw": 0.0
  },
  "static_obstacles": []
}
```

### 完整示例

```json
{
  "name": "corridor_with_obstacles",
  "description": "A corridor with static obstacles",
  "start": {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  },
  "goal": {
    "x": 20.0,
    "y": 0.0,
    "yaw": 0.0
  },
  "static_obstacles": [
    {
      "type": "circle",
      "x": 5.0,
      "y": 2.0,
      "radius": 1.0
    },
    {
      "type": "circle",
      "x": 10.0,
      "y": -2.0,
      "radius": 1.5
    },
    {
      "type": "rectangle",
      "x": 15.0,
      "y": 0.0,
      "width": 2.0,
      "height": 1.0,
      "yaw": 0.785
    }
  ],
  "dynamic_obstacles": [
    {
      "type": "circle",
      "x": 8.0,
      "y": 0.0,
      "radius": 0.5,
      "vx": 0.5,
      "vy": 0.0
    }
  ]
}
```

### 字段说明

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `name` | string | 是 | 场景名称 |
| `description` | string | 否 | 场景描述 |
| `start` | object | 是 | 起点位置 (x, y, yaw) |
| `goal` | object | 是 | 终点位置 (x, y, yaw) |
| `static_obstacles` | array | 否 | 静态障碍物列表 |
| `dynamic_obstacles` | array | 否 | 动态障碍物列表 |

## 🎮 命令行参数

### 必需参数

| 参数 | 说明 | 示例 |
|------|------|------|
| `--scenario` | 场景文件路径 | `--scenario ../scenarios/test.json` |
| `--planner` | 规划器插件名称 | `--planner AstarPlanner` |

### 可选参数

| 参数 | 说明 | 默认值 | 示例 |
|------|------|--------|------|
| `--perception` | 感知插件名称 | 无 | `--perception GridMapBuilder` |
| `--verbose` | 启用详细日志 | false | `--verbose` |
| `--help` | 显示帮助信息 | - | `--help` |

## 📊 查看结果

### 控制台输出

程序会在控制台输出：

1. **场景信息**：起点、终点、障碍物数量
2. **感知信息**：地图大小、分辨率、障碍物标记
3. **规划结果**：路径点数、规划时间、成功/失败状态
4. **性能统计**：总耗时、各模块耗时

### 返回值

| 返回值 | 说明 |
|--------|------|
| 0 | 规划成功 |
| 1 | 参数错误 |
| 2 | 场景加载失败 |
| 3 | 插件加载失败 |
| 4 | 规划失败 |

## 🔍 故障排查

### 问题 1：找不到插件

**错误信息**：
```
Error: Failed to load planner plugin: AstarPlanner
```

**解决方法**：
```bash
# 检查插件是否编译
ls build/plugins/planning/

# 重新编译插件
cd build
cmake .. -DBUILD_PLUGINS=ON
make -j$(nproc)
```

### 问题 2：场景文件格式错误

**错误信息**：
```
Error: Failed to parse scenario file
```

**解决方法**：
- 检查 JSON 格式是否正确（使用 `jq` 或在线 JSON 验证器）
- 确保所有必需字段都存在
- 检查数值类型是否正确

### 问题 3：规划失败

**错误信息**：
```
Planning failed: No valid path found
```

**可能原因**：
1. 起点或终点在障碍物内
2. 障碍物完全阻挡了路径
3. 地图范围太小

**解决方法**：
- 使用 `--verbose` 查看详细日志
- 检查场景文件中的障碍物配置
- 调整地图参数（如果使用配置文件）

## 📚 下一步

- **创建自定义插件**：查看 [插件开发指南](../developer-guide/plugin-development.md)
- **使用在线模式**：查看 [在线模式指南](online-mode.md)
- **性能测试**：查看 [开发工具指南](../developer-guide/development-tools.md)

---

**遇到问题？** 提交 [Issue](https://github.com/ahrs365/ahrs-simulator/issues)


# NavSim Local 快速开始指南

本文档帮助您快速上手 NavSim Local 路径规划仿真平台，包括编译、运行和基本使用。

## 📋 目录

- [系统要求](#系统要求)
- [安装依赖](#安装依赖)
- [编译项目](#编译项目)
- [运行示例](#运行示例)
- [命令行参数](#命令行参数)
- [创建测试场景](#创建测试场景)
- [常见问题](#常见问题)

## 💻 系统要求

- **操作系统**: Linux (Ubuntu 20.04+) 或 macOS
- **编译器**: GCC 9+ 或 Clang 10+ (支持 C++17)
- **CMake**: 3.16+
- **Python**: 3.6+

## 📦 安装依赖

### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libboost-all-dev \
    python3 \
    python3-pip
```

### macOS

```bash
brew install cmake eigen boost
```

### Python 依赖

```bash
pip3 install jinja2
```

## 🔨 编译项目

### 1. 克隆仓库

```bash
cd /path/to/your/workspace
git clone <repository-url>
cd navsim-local
```

### 2. 创建构建目录

```bash
mkdir -p build
cd build
```

### 3. 配置和编译

```bash
# 配置项目
cmake ..

# 编译（使用 4 个线程）
make -j4
```

**编译选项**：

```bash
# 只编译特定插件
cmake -DBUILD_ASTAR_PLANNER_PLUGIN=ON \
      -DBUILD_JPS_PLANNER_PLUGIN=OFF \
      ..

# 启用调试模式
cmake -DCMAKE_BUILD_TYPE=Debug ..

# 启用详细输出
make VERBOSE=1
```

### 4. 验证编译

```bash
# 检查可执行文件
ls -lh navsim_local_debug

# 检查插件库
find plugins -name "*.so" -type f
```

预期输出：
```
plugins/planning/straight_line_planner/libstraight_line_plugin.so
plugins/planning/astar_planner/libastar_planner_plugin.so
plugins/planning/jps_planner/libjps_planner_plugin.so
plugins/perception/grid_map_builder/libgrid_map_builder_plugin.so
plugins/perception/esdf_builder/libesdf_builder_plugin.so
```

## 🚀 运行示例

### 示例 1: 使用 StraightLine 规划器

```bash
cd navsim-local
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner StraightLinePlanner
```

**预期输出**：
```
=== NavSim Local ===
Version: 1.0.0
====================

[1/5] Initializing plugin system...
[2/5] Loading plugins...
  Loading planner plugin: StraightLinePlanner
  Successfully loaded 1 plugins
[3/5] Loading scenario from: scenarios/simple_corridor.json
  Scenario loaded successfully
  Ego pose: (0, 0, 0)
  Goal pose: (20, 0, 0)
[4/5] Running planner...
[5/5] Planning result:
  Success: yes
  Planner: StraightLinePlanner
  Trajectory points: 50
  Computation time: 0.22 ms

=== Done ===
```

### 示例 2: 使用 A* 规划器（需要栅格地图）

```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner AstarPlanner \
  --perception GridMapBuilder
```

### 示例 3: 使用 JPS 规划器（需要 ESDF 地图）

```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner JpsPlanner \
  --perception EsdfBuilder
```

### 示例 4: 启用详细日志

```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner AstarPlanner \
  --perception GridMapBuilder \
  --verbose
```

### 示例 5: 保存规划结果

```bash
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner AstarPlanner \
  --perception GridMapBuilder \
  --output result.json
```

## 🛠️ 命令行参数

### 必需参数

- `--scenario <file>` - JSON 场景文件路径
- `--planner <name>` - 规划器插件名称或路径

### 可选参数

- `--perception <plugins>` - 逗号分隔的感知插件列表
- `--visualize` - 启用可视化（如果支持）
- `--verbose` - 启用详细日志
- `--output <file>` - 保存规划结果到 JSON 文件
- `--help` - 显示帮助信息

### 使用完整路径加载插件

```bash
./build/navsim_local_debug \
  --scenario scenarios/test.json \
  --planner /path/to/my_custom_planner/build/libmy_planner.so
```

### 加载多个感知插件

```bash
./build/navsim_local_debug \
  --scenario scenarios/complex.json \
  --planner JpsPlanner \
  --perception GridMapBuilder,EsdfBuilder
```

## 📝 创建测试场景

### 方法 1: 使用场景生成工具（推荐）

```bash
python3 tools/navsim_create_scenario.py \
    --output scenarios/my_scenario.json \
    --interactive
```

工具会引导您：
1. 设置起点和终点
2. 添加静态障碍物（圆形、矩形、多边形）
3. 添加动态障碍物
4. 配置车辆参数

### 方法 2: 手动编写 JSON

创建 `scenarios/my_scenario.json`：

```json
{
  "scenario_name": "my_test",
  "description": "My test scenario",
  "timestamp": 0.0,
  "planning_horizon": 5.0,
  "ego": {
    "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    "twist": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
    "chassis_model": "differential",
    "kinematics": {
      "wheelbase": 2.8,
      "track_width": 2.0,
      "body_length": 4.8,
      "body_width": 2.0,
      "wheel_radius": 0.3
    },
    "limits": {
      "max_velocity": 15.0,
      "max_acceleration": 3.0,
      "max_deceleration": 8.0,
      "max_steer_angle": 0.6,
      "max_curvature": 0.2
    }
  },
  "task": {
    "goal_pose": {"x": 20.0, "y": 0.0, "yaw": 0.0},
    "type": "GOTO_GOAL",
    "tolerance": {"position": 0.5, "yaw": 0.2}
  },
  "obstacles": [
    {
      "type": "circle",
      "center": {"x": 10.0, "y": 0.0},
      "radius": 1.0,
      "confidence": 1.0
    }
  ],
  "dynamic_obstacles": []
}
```

### 方法 3: 复制和修改现有场景

```bash
cp scenarios/simple_corridor.json scenarios/my_scenario.json
# 编辑 my_scenario.json
```

## ❓ 常见问题

### Q1: 编译时找不到 Eigen3

**问题**：
```
CMake Error: Could not find Eigen3
```

**解决方案**：
```bash
# Ubuntu/Debian
sudo apt-get install libeigen3-dev

# macOS
brew install eigen
```

### Q2: 插件加载失败

**问题**：
```
Failed to load planner plugin: MyPlanner
```

**解决方案**：
1. 检查插件是否编译成功：
   ```bash
   find build/plugins -name "*my_planner*"
   ```

2. 检查插件名称是否正确（区分大小写）

3. 使用完整路径加载：
   ```bash
   --planner /full/path/to/libmy_planner_plugin.so
   ```

### Q3: 规划失败 "Occupancy grid not available"

**问题**：
```
Planner 'AstarPlanner' failed: Occupancy grid not available in context
```

**解决方案**：
A* 规划器需要栅格地图，必须加载 GridMapBuilder 感知插件：
```bash
--perception GridMapBuilder
```

### Q4: 如何查看详细的规划过程

**解决方案**：
使用 `--verbose` 参数：
```bash
./build/navsim_local_debug \
  --scenario scenarios/test.json \
  --planner AstarPlanner \
  --perception GridMapBuilder \
  --verbose
```

### Q5: 如何测试自定义插件

**解决方案**：
1. 编译插件：
   ```bash
   cd build
   make my_planner_plugin -j4
   ```

2. 运行测试：
   ```bash
   ./navsim_local_debug \
     --scenario ../scenarios/simple_corridor.json \
     --planner MyPlanner
   ```

## 📚 下一步

- **创建自定义插件**：参考 [插件开发指南](PLUGIN_DEVELOPMENT.md)
- **使用开发工具**：参考 [开发工具指南](DEVELOPMENT_TOOLS.md)
- **了解架构设计**：参考 [架构设计文档](ARCHITECTURE.md)

## 🆘 获取帮助

如果遇到问题：
1. 查看 [常见问题](#常见问题) 部分
2. 查看详细文档
3. 提交 Issue


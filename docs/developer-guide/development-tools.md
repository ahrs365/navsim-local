# NavSim 开发工具指南

本文档介绍 NavSim 提供的三个核心开发工具，帮助您快速开发和测试插件。

## 📋 目录

1. [工具概览](#工具概览)
2. [插件脚手架工具](#插件脚手架工具)
3. [场景生成工具](#场景生成工具)
4. [性能分析工具](#性能分析工具)
5. [完整工作流示例](#完整工作流示例)

## 🛠️ 工具概览

| 工具 | 功能 | 用途 | 节省时间 |
|------|------|------|----------|
| `navsim_create_plugin.py` | 插件脚手架 | 5 分钟创建新插件 | ~1.5 小时 |
| `navsim_create_scenario.py` | 场景生成 | 快速创建测试场景 | ~30 分钟 |
| `navsim_benchmark.py` | 性能分析 | 批量测试和对比 | ~1 小时 |

## 🏗️ 插件脚手架工具

### 功能

自动生成插件模板代码，包括：
- ✅ `algorithm/` 层（纯算法实现）
- ✅ `adapter/` 层（平台接口适配）
- ✅ `CMakeLists.txt`（构建配置）
- ✅ `README.md`（插件文档）
- ✅ 注册代码（双重注册机制）

### 基本用法

```bash
python3 tools/navsim_create_plugin.py \
    --name MyPlanner \
    --type planner \
    --output plugins/planning/my_planner \
    --author "Your Name" \
    --description "My awesome planner"
```

### 参数说明

| 参数 | 必需 | 说明 | 示例 |
|------|------|------|------|
| `--name` | ✅ | 插件名称（CamelCase） | `MyPlanner` |
| `--type` | ✅ | 插件类型 | `planner` 或 `perception` |
| `--output` | ✅ | 输出目录 | `plugins/planning/my_planner` |
| `--author` | ❌ | 作者名称 | `Your Name` |
| `--description` | ❌ | 插件描述 | `My awesome planner` |
| `--verbose` | ❌ | 详细输出 | - |

### 示例

#### 创建规划器插件

```bash
python3 tools/navsim_create_plugin.py \
    --name RRTStarPlanner \
    --type planner \
    --output plugins/planning/rrt_star \
    --author "John Doe" \
    --description "RRT* path planner"
```

**输出**:
```
Creating planner plugin: RRTStarPlanner
  Output directory: plugins/planning/rrt_star
  Template: templates/planner_plugin

  Created: plugins/planning/rrt_star/README.md
  Created: plugins/planning/rrt_star/CMakeLists.txt
  Created: plugins/planning/rrt_star/algorithm/rrt_star_planner.hpp
  Created: plugins/planning/rrt_star/algorithm/rrt_star_planner.cpp
  Created: plugins/planning/rrt_star/adapter/rrt_star_planner_plugin.hpp
  Created: plugins/planning/rrt_star/adapter/rrt_star_planner_plugin.cpp
  Created: plugins/planning/rrt_star/adapter/register.cpp

✅ Plugin created successfully!
```

#### 创建感知插件

```bash
python3 tools/navsim_create_plugin.py \
    --name PointCloudMapBuilder \
    --type perception \
    --output plugins/perception/point_cloud_map
```

### 生成的目录结构

```
my_planner/
├── algorithm/              # 算法层（纯算法，无平台依赖）
│   ├── my_planner.hpp
│   └── my_planner.cpp
├── adapter/                # 适配器层（平台接口适配）
│   ├── my_planner_plugin.hpp
│   ├── my_planner_plugin.cpp
│   └── register.cpp
├── CMakeLists.txt
└── README.md
```

### 下一步

1. **实现算法**: 编辑 `algorithm/my_planner.cpp` 中的 `plan()` 方法
2. **添加到构建系统**: 在 `plugins/planning/CMakeLists.txt` 中添加：
   ```cmake
   add_subdirectory(my_planner)
   ```
3. **编译插件**:
   ```bash
   cd build
   cmake ..
   make my_planner_plugin
   ```
4. **测试插件**:
   ```bash
   ./build/navsim_local_debug \
       --planner MyPlanner \
       --scenario scenarios/simple_corridor.json
   ```

---

## 🎬 场景生成工具

### 功能

快速创建和编辑 JSON 场景文件，支持：
- 交互式创建
- 从模板创建
- 编辑现有场景

### 用法

#### 从模板创建

```bash
python3 tools/navsim_create_scenario.py \
    --template corridor \
    --output scenarios/my_corridor.json
```

#### 交互式创建

```bash
python3 tools/navsim_create_scenario.py \
    --output scenarios/my_scenario.json
```

#### 列出可用模板

```bash
python3 tools/navsim_create_scenario.py --list-templates
```

**输出**:
```
Available templates:
  - empty: Empty scenario template
  - corridor: Navigate through a corridor with obstacles
  - parking: Park in a tight space
```

### 参数说明

| 参数 | 必需 | 说明 | 示例 |
|------|------|------|------|
| `--output` | ✅ | 输出文件路径 | `scenarios/my_scenario.json` |
| `--template` | ❌ | 使用模板 | `corridor`, `parking`, `empty` |
| `--edit` | ❌ | 编辑现有场景 | `scenarios/simple_corridor.json` |
| `--list-templates` | ❌ | 列出可用模板 | - |

### 示例

#### 创建走廊场景

```bash
python3 tools/navsim_create_scenario.py \
    --template corridor \
    --output scenarios/narrow_corridor.json
```

#### 创建停车场景

```bash
python3 tools/navsim_create_scenario.py \
    --template parking \
    --output scenarios/parallel_parking.json
```

#### 交互式创建自定义场景

```bash
python3 tools/navsim_create_scenario.py \
    --output scenarios/custom_scenario.json
```

**交互式提示**:
```
=== NavSim Scenario Creator ===

Scenario name: My Custom Scenario
Description: A custom test scenario

--- Ego Vehicle ---

Initial pose:
  x (m) [0.0]: 0
  y (m) [0.0]: 0
  yaw (rad) [0.0]: 0
Initial velocity vx (m/s) [0.0]: 0
...
```

---

## 📊 性能分析工具

### 功能

批量测试场景，生成性能报告：
- 测试单个或多个规划器
- 对比不同规划器性能
- 生成 JSON 和 HTML 报告
- 统计成功率、计算时间等

### 用法

#### 测试单个规划器

```bash
python3 tools/navsim_benchmark.py \
    --planner JpsPlanner \
    --perception GridMapBuilder,ESDFBuilder \
    --scenarios scenarios/*.json \
    --output reports/jps_benchmark.json
```

#### 对比多个规划器

```bash
python3 tools/navsim_benchmark.py \
    --planners JpsPlanner,AStarPlanner,StraightLinePlanner \
    --perception GridMapBuilder,ESDFBuilder \
    --scenarios scenarios/*.json \
    --output reports/comparison.json \
    --html reports/comparison.html
```

### 参数说明

| 参数 | 必需 | 说明 | 示例 |
|------|------|------|------|
| `--planner` | ❌* | 单个规划器 | `JpsPlanner` |
| `--planners` | ❌* | 多个规划器（逗号分隔） | `JpsPlanner,AStarPlanner` |
| `--perception` | ❌ | 感知插件（逗号分隔） | `GridMapBuilder,ESDFBuilder` |
| `--scenarios` | ✅ | 场景文件（支持通配符） | `scenarios/*.json` |
| `--navsim-debug` | ❌ | navsim_local_debug 路径 | `build/navsim_local_debug` |
| `--output` | ❌ | JSON 报告路径 | `reports/benchmark.json` |
| `--html` | ❌ | HTML 报告路径 | `reports/benchmark.html` |
| `--verbose` | ❌ | 详细输出 | - |

*注：`--planner` 和 `--planners` 必须指定其中一个

### 示例

#### 测试 JPS 规划器

```bash
python3 tools/navsim_benchmark.py \
    --planner JpsPlanner \
    --perception GridMapBuilder,ESDFBuilder \
    --scenarios scenarios/*.json \
    --output reports/jps_benchmark.json \
    --html reports/jps_benchmark.html
```

**输出**:
```
=== NavSim Benchmark ===
Planners: JpsPlanner
Perception: GridMapBuilder, ESDFBuilder
Scenarios: 3

Testing JpsPlanner...
  dynamic_obstacles.json... ✓ (10.73 ms)
  parking_scenario.json... ✓ (10.95 ms)
  simple_corridor.json... ✓ (8.35 ms)

============================================================
Planner: JpsPlanner
============================================================
Total scenarios:      3
Success:              3 (100.0%)
Failure:              0
Avg computation time: 10.01 ms
Min computation time: 8.35 ms
Max computation time: 10.95 ms
Std computation time: 1.44 ms

✅ JSON report saved to: reports/jps_benchmark.json
✅ HTML report saved to: reports/jps_benchmark.html
```

#### 对比三个规划器

```bash
python3 tools/navsim_benchmark.py \
    --planners StraightLinePlanner,AStarPlanner,JpsPlanner \
    --perception GridMapBuilder,ESDFBuilder \
    --scenarios scenarios/*.json \
    --html reports/planner_comparison.html
```

### 报告格式

#### JSON 报告

```json
{
  "timestamp": "2025-10-18 14:46:11",
  "results": [
    {
      "scenario": "simple_corridor.json",
      "planner": "JpsPlanner",
      "perception": ["GridMapBuilder", "ESDFBuilder"],
      "success": true,
      "trajectory_points": 2,
      "computation_time_ms": 8.35,
      "failure_reason": ""
    }
  ],
  "summaries": [
    {
      "planner": "JpsPlanner",
      "total_scenarios": 3,
      "success_count": 3,
      "failure_count": 0,
      "success_rate": 100.0,
      "avg_computation_time_ms": 10.01,
      "min_computation_time_ms": 8.35,
      "max_computation_time_ms": 10.95,
      "std_computation_time_ms": 1.44
    }
  ]
}
```

#### HTML 报告

生成的 HTML 报告包含：
- 汇总表格（成功率、平均时间等）
- 详细结果表格（每个场景的测试结果）
- 美观的样式和颜色标记

---

## 🚀 完整工作流示例

### 场景 1: 开发新的规划器插件

```bash
# 1. 创建插件
python3 tools/navsim_create_plugin.py \
    --name MyPlanner \
    --type planner \
    --output plugins/planning/my_planner \
    --author "Your Name"

# 2. 实现算法
vim plugins/planning/my_planner/algorithm/my_planner.cpp

# 3. 添加到构建系统
echo "add_subdirectory(my_planner)" >> plugins/planning/CMakeLists.txt

# 4. 编译
cd build && cmake .. && make my_planner_plugin

# 5. 创建测试场景
cd ..
python3 tools/navsim_create_scenario.py \
    --template corridor \
    --output scenarios/test_my_planner.json

# 6. 测试插件
./build/navsim_local_debug \
    --planner MyPlanner \
    --scenario scenarios/test_my_planner.json

# 7. 性能测试
python3 tools/navsim_benchmark.py \
    --planner MyPlanner \
    --scenarios scenarios/*.json \
    --html reports/my_planner_benchmark.html
```

### 场景 2: 对比多个规划器

```bash
# 1. 创建多个测试场景
python3 tools/navsim_create_scenario.py --template corridor --output scenarios/corridor_1.json
python3 tools/navsim_create_scenario.py --template corridor --output scenarios/corridor_2.json
python3 tools/navsim_create_scenario.py --template parking --output scenarios/parking_1.json

# 2. 运行对比测试
python3 tools/navsim_benchmark.py \
    --planners JpsPlanner,AStarPlanner,StraightLinePlanner \
    --perception GridMapBuilder,ESDFBuilder \
    --scenarios scenarios/*.json \
    --output reports/comparison.json \
    --html reports/comparison.html

# 3. 查看 HTML 报告
firefox reports/comparison.html
```

---

## 📚 相关文档

- [本地调试模式](LOCAL_DEBUG_MODE.md)
- [可用插件列表](AVAILABLE_PLUGINS.md)
- [插件加载机制](PLUGIN_LOADING_MECHANISM.md)
- [重构方案](../REFACTORING_PROPOSAL.md)

---

## 💡 提示和技巧

### 快速迭代开发

```bash
# 使用 watch 命令自动重新编译
watch -n 2 'cd build && make my_planner_plugin'

# 使用别名简化命令
alias navsim_test='./build/navsim_local_debug --scenario scenarios/simple_corridor.json'
navsim_test --planner MyPlanner
```

### 批量创建场景

```bash
# 使用循环创建多个场景
for i in {1..5}; do
  python3 tools/navsim_create_scenario.py \
      --template corridor \
      --output scenarios/corridor_$i.json
done
```

### 自动化测试

```bash
# 创建测试脚本
cat > test_all.sh << 'EOF'
#!/bin/bash
python3 tools/navsim_benchmark.py \
    --planners $(ls build/plugins/planning/*/*.so | xargs -n1 basename | sed 's/lib//;s/_plugin.so//' | tr '\n' ',') \
    --scenarios scenarios/*.json \
    --html reports/all_planners.html
EOF

chmod +x test_all.sh
./test_all.sh
```

---

## 🎯 总结

使用这三个工具，您可以：

1. **5 分钟创建新插件** - 使用 `navsim_create_plugin.py`
2. **快速创建测试场景** - 使用 `navsim_create_scenario.py`
3. **自动化性能测试** - 使用 `navsim_benchmark.py`

这大大提高了开发效率，让您专注于算法实现，而不是样板代码和测试环境搭建。


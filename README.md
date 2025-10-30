# NavSim Local


> 想马上运行起来？参考[快速上手指南](docs/quick-start.md)。

NavSim Local 是一个面向自动驾驶/移动机器人规划算法研发的“全内环”（perception → planning → control）沙箱平台。它在单进程(便于前期开发，之后版本可能改为多进程)内同时运行仿真器、感知与规划插件框架以及 ImGui/SDL2 可视化前端。项目以插件化设计为核心，开发者可以自由替换或扩展各个阶段的算法组件。

在线场景编辑器：[地址](https://www.gl-robotics.com/navsim-online/index.html)

![alt text](docs/assets/image-6.png)


---

## 核心特性

- **模块化架构**：独立的预处理流水线、感知插件管理器和规划插件管理器由 `AlgorithmManager` 统一调度。
- **插件首位**：所有感知/规划能力均由插件实现，可在运行时动态加载或静态编译。
- **本地仿真**：内置 `LocalSimulator` 支持 JSON 场景回放、动态障碍物积分、碰撞检测与控制指令回传。
- **可视化调试**：ImGui 控制台实时展示 BEV 障碍物、轨迹、仿真状态与性能统计。
- **双运行模式(todo)**：既可离线加载场景运行，也能通过 WebSocket 与 `navsim-online` 协同。
- **规划器插件社区**：当前仅实现了一个差分驱动的轨迹优化插件，后边会更新更多的规划器插件。会在公众号里写文章预告。

---

## 架构总览

### 简明框图

![NavSim Local Overview](docs/assets/架构图.png)

---

```mermaid
flowchart LR
  subgraph Sources["数据入口"]
    WT["WorldTick\nProto"]
    SC["Scenario JSON"]
    BR["Bridge / 在线模式"]
  end

  subgraph LocalSim["Local Simulator"]
    Sim["LocalSimulator"]
  end

  SC -->|加载| Sim
  Sim -->|世界状态| WT
  BR -->|远端世界状态| WT

  WT --> PP["Preprocessing Pipeline\n(BEV Extractor &amp; Predictor)"]
  PP -->|PerceptionInput| PPM["Perception Plugin Manager"]

  subgraph Perception["感知插件层"]
    PPM -->|栅格 / ESDF / 地图特征| PC["PlanningContext"]
  end

  PC --> PLM["Planner Plugin Manager"]

  subgraph Planning["规划插件层"]
    PLM -->|调用| Primary["Primary Planner"]
    PLM -->|回退| Fallback["Fallback Planner"]
    Primary -->|PlanningResult| PLM
    Fallback -->|PlanningResult| PLM
  end

  PLM --> TT["TrajectoryTracker\n(Playback &amp; Hold)"]
  TT --> EgoCmd["EgoCmd Proto"]
  EgoCmd --> Sim

  Sim -.-> Viz["ImGui Visualizer"]
  PPM -.-> Viz
  PLM -.-> Viz
  TT -.-> Viz
  Viz <--> BR
```

核心 orchestrator `AlgorithmManager` 负责：

- 初始化并管理插件系统、预处理流水线、轨迹跟踪器和可视化器；
- 划分感知/规划时间预算、统计性能指标、处理失败重试；
- 与 `LocalSimulator` 或外部 Bridge 交互并维持仿真循环；
- 在接近目标点时复用“Hold Trajectory”，避免终点附近抖动。

主要子系统说明：

- **数据入口**：场景文件、在线 Bridge 与 LocalSimulator 都能提供 `WorldTick`；
- **Preprocessing Pipeline**：规范化世界状态，输出标准 `PerceptionInput`；
- **感知插件层**：按优先级构建栅格、ESDF 等地图，在 `PlanningContext` 中汇总；
- **规划插件层**：`PlannerPluginManager` 动态选择主/备规划器，遵循时间预算；
- **TrajectoryTracker**：裁剪与缓冲轨迹，维护 hold 逻辑并输出 `EgoCmd`；
- **Visualizer & Bridge**：同步感知/规划结果，用于本地调试或在线联机。



## 插件系统

### 管理器

| 组件 | 作用 |
|------|------|
| `PerceptionPluginManager` | 根据优先级执行已启用的感知插件，把产物写入 `PlanningContext` |
| `PlannerPluginManager` | 选择主/备规划插件执行规划，可在失败时回退到备用 |

插件通过 `plugin::PluginRegistry` 注册，可静态链接也可动态加载 (`DynamicPluginLoader` 会从 `./build/plugins`、`./plugins` 中查找)。插件接口位于 `platform/include/plugin/framework`：

- `PerceptionPluginInterface`：接收 `PerceptionInput`、输出地图/障碍物等到 `PlanningContext`。
- `PlannerPluginInterface`：读取 `PlanningContext`，在给定时间预算内计算轨迹，生成 `PlanningResult`。

### 预置插件

**感知**
- `GridMapBuilder`：构建可配置栅格地图（分辨率、膨胀半径等）。
- `EsdfBuilder`：基于 occupancy grid 生成 ESDF，可选择是否包含动态障碍物。

**规划**
- `StraightLinePlanner`：自车几何直线规划，配合梯形速度曲线，作为极简 fallback。
- `AstarPlanner（不可用）`：基于网格的 A* 搜索，支持权重调节。
- `JpsPlanner`：跳点搜索 + 轨迹优化（LBFGS），具备大量优化参数可调。

所有内建插件默认构建为 `navsim_builtin_plugins` 并与核心静态库链接。

### 配置方式

运行时配置位于 `config/default.json`（或自定义文件）：

```jsonc
{
  "planning": {
    "primary_planner": "JpsPlanner",
    "fallback_planner": "StraightLine",
    "enable_fallback": true,
    "planners": {
      "JpsPlanner": {
        "safe_dis": 0.3,
        "jps_truncation_time": 5.0,
        "optimizer": {
          "collision_weight": 5e5
        }
      }
    }
  },
  "perception": {
    "plugins": [
      {
        "name": "GridMapBuilder",
        "enabled": true,
        "priority": 100,
        "params": {
          "resolution": 0.1,
          "map_width": 30.0,
          "map_height": 30.0,
          "inflation_radius": 0.2
        }
      },
      {
        "name": "EsdfBuilder",
        "enabled": true,
        "priority": 90,
        "params": {
          "max_distance": 5.0,
          "include_dynamic": true
        }
      }
    ]
  },
  "algorithm": {
    "max_computation_time_ms": 25.0,
    "verbose_logging": false,
    "goal_hold_distance_": 2.0
  }
}
```

要编写自定义插件：
1. 继承对应接口，完成 `initialize`、`process` 等方法。
2. 使用宏注册（参考内建插件）。
3. 将插件加入构建系统（新建 `CMakeLists.txt`，修改 `plugins/<category>/CMakeLists.txt`）。
4. 在配置文件中声明并启用该插件。

---

## 仿真引擎

### LocalSimulator

位于 `platform/src/sim/local_simulator.cpp`，职责包括：

- 加载 JSON 场景 (`scenarios/*.json`) 并转化为 `WorldState`；
- 按设定的时间步 (`SimulatorConfig.time_step`) 积分动态障碍物与自车运动；
- 执行碰撞检测（自车简化为圆形模型）并触发回调；
- 支持暂停/恢复/重置，记录实时倍率与统计信息；
- 通过回调将 `WorldTick` 发送给 `AlgorithmManager`。

场景文件包含起终点、静态/动态障碍物、容差等信息，可直接编辑或生成。

### 轨迹追踪与 Goal Hold

`control::TrajectoryTracker` 基于规划结果计算跟踪策略（lookahead 时间/距离可调），并管理“hold trajectory”，在自车接近目标（距离 ≤ `goal_hold_distance`）时复用上一条轨迹，减少终点振荡。

当前为了最真实的反应出规划器的轨迹，默认采用PLAYBACK模式，即单纯回放轨迹，没有采用复杂的跟踪算法。

### 可视化

`viz::ImGuiVisualizer`（SDL2 后端）负责：

- 绘制 BEV 障碍物、ESDF、轨迹、动态障碍物、系统信息。
- 管理仿真控制（开始/暂停/重置）并展示连接状态。
- 若初始化失败，会自动降级为 `NullVisualizer`，并在日志中给出提示。

---

## 构建指南

### 环境依赖

- CMake ≥ 3.16
- C++17 编译器（GCC / Clang）
- Protobuf 编译器与库
- SDL2 开发包（Linux: `sudo apt-get install libsdl2-dev`; macOS: `brew install sdl2`）
- ImGui + ImPlot（脚本会自动下载到 `third_party`，若不存在）
- ixwebsocket（已包含于 `third_party/ixwebsocket`）
- Eigen3（系统安装或使用 `third_party/eigen` 兜底）
- nlohmann/json（header-only，已包含）

### 一键脚本

```bash
# 本地仿真（默认场景 map1）
./build.sh local

# 指定场景
./build.sh -m map5 local

# WebSocket 模式（需 navsim-online 后端）
./build.sh websocket

# 清理重建
./build.sh -c local
```

脚本行为：
- 始终启用可视化（检查 SDL2 并拉取 ImGui）。
- 使用 `RelWithDebInfo` 编译（可切换 `-d/--debug` 或 `-r/--release`）。
- 产出可执行文件 `build/navsim_algo`，并在需要时构建插件共享库。

### 手动构建

```bash
mkdir -p build && cd build
cmake .. \
  -DENABLE_VISUALIZATION=ON \
  -DBUILD_PLUGINS=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build . -j$(nproc)
```

---

## 运行指南

### 本地仿真模式

```bash
./build/navsim_algo \
  --local-sim \
  --scenario=scenarios/map3.json \
  --config=config/default.json
```

- ImGui 窗口自动打开
- 可通过配置文件调整插件参数、Max computation time、Goal hold 距离等。

### WebSocket 联机模式（todo）

目前online模式，仅支持场景绘制和底盘配置，保存地图等功能，联机模式等待下阶段开发。

```bash
./build/navsim_algo \
  ws://127.0.0.1:8080/ws demo \
  --config=config/default.json
```

- 建议先启动 `navsim-online` 服务器。
- 应用作为客户端连接，接收远端 `WorldTick`，发布计划结果与心跳。
- 可通过 Bridge 的调试接口发送感知诊断信息到前端。

### 调试与监控

- `algorithm.verbose_logging = true` 可输出详细的感知/规划耗时及轨迹信息。
- 可视化器右上角的系统信息展示正在使用的配置、主/备规划器、连接状态等。


---

## 开发建议

1. 新增插件或算法时，项目提供了脚本自动生成插件模板，但是有待改进，待完善以后再使用。
2. 开发自己的规划器阶段，可用`build_and_run_debug.sh`进行快速调试。这是一个单帧规划的脚本。
3. navsim-local/scenarios/目录放置了一些json地图，这些地图是通过navsim-online在线编辑绘制的，包含了起点终点，障碍物，动态障碍物，底盘类型，运动学约束。在网页上下载后放在此目录即可。在线网站暂未上线，大家可以先用这内置的地图体验，在线网站即将上线，敬请期待。

---

## 结果展示
### map1

感觉不够平滑

![alt text](docs/assets/map1.png)

### map2

起步倒车了，这个ddr-opt的特性，老是喜欢倒车

![alt text](docs/assets/map2.png)

### map3

起步先倒车了

![alt text](docs/assets/map3.png)

### map4

轨迹不平滑，且起点有倒车，这个倒车是个问题，起步老是喜欢先倒车

![alt text](docs/assets/map4.png)

### map6

看速度曲线，起点有倒车，这个没必要，单纯自转就行了

![alt text](docs/assets/map6.png)

### map7

这个轨迹不平滑啊，估计是哪项权重太大了

![alt text](docs/assets/map7.png)

### map8

这个轨迹看着不怎么好，终点附近不够平滑，感觉是受障碍物影响比较大
![alt text](docs/assets/map8.png)

### map9 

起点附近有多余的倒车，不是很合理

![alt text](docs/assets/map9.png)

## License

本项目依据 GNU General Public License v3.0（GPL-3.0）授权发布，详见仓库根目录的 `LICENSE`。

## Citing

如果你的工作基于此项目，尤其是在使用 JPS 规划器插件时引用了相关算法实现，请参考以下文献：

```bibtex
@ARTICLE{zhang2024universaltrajectoryoptimizationframework,
  author={Zhang, Mengke and Chen, Nanhe and Wang, Hu and Qiu, Jianxiong and Han, Zhichao and Ren, Qiuyu and Xu, Chao and Gao, Fei and Cao, Yanjun},
  journal={IEEE Transactions on Automation Science and Engineering},
  title={Universal Trajectory Optimization Framework for Differential Drive Robot Class},
  year={2025},
  volume={22},
  number={},
  pages={13030-13045},
  keywords={Robots;Mobile robots;Kinematics;Trajectory optimization;Planning;Robot kinematics;Computational modeling;Dynamics;Wheels;Tracking;Motion planning;trajectory optimization;differential drive robot class;nonholonomic dynamics},
  doi={10.1109/TASE.2025.3550676}
}
```

**更多开发细节，可关注公众号了解： 哎嗨人生**

![NavSim Local Overview](docs/assets/微信公众号.jpg)
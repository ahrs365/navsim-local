# NavSim Local 文档索引

欢迎查阅 NavSim Local 文档！本文档库分为用户指南和开发者指南两部分。

## 📘 用户指南

面向使用 NavSim Local 进行路径规划仿真的用户。

### [快速开始](user-guide/getting-started.md)

**阅读时间**: 10 分钟  
**适合人群**: 新用户

**内容**：
- 系统要求和依赖安装
- 编译项目
- 运行离线仿真
- 场景文件格式
- 命令行参数说明
- 故障排查

**快速预览**：

```bash
# 编译
cd navsim-local && mkdir build && cd build
cmake .. -DBUILD_PLUGINS=ON && make -j$(nproc)

# 运行
./navsim_local_debug \
  --scenario ../scenarios/simple_corridor.json \
  --planner AstarPlanner \
  --perception GridMapBuilder --visualize
```

---

### [在线模式](user-guide/online-mode.md)

**阅读时间**: 15 分钟  
**适合人群**: 需要 Web 可视化的用户

**内容**：
- 在线模式 vs 离线模式
- 系统架构和数据流
- 启动 navsim-online 服务器
- 启动 navsim_algo 算法程序
- 配置文件详解
- Web 界面使用
- 高级功能（ImGui 可视化、多房间）

**快速预览**：

```bash
# 终端 1: 启动服务器
cd navsim-online && bash run_navsim.sh

# 终端 2: 启动算法
cd navsim-local/build
./navsim_algo ws://127.0.0.1:8080/ws demo --config=../config/default.json

# 浏览器: 打开 http://127.0.0.1:8000/index.html
```

---

## 🔧 开发者指南

面向开发自定义规划器插件的开发者。

### [插件开发](developer-guide/plugin-development.md)

**阅读时间**: 30 分钟  
**适合人群**: 算法开发者

**内容**：
- 插件系统概述
- 三层架构详解（算法层、适配层、平台层）
- 使用脚手架工具创建插件
- 实现自定义规划器
- 编译和测试插件
- 完整示例（A* Planner）

**快速预览**：

```bash
# 创建插件
python3 tools/navsim_create_plugin.py \
  --name MyPlanner \
  --type planner \
  --output plugins/planning/my_planner

# 实现算法
# 编辑 plugins/planning/my_planner/algorithm/my_planner.cpp

# 编译测试
cd build && cmake .. && make my_planner_plugin
./navsim_local_debug --planner MyPlanner --scenario ../scenarios/test.json --visualize
```

---

### [架构设计](developer-guide/architecture.md)

**阅读时间**: 20 分钟  
**适合人群**: 深入了解系统的开发者

**内容**：
- 系统整体架构
- 三层解耦设计理念
- 插件加载机制（动态加载 + 静态注册）
- 数据流和生命周期
- 核心组件详解
- 设计模式和最佳实践

**关键概念**：

```
Platform Layer (平台层)
    ↓ 定义接口
Adapter Layer (适配层)
    ↓ 调用算法
Algorithm Layer (算法层)
```

---

### [开发工具](developer-guide/development-tools.md)

**阅读时间**: 15 分钟  
**适合人群**: 需要批量测试和性能分析的开发者

**内容**：
- 插件脚手架工具（`navsim_create_plugin.py`）
- 场景生成工具（`navsim_create_scenario.py`）
- 性能基准测试工具（`navsim_benchmark.py`）
- 工具使用示例
- 自定义工具开发

**快速预览**：

```bash
# 生成场景
python3 tools/navsim_create_scenario.py \
  --output scenarios/my_test.json \
  --interactive

# 性能测试
python3 tools/navsim_benchmark.py \
  --scenarios scenarios/*.json \
  --planners AstarPlanner,JpsPlanner \
  --output results.json
```

---

## 📂 文档结构

```
docs/
├── README.md                          # 本文档（索引）
├── user-guide/                        # 用户指南
│   ├── getting-started.md             # 快速开始
│   └── online-mode.md                 # 在线模式
├── developer-guide/                   # 开发者指南
│   ├── plugin-development.md          # 插件开发
│   ├── architecture.md                # 架构设计
│   └── development-tools.md           # 开发工具
└── archive/                           # 历史文档归档
    └── ...
```

## 🗺️ 学习路径

### 新用户路径

1. **[快速开始](user-guide/getting-started.md)** - 编译和运行离线仿真（10 分钟）
2. **[在线模式](user-guide/online-mode.md)** - 体验 Web 可视化（15 分钟）

**总时间**: 25 分钟

---

### 开发者路径

1. **[快速开始](user-guide/getting-started.md)** - 了解基本用法（10 分钟）
2. **[插件开发](developer-guide/plugin-development.md)** - 创建第一个插件（30 分钟）
3. **[架构设计](developer-guide/architecture.md)** - 深入理解系统（20 分钟）
4. **[开发工具](developer-guide/development-tools.md)** - 提高开发效率（15 分钟）

**总时间**: 75 分钟

---

### 高级用户路径

1. **[在线模式](user-guide/online-mode.md)** - 配置文件和高级功能（15 分钟）
2. **[架构设计](developer-guide/architecture.md)** - 理解插件机制（20 分钟）
3. **[开发工具](developer-guide/development-tools.md)** - 性能测试和对比（15 分钟）

**总时间**: 50 分钟

---

## 🔗 相关资源

### 项目文档

- **[主 README](../README.md)** - 项目概述和快速导航
- **[配置文件说明](../config/README.md)** - 在线模式配置详解

### 外部资源

- **[GitHub 仓库](https://github.com/ahrs365/ahrs-simulator)** - 源代码和 Issue 跟踪
- **[navsim-online 文档](../../navsim-online/README.md)** - 在线仿真服务器文档

---

## 📝 文档贡献

发现文档错误或有改进建议？欢迎贡献！

1. Fork 仓库
2. 编辑文档（Markdown 格式）
3. 提交 Pull Request

**文档规范**：
- 使用清晰的 Markdown 格式
- 包含可运行的代码示例
- 添加必要的图表（使用 Mermaid）
- 中文文档，专业术语可保留英文

---

**最后更新**: 2025-10-18


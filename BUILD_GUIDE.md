# NavSim Local - 编译和运行指南

## 快速开始

### 1. 本地仿真模式（推荐用于开发和测试）

```bash
cd navsim-local
./build.sh local
```

这将：
- 编译项目
- 从 `scenarios/map1.json` 加载场景
- 运行本地仿真
- 使用 `config/default.json` 配置

### 2. WebSocket 在线模式（推荐用于演示和远程调试）

```bash
# 终端1：启动 navsim-online 服务器
cd navsim-online/server
python3 main.py

# 终端2：编译并运行 navsim_algo
cd navsim-local
./build.sh websocket

# 终端3：打开浏览器
# http://localhost:8080
# 在网页上绘制场景，点击"开始"按钮
```

---

## 编译选项

### 基本用法

```bash
./build.sh [选项] [模式]
```

### 选项说明

| 选项 | 说明 |
|------|------|
| `-h, --help` | 显示帮助信息 |
| `-c, --clean` | 清理旧的构建 |
| `-d, --debug` | 使用 Debug 构建类型 |
| `-r, --release` | 使用 Release 构建类型 |
| `--no-build` | 跳过编译，直接运行 |
| `--build-only` | 只编译，不运行 |

### 模式说明

| 模式 | 说明 |
|------|------|
| `local` | 本地仿真模式（从 JSON 文件加载场景） |
| `websocket` | WebSocket 在线模式（从前端接收场景） |

---

## 常用命令示例

### 开发场景

```bash
# 首次编译
./build.sh local

# 修改代码后重新编译并运行
./build.sh local

# 清理并重新编译
./build.sh -c local

# Debug 模式编译（用于调试）
./build.sh -d local
```

### 可视化依赖

ImGui 窗口现已默认启用，请确保本地已安装可视化依赖：

```bash
# 安装 SDL2（首次使用）
sudo apt-get install libsdl2-dev
```

**可视化控制键**：
- `F` - 跟随自车
- `+/-` - 缩放
- `ESC` - 关闭窗口
- `Ctrl+C` - 停止仿真

### 性能测试

```bash
# Release 模式编译（最高性能）
./build.sh -r local
```

### 只编译不运行

```bash
# 只编译
./build.sh --build-only

# 稍后运行
./build/navsim_algo --local-sim \
  --scenario=scenarios/map1.json \
  --config=config/default.json
```

### 跳过编译直接运行

```bash
# 如果已经编译过，可以跳过编译
./build.sh --no-build local
```

---

## 手动编译（高级用法）

如果你需要更精细的控制，可以手动使用 CMake：

### 1. 配置

```bash
cd navsim-local

# 基本配置（可视化默认启用）
cmake -B build -S . \
  -DENABLE_VISUALIZATION=ON \
  -DBUILD_PLUGINS=ON \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### 2. 编译

```bash
# 使用所有 CPU 核心编译
cmake --build build -j$(nproc)

# 或者指定核心数
cmake --build build -j8
```

### 3. 运行

```bash
# 本地仿真模式
./build/navsim_algo --local-sim \
  --scenario=scenarios/map1.json \
  --config=config/default.json

# WebSocket 在线模式
./build/navsim_algo ws://127.0.0.1:8080/ws demo \
  --config=config/default.json
```

---

## 构建类型说明

| 构建类型 | 优化级别 | 调试信息 | 用途 |
|---------|---------|---------|------|
| `Debug` | 无优化 | 完整 | 开发调试 |
| `RelWithDebInfo` | 优化 | 部分 | 日常开发（默认） |
| `Release` | 最高优化 | 无 | 性能测试、发布 |

---

## 目录结构

```
navsim-local/
├── build.sh                    # 编译和运行脚本（新）
├── build_with_visualization.sh # 旧脚本（已废弃）
├── BUILD_GUIDE.md              # 本文档
├── QUICK_START.md              # 快速开始指南
├── build/                      # 编译输出目录
│   ├── navsim_algo            # 主程序
│   └── plugins/               # 插件库
├── scenarios/                  # 场景文件
│   └── map1.json              # 示例场景
├── config/                     # 配置文件
│   └── default.json           # 默认配置
├── apps/                       # 应用程序源码
├── platform/                   # 平台核心代码
└── third_party/               # 第三方库
    └── imgui/                 # ImGui（可选）
```

---

## 故障排除

### 1. 编译错误

**问题**：找不到头文件或库

**解决**：
```bash
# 安装依赖
sudo apt-get install -y \
  build-essential \
  cmake \
  libprotobuf-dev \
  protobuf-compiler \
  libeigen3-dev \
  nlohmann-json3-dev

# 如果启用可视化
sudo apt-get install libsdl2-dev
```

### 2. 场景文件不存在

**问题**：`scenarios/map1.json` 不存在

**解决**：
```bash
# 创建示例场景文件
mkdir -p scenarios
cat > scenarios/map1.json << 'EOF'
{
  "ego": {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  },
  "goal": {
    "x": 10.0,
    "y": 10.0,
    "yaw": 0.0,
    "tolerance": 0.5
  },
  "static_obstacles": [],
  "dynamic_obstacles": []
}
EOF
```

### 3. WebSocket 连接失败

**问题**：`Unable to connect to 127.0.0.1 on port 8080`

**解决**：
```bash
# 确保 navsim-online 服务器已启动
cd navsim-online/server
python3 main.py
```

### 4. 插件加载失败

**问题**：`Failed to find plugin: XXX`

**解决**：
```bash
# 清理并重新编译
./build.sh -c --build-only

# 检查插件是否存在
ls -la build/plugins/planning/
ls -la build/plugins/perception/
```

---

## 性能优化建议

### 1. 编译优化

```bash
# 使用 Release 模式
./build.sh -r local

# 或者手动指定优化标志
cmake -B build -S . \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-O3 -march=native"
```

### 2. 运行优化

```bash
# 禁用可视化（提高性能）
./build.sh local

# 使用更小的时间步长
# 修改 config/default.json 中的 time_step
```

---

## 下一步

- 阅读 [QUICK_START.md](QUICK_START.md) 了解更多使用方法
- 查看 [../docs/ARCHITECTURE_CLARIFICATION.md](../docs/ARCHITECTURE_CLARIFICATION.md) 了解架构设计
- 查看 [../docs/FINAL_SUMMARY.md](../docs/FINAL_SUMMARY.md) 了解项目完整功能

---

## 常见问题

**Q: 如何切换不同的场景？**

A: 修改命令行参数：
```bash
./build/navsim_algo --local-sim \
  --scenario=scenarios/my_scene.json \
  --config=config/default.json
```

**Q: 如何切换不同的规划器？**

A: 修改 `config/default.json` 中的 `primary_planner` 和 `fallback_planner`。

**Q: 如何添加自定义插件？**

A: 参考 `plugins/planning/jps_planner/` 或 `plugins/perception/grid_map_builder/` 的实现。

**Q: 如何调试代码？**

A: 使用 Debug 模式编译并使用 GDB：
```bash
./build.sh -d --build-only
gdb --args ./build/navsim_algo --local-sim \
  --scenario=scenarios/map1.json \
  --config=config/default.json
```

# NavSim Local 可视化指南

## 📖 概述

NavSim Local 提供了基于 **ImGui + SDL2** 的实时可视化工具，用于开发和调试。

### 特性

- ✅ **所见即所得**：直接在 `AlgorithmManager::process()` 中可视化，无需数据转换
- ✅ **实时渲染**：60 FPS 流畅显示
- ✅ **零开销**：可通过编译选项完全禁用，生产环境无性能影响
- ✅ **交互控制**：支持缩放、平移、跟随自车等功能

### 显示内容

1. **感知数据**
   - 自车状态（位置、朝向、速度）
   - 目标点
   - BEV 障碍物（圆形、矩形、多边形）
   - 动态障碍物及预测轨迹
   - 栅格占据地图

2. **规划结果**
   - 规划轨迹
   - 规划器名称
   - 计算时间统计

3. **调试信息**
   - 处理状态
   - 性能指标
   - 障碍物数量
   - 轨迹点数

---

## 🔧 安装依赖

### Ubuntu/Debian

```bash
# 安装 SDL2
sudo apt-get update
sudo apt-get install libsdl2-dev

# 下载 ImGui（header-only）
cd navsim-local/third_party
git clone https://github.com/ocornut/imgui.git --depth 1
```

### macOS

```bash
# 安装 SDL2
brew install sdl2

# 下载 ImGui
cd navsim-local/third_party
git clone https://github.com/ocornut/imgui.git --depth 1
```

---

## 🚀 编译和运行

### 启用可视化编译

```bash
cd navsim-local
rm -rf build
cmake -B build -S . -DENABLE_VISUALIZATION=ON -DBUILD_PLUGINS=ON
cmake --build build
```

### 运行（带可视化）

```bash
# 方式 1: 通过配置文件启用
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json

# 方式 2: 通过命令行参数启用（需要添加参数解析）
./build/navsim_algo ws://127.0.0.1:8080/ws demo --visualize
```

### 禁用可视化编译（生产环境）

```bash
cd navsim-local
rm -rf build
cmake -B build -S . -DENABLE_VISUALIZATION=OFF
cmake --build build
```

---

## ⌨️ 交互控制

### 键盘快捷键

| 按键 | 功能 |
|------|------|
| `F` | 切换跟随自车模式 |
| `+` / `=` | 放大视图 |
| `-` | 缩小视图 |
| `ESC` | 关闭可视化窗口 |

### 鼠标操作

- **左键拖拽**：平移视图（未实现，待扩展）
- **滚轮**：缩放视图（未实现，待扩展）

---

## 📝 配置文件

创建 `config/with_visualization.json`：

```json
{
  "algorithm": {
    "primary_planner": "AStarPlanner",
    "fallback_planner": "StraightLinePlanner",
    "enable_planner_fallback": true,
    "max_computation_time_ms": 25.0,
    "verbose_logging": true,
    "enable_visualization": true
  },
  "perception": {
    "plugins": [
      {
        "name": "GridMapBuilder",
        "enabled": true,
        "priority": 100,
        "params": {
          "resolution": 0.1,
          "map_width": 100.0,
          "map_height": 100.0
        }
      }
    ]
  }
}
```

---

## 🎨 代码集成示例

### 在 AlgorithmManager::process() 中使用

可视化已经自动集成到 `AlgorithmManager::process()` 中，无需手动调用。

```cpp
bool AlgorithmManager::process(const proto::WorldTick& world_tick,
                              std::chrono::milliseconds deadline,
                              proto::PlanUpdate& plan_update,
                              proto::EgoCmd& ego_cmd) {
  // 🎨 自动开始新帧
  if (visualizer_) visualizer_->beginFrame();
  
  // 处理感知
  auto perception_input = preprocessing_pipeline.process(world_tick);
  
  // 🎨 自动可视化感知数据
  if (visualizer_) {
    visualizer_->drawEgo(perception_input.ego);
    visualizer_->drawGoal(perception_input.task.goal_pose);
    visualizer_->drawBEVObstacles(perception_input.bev_obstacles);
    visualizer_->drawDynamicObstacles(perception_input.dynamic_obstacles);
  }
  
  // 处理规划
  planner_plugin_manager_->plan(context, remaining_time, planning_result);
  
  // 🎨 自动可视化规划结果
  if (visualizer_) {
    visualizer_->drawTrajectory(planning_result.trajectory, planning_result.planner_name);
  }
  
  // 🎨 自动结束帧并渲染
  if (visualizer_) visualizer_->endFrame();
  
  return true;
}
```

### 自定义可视化器

如果需要自定义可视化逻辑，可以实现 `IVisualizer` 接口：

```cpp
#include "viz/visualizer_interface.hpp"

class MyCustomVisualizer : public navsim::viz::IVisualizer {
public:
  bool initialize() override {
    // 初始化你的可视化系统
    return true;
  }
  
  void drawTrajectory(const std::vector<plugin::TrajectoryPoint>& trajectory,
                      const std::string& planner_name) override {
    // 自定义绘制逻辑
    for (const auto& point : trajectory) {
      // 绘制轨迹点
    }
  }
  
  // 实现其他接口...
};
```

---

## 🐛 故障排除

### 问题 1: 找不到 SDL2

**错误信息**：
```
CMake Error: Could not find SDL2
```

**解决方案**：
```bash
# Ubuntu/Debian
sudo apt-get install libsdl2-dev

# macOS
brew install sdl2
```

### 问题 2: 找不到 ImGui

**错误信息**：
```
CMake Error: ImGui not found at third_party/imgui
```

**解决方案**：
```bash
cd navsim-local/third_party
git clone https://github.com/ocornut/imgui.git --depth 1
```

### 问题 3: OpenGL 错误

**错误信息**：
```
Failed to create OpenGL context
```

**解决方案**：
- 确保系统有 OpenGL 驱动
- Ubuntu: `sudo apt-get install mesa-utils`
- 检查是否在远程 SSH 会话中（需要 X11 转发）

### 问题 4: 窗口无法显示

**可能原因**：
- 在无图形环境的服务器上运行
- 缺少 X11 显示

**解决方案**：
- 在本地机器上运行
- 或使用 SSH X11 转发：`ssh -X user@server`

---

## 📊 性能影响

### 启用可视化

- **额外开销**：约 5-10 ms/帧（取决于场景复杂度）
- **适用场景**：本地开发、调试

### 禁用可视化

- **额外开销**：0 ms（编译器优化掉所有调用）
- **适用场景**：生产环境、性能测试

---

## 🔮 未来扩展

### 计划功能

- [ ] 完整的 2D 场景渲染（障碍物、轨迹）
- [ ] 鼠标交互（拖拽、缩放）
- [ ] 录制和回放功能
- [ ] 导出图片/视频
- [ ] 3D 可视化（可选）
- [ ] 性能分析图表

### 贡献

欢迎提交 PR 添加新功能！

---

## 📚 参考资料

- [ImGui 官方文档](https://github.com/ocornut/imgui)
- [SDL2 官方文档](https://wiki.libsdl.org/)
- [OpenGL 教程](https://learnopengl.com/)


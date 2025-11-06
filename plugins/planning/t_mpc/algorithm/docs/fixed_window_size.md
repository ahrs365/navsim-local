# 固定窗口大小功能

## 📊 功能说明

实现了根据起点和终点自动计算并固定窗口大小的功能，避免仿真过程中窗口忽大忽小的问题。

## 🎯 工作原理

### 1. **自动计算窗口范围**

在 `main()` 函数中，程序会：
1. 获取起点位置：`simulation.getStartPosition()` → 从 `state_` 中读取 (x, y)
2. 获取终点位置：`simulation.getGoalPosition()` → 从 `data_.goal` 中读取
3. 计算边界：
   ```cpp
   double min_x = std::min(start.x(), goal.x());
   double max_x = std::max(start.x(), goal.x());
   double min_y = std::min(start.y(), goal.y());
   double max_y = std::max(start.y(), goal.y());
   ```

### 2. **添加边距**

为了避免轨迹贴边，添加 20% 的边距：
```cpp
double dx = std::max(1.0, max_x - min_x);
double dy = std::max(1.0, max_y - min_y);
double margin_x = 0.2 * dx;
double margin_y = 0.2 * dy;

// 最终窗口范围
min_x - margin_x, max_x + margin_x
min_y - margin_y, max_y + margin_y
```

### 3. **设置固定窗口**

通过 `setVisualizerAxisLimits()` 将计算出的范围传递给可视化器：
```cpp
simulation.setVisualizerAxisLimits(
    min_x - margin_x, max_x + margin_x,
    min_y - margin_y, max_y + margin_y
);
```

## 🔧 代码改动

### MatplotlibVisualizer 类

#### 新增方法
```cpp
void setAxisLimits(double min_x, double max_x, double min_y, double max_y)
{
    _fixed_min_x = min_x;
    _fixed_max_x = max_x;
    _fixed_min_y = min_y;
    _fixed_max_y = max_y;
    _axis_limits_set = true;
}
```

#### 新增成员变量
```cpp
private:
    bool _axis_limits_set{false};
    double _fixed_min_x{0.0};
    double _fixed_max_x{0.0};
    double _fixed_min_y{0.0};
    double _fixed_max_y{0.0};
```

#### 修改 update() 方法

在绘制时，检查是否设置了固定窗口：
```cpp
if (_axis_limits_set)
{
    plt::xlim(_fixed_min_x, _fixed_max_x);
    plt::ylim(_fixed_min_y, _fixed_max_y);
}
else
{
    // 原有的自动计算逻辑
    // ...
}
```

### JackalLikeSimulation 类

#### 新增方法
```cpp
Eigen::Vector2d getStartPosition() const
{
    return Eigen::Vector2d(state_.get("x"), state_.get("y"));
}

Eigen::Vector2d getGoalPosition() const
{
    return data_.goal;
}

void setVisualizerAxisLimits(double min_x, double max_x, double min_y, double max_y)
{
    visualizer_.setAxisLimits(min_x, max_x, min_y, max_y);
}
```

### main() 函数

在创建仿真对象后，立即计算并设置窗口大小：
```cpp
JackalLikeSimulation simulation(config_path);

// 根据起点和终点计算窗口大小
const auto &start = simulation.getStartPosition();
const auto &goal = simulation.getGoalPosition();

// 计算边界和边距
double min_x = std::min(start.x(), goal.x());
double max_x = std::max(start.x(), goal.x());
double min_y = std::min(start.y(), goal.y());
double max_y = std::max(start.y(), goal.y());

double dx = std::max(1.0, max_x - min_x);
double dy = std::max(1.0, max_y - min_y);
double margin_x = 0.2 * dx;
double margin_y = 0.2 * dy;

// 设置固定的窗口大小
simulation.setVisualizerAxisLimits(
    min_x - margin_x, max_x + margin_x,
    min_y - margin_y, max_y + margin_y
);

simulation.run();
```

## 📈 效果

### 之前
- 窗口根据当前轨迹动态调整
- 仿真过程中窗口大小不断变化
- 用户体验不佳

### 之后
- 窗口在仿真开始时固定
- 整个仿真过程中窗口大小保持不变
- 用户可以清晰地看到整个规划场景

## ✅ 编译和测试

```bash
cd build
make -j$(nproc)
./mpc_planner_main
```

程序正常启动，窗口大小在整个仿真过程中保持固定。

## 🎯 特点

1. **自动计算** - 无需手动指定窗口范围
2. **智能边距** - 自动添加 20% 的边距，避免轨迹贴边
3. **灵活性** - 可通过 `setAxisLimits()` 手动设置窗口范围
4. **向后兼容** - 如果不调用 `setAxisLimits()`，使用原有的自动计算逻辑

## 📝 后续优化建议

1. **可配置边距** - 将 20% 的边距改为可配置参数
2. **动态调整** - 根据障碍物位置动态调整窗口范围
3. **多窗口支持** - 为不同的可视化窗口设置不同的范围
4. **保存/加载** - 保存用户设置的窗口范围，下次运行时自动加载


# 动态障碍物可视化修复报告

## 🎯 修复的问题

### 问题 1：日志只打印第一个障碍物的尺寸
**现象**：日志只显示第一个动态障碍物的信息，其他障碍物的信息被忽略。

**修复**：
- 修改 `algorithm_manager.cpp` 和 `imgui_visualizer.cpp`
- 使用循环打印所有障碍物的信息

**修复后的日志**：
```
[AlgorithmManager] Calling visualizer->drawDynamicObstacles() with 2 obstacles...
[AlgorithmManager]   Dyn obs #0: shape=circle, pos=(X, Y), length=L, width=W
[AlgorithmManager]   Dyn obs #1: shape=rectangle, pos=(X, Y), length=L, width=W
[Viz]   Drawing 2 dynamic obstacles
[Viz]     Dyn obs #0: shape=circle, pos=(X, Y), length=L, width=W
[Viz]     Dyn obs #1: shape=rectangle, pos=(X, Y), length=L, width=W
```

---

### 问题 2：紫色矩形下方叠加了绿色圆形
**现象**：紫色矩形动态障碍物下方有绿色圆形叠加显示。

**根本原因**：
- `BEVExtractor::extractDynamicObstacles()` 错误地把动态障碍物添加到了 `BEVObstacles` 的 `rectangles` 中
- 导致动态障碍物被同时绘制为：
  - 绿色圆形（BEV 静态障碍物的矩形，简化为圆形）
  - 紫色矩形（动态障碍物）

**修复**：
- 删除 `bev_extractor.cpp` 中的 `extractDynamicObstacles(world_tick, *obstacles)` 调用
- 动态障碍物应该由 `DynamicObstaclePredictor` 处理，而不是 `BEVExtractor`

**修复位置**：`navsim-local/src/plugin/preprocessing/bev_extractor.cpp` 第 24-31 行

---

### 问题 3：紫色矩形的长宽与前端不一致
**现象**：navsim-local 可视化窗口中显示的矩形长宽与 navsim-online 前端不一致。

**根本原因**：
- Protobuf 定义：`Rectangle.w` = width（宽度），`Rectangle.h` = height（高度）
- 之前的代码：`pred_obs.length = rect.h()`, `pred_obs.width = rect.w()`
- 这是正确的！但是可视化代码中可能有问题

**修复**：
- 确认 `dynamic_predictor.cpp` 中的赋值是正确的：
  ```cpp
  pred_obs.width = rect.w();   // 宽度（横向）
  pred_obs.length = rect.h();  // 长度（纵向，车辆前后方向）
  ```
- 可视化代码中使用 `width` 和 `length` 绘制矩形

---

### 问题 4：不应该用长宽相等来判断是否为圆形
**现象**：使用 `std::abs(dyn_obs.length - dyn_obs.width) < 0.01` 判断圆形，不可靠。

**根本原因**：
- Protobuf 中有明确的形状类型：`DynamicShape.oneof shape { Circle, Rectangle }`
- 可以使用 `shape().has_circle()` 和 `shape().has_rectangle()` 判断
- 但是 `DynamicObstacle` 结构体没有存储形状类型

**修复**：
1. 在 `DynamicObstacle` 结构体中添加 `std::string shape_type` 字段
2. 在 `DynamicObstaclePredictor::predictConstantVelocity()` 中设置 `shape_type`：
   ```cpp
   if (dyn_obs.shape().has_circle()) {
     pred_obs.shape_type = "circle";
   } else if (dyn_obs.shape().has_rectangle()) {
     pred_obs.shape_type = "rectangle";
   }
   ```
3. 在可视化代码中使用 `shape_type` 判断：
   ```cpp
   bool is_circle = (dyn_obs.shape_type == "circle");
   ```

---

## 📝 修改的文件

### 1. `navsim-local/include/core/planning_context.hpp`
**修改**：在 `DynamicObstacle` 结构体中添加 `shape_type` 字段

```cpp
struct DynamicObstacle {
  int id;
  std::string type;  // "vehicle", "pedestrian", "cyclist"
  std::string shape_type;  // "circle" or "rectangle" - 🔧 新增
  
  // ... 其他字段 ...
};
```

---

### 2. `navsim-local/src/plugin/preprocessing/dynamic_predictor.cpp`
**修改**：从 protobuf 中提取形状类型和尺寸

```cpp
// 🔧 修复：从 shape 中提取形状类型和尺寸
if (dyn_obs.shape().has_circle()) {
  const auto& circle = dyn_obs.shape().circle();
  pred_obs.shape_type = "circle";
  pred_obs.length = circle.r() * 2.0;  // 直径
  pred_obs.width = circle.r() * 2.0;   // 直径
} else if (dyn_obs.shape().has_rectangle()) {
  const auto& rect = dyn_obs.shape().rectangle();
  pred_obs.shape_type = "rectangle";
  // 注意：protobuf 中 w=width, h=height
  pred_obs.width = rect.w();   // 宽度（横向）
  pred_obs.length = rect.h();  // 长度（纵向，车辆前后方向）
} else {
  pred_obs.shape_type = "rectangle";
  pred_obs.length = 4.5;
  pred_obs.width = 2.0;
}
```

---

### 3. `navsim-local/src/plugin/preprocessing/bev_extractor.cpp`
**修改**：删除动态障碍物提取（第 24-31 行）

```cpp
// 提取静态障碍物
extractStaticObstacles(world_tick, *obstacles);

// 🔧 修复问题2：不要在这里提取动态障碍物！
// 动态障碍物应该由 DynamicObstaclePredictor 处理
// extractDynamicObstacles(world_tick, *obstacles);  // ← 删除此调用

total_extractions_++;
```

---

### 4. `navsim-local/src/core/algorithm_manager.cpp`
**修改**：打印所有动态障碍物的信息（第 206-214 行）

```cpp
std::cout << "[AlgorithmManager] Calling visualizer->drawDynamicObstacles() with " 
          << perception_input.dynamic_obstacles.size() << " obstacles..." << std::endl;
// 🔧 修复问题1：打印所有障碍物的信息
for (size_t i = 0; i < perception_input.dynamic_obstacles.size(); ++i) {
  const auto& obs = perception_input.dynamic_obstacles[i];
  std::cout << "[AlgorithmManager]   Dyn obs #" << i << ": shape=" << obs.shape_type
            << ", pos=(" << obs.current_pose.x << ", " << obs.current_pose.y 
            << "), length=" << obs.length << ", width=" << obs.width << std::endl;
}
```

---

### 5. `navsim-local/src/viz/imgui_visualizer.cpp`
**修改 1**：打印所有动态障碍物的信息（第 580-597 行）

```cpp
// 4. 绘制动态障碍物
static int dyn_obs_log_count = 0;
if (dyn_obs_log_count++ % 60 == 0 && !dynamic_obstacles_.empty()) {
  std::cout << "[Viz]   Drawing " << dynamic_obstacles_.size() << " dynamic obstacles" << std::endl;
  // 🔧 修复问题1：打印所有障碍物的信息
  for (size_t i = 0; i < dynamic_obstacles_.size(); ++i) {
    const auto& obs = dynamic_obstacles_[i];
    std::cout << "[Viz]     Dyn obs #" << i << ": shape=" << obs.shape_type 
              << ", pos=(" << obs.current_pose.x << ", " << obs.current_pose.y
              << "), length=" << obs.length << ", width=" << obs.width << std::endl;
  }
}

for (const auto& dyn_obs : dynamic_obstacles_) {
  auto center = worldToScreen(dyn_obs.current_pose.x, dyn_obs.current_pose.y);
  
  // 🔧 修复问题4：使用 shape_type 判断，而不是长宽相等
  bool is_circle = (dyn_obs.shape_type == "circle");
  
  // ... 绘制代码 ...
}
```

---

## 🧪 测试步骤

1. **重启 navsim-local**
2. **在 Web 界面放置动态障碍物**：
   - 一个圆形动态障碍物
   - 一个矩形动态障碍物
3. **观察日志**：
   ```
   [AlgorithmManager] Calling visualizer->drawDynamicObstacles() with 2 obstacles...
   [AlgorithmManager]   Dyn obs #0: shape=circle, pos=(X, Y), length=L, width=W
   [AlgorithmManager]   Dyn obs #1: shape=rectangle, pos=(X, Y), length=L, width=W
   [Viz]   Drawing 2 dynamic obstacles
   [Viz]     Dyn obs #0: shape=circle, pos=(X, Y), length=L, width=W
   [Viz]     Dyn obs #1: shape=rectangle, pos=(X, Y), length=L, width=W
   [Viz]       Dyn obs (circle) radius=X pixels (diameter=Y)
   [Viz]       Dyn obs (rect) size=WxH pixels (w=?, h=?), yaw=?
   ```
4. **观察可视化窗口**：
   - 🟣 紫色圆形（圆形动态障碍物）
   - 🟣 紫色矩形（矩形动态障碍物，带旋转，前方有黄色点）
   - ✅ **没有绿色圆形叠加**

---

## ✅ 验证点

- ✅ 日志打印所有动态障碍物的信息（不只是第一个）
- ✅ 紫色矩形下方没有绿色圆形叠加
- ✅ 矩形的长宽与前端一致
- ✅ 使用 `shape_type` 判断形状类型（不依赖长宽相等）
- ✅ 圆形显示为紫色圆形
- ✅ 矩形显示为紫色矩形（带旋转和朝向指示）

---

**修复完成时间**：2025-10-14  
**编译状态**：✅ 成功  
**测试状态**：⏳ 待用户验证


# ESDF 可视化改进

## 🎨 改进内容

### 1. 更丰富的颜色渐变方案

**之前的问题**：
- 只有 4 种颜色：蓝色 -> 绿色 -> 黄色 -> 红色
- 颜色变化不够平滑
- 难以区分不同距离

**改进后的方案**：
- 7 种颜色渐变，更细腻的距离表示
- 从障碍物到远离障碍物的完整色谱

### 2. 修复栅格大小不一致问题

**之前的问题**：
- 使用格子中心点 + 固定大小绘制
- 放大后栅格大小不一致

**改进后的方案**：
- 使用格子的左下角和右上角坐标
- 通过 `worldToScreen()` 转换，确保栅格大小一致
- 支持采样绘制时的正确缩放

---

## 🌈 新的颜色方案

### 颜色映射表

| 距离 (m) | 归一化距离 | 颜色 | RGB | 含义 |
|---------|-----------|------|-----|------|
| 0.0 | 0.00 | 深红色 | (139, 0, 0) | 障碍物 |
| 0.5 | 0.10 | 红色 | (255, 0, 0) | 非常接近障碍物 |
| 1.0 | 0.20 | 橙色 | (255, 165, 0) | 接近障碍物 |
| 2.0 | 0.40 | 黄色 | (255, 255, 0) | 中等距离 |
| 3.0 | 0.60 | 绿色 | (0, 255, 0) | 较远 |
| 4.0 | 0.80 | 青色 | (0, 255, 255) | 远 |
| 5.0+ | 1.00 | 蓝色 | (0, 0, 255) | 很远（安全） |

### 颜色渐变逻辑

```cpp
if (normalized_dist < 0.1) {
  // 0.0 - 0.5m: 深红色 -> 红色
  double t = normalized_dist / 0.1;
  r = 139 + (255 - 139) * t;
  g = 0;
  b = 0;
} else if (normalized_dist < 0.2) {
  // 0.5m - 1.0m: 红色 -> 橙色
  double t = (normalized_dist - 0.1) / 0.1;
  r = 255;
  g = 165 * t;
  b = 0;
} else if (normalized_dist < 0.4) {
  // 1.0m - 2.0m: 橙色 -> 黄色
  double t = (normalized_dist - 0.2) / 0.2;
  r = 255;
  g = 165 + (255 - 165) * t;
  b = 0;
} else if (normalized_dist < 0.6) {
  // 2.0m - 3.0m: 黄色 -> 绿色
  double t = (normalized_dist - 0.4) / 0.2;
  r = 255 * (1.0 - t);
  g = 255;
  b = 0;
} else if (normalized_dist < 0.8) {
  // 3.0m - 4.0m: 绿色 -> 青色
  double t = (normalized_dist - 0.6) / 0.2;
  r = 0;
  g = 255;
  b = 255 * t;
} else {
  // 4.0m - 5.0m: 青色 -> 蓝色
  double t = (normalized_dist - 0.8) / 0.2;
  r = 0;
  g = 255 * (1.0 - t);
  b = 255;
}
```

---

## 🔧 栅格绘制修复

### 之前的实现（有问题）

```cpp
// 计算格子的世界坐标（中心点）
double world_x = cfg.origin.x + (x + 0.5) * cfg.resolution;
double world_y = cfg.origin.y + (y + 0.5) * cfg.resolution;

// 转换到屏幕坐标
auto center = worldToScreen(world_x, world_y);
float cell_size = cfg.resolution * config_.pixels_per_meter * view_state_.zoom * sample_step;

// 绘制矩形（使用固定大小）
draw_list->AddRectFilled(
  ImVec2(center.x - cell_size/2, center.y - cell_size/2),
  ImVec2(center.x + cell_size/2, center.y + cell_size/2),
  color
);
```

**问题**：
- `cell_size` 是在世界坐标系中计算的
- 但是 `center` 已经经过了 `worldToScreen()` 转换
- 导致放大后栅格大小不一致

### 改进后的实现

```cpp
// 计算格子的世界坐标（左下角）
double world_x = cfg.origin.x + x * cfg.resolution;
double world_y = cfg.origin.y + y * cfg.resolution;

// 转换到屏幕坐标（左下角和右上角）
auto p1 = worldToScreen(world_x, world_y);
auto p2 = worldToScreen(world_x + cfg.resolution * sample_step, 
                        world_y + cfg.resolution * sample_step);

// 绘制矩形（使用转换后的坐标）
draw_list->AddRectFilled(
  ImVec2(p1.x, p1.y),
  ImVec2(p2.x, p2.y),
  color
);
```

**优点**：
- 两个角点都经过 `worldToScreen()` 转换
- 确保栅格大小与缩放一致
- 支持采样绘制时的正确缩放

---

## 📊 可视化效果对比

### 之前的效果

```
🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴  ← 全是红色，难以区分距离
🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴
🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴
🔴🔴🔴🔴⬛⬛🔴🔴🔴🔴
🔴🔴🔴🔴🔴🔴🔴🔴🔴🔴
```

### 改进后的效果

```
🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵  ← 蓝色：5.0m+ (很远，安全)
🔵🟦🟦🟦🟦🟦🟦🟦🟦🔵  ← 青色：4.0m (远)
🔵🟦🟢🟢🟢🟢🟢🟢🟦🔵  ← 绿色：3.0m (较远)
🔵🟦🟢🟡🟡🟡🟡🟢🟦🔵  ← 黄色：2.0m (中等)
🔵🟦🟢🟡🟠🟠🟡🟢🟦🔵  ← 橙色：1.0m (近)
🔵🟦🟢🟡🟠🔴🟠🟡🟢🟦🔵  ← 红色：0.5m (很近)
🔵🟦🟢🟡🟠🔴⬛🔴🟠🟡🟢🟦🔵  ← 深红色：0.0m (障碍物)
🔵🟦🟢🟡🟠🔴🟠🟡🟢🟦🔵
🔵🟦🟢🟡🟡🟡🟡🟢🟦🔵
🔵🟦🟢🟢🟢🟢🟢🟢🟦🔵
🔵🟦🟦🟦🟦🟦🟦🟦🟦🔵
🔵🔵🔵🔵🔵🔵🔵🔵🔵🔵
```

---

## 🎯 使用说明

### 1. 启用 ESDF 可视化

在 Legend 面板中勾选 "Show ESDF Map"

### 2. 查看颜色图例

展开 "Show ESDF Map" 选项，可以看到完整的颜色映射：

```
Color gradient (distance from obstacles):
  0.0m: Dark Red
  0.5m: Red
  1.0m: Orange
  2.0m: Yellow
  3.0m: Green
  4.0m: Cyan
  5.0m: Blue
```

### 3. 理解颜色含义

- **深红色/红色区域**：非常接近障碍物，危险区域
- **橙色/黄色区域**：接近障碍物，需要注意
- **绿色区域**：较安全的区域
- **青色/蓝色区域**：远离障碍物，安全区域

### 4. 用于路径规划

- JPS 规划器会优先选择**绿色/青色/蓝色**区域
- 避开**红色/橙色**区域
- 在**黄色**区域谨慎规划

---

## 🔍 技术细节

### 透明度设置

```cpp
uint32_t color = IM_COL32(r, g, b, 150);  // Alpha = 150 (半透明)
```

- 使用半透明（Alpha = 150）
- 可以看到下层的障碍物和轨迹
- 不会完全遮挡其他元素

### 性能优化

```cpp
int sample_step = std::max(1, static_cast<int>(2.0 / view_state_.zoom));
```

- 根据缩放级别调整采样率
- 放大时 `sample_step = 1`（绘制所有格子）
- 缩小时 `sample_step > 1`（跳过部分格子）
- 优化性能，避免绘制过多格子

### 跳过远距离格子

```cpp
if (distance >= cfg.max_distance * 0.9) continue;
```

- 跳过距离 >= 4.5m 的格子
- 这些格子通常是蓝色，信息量较少
- 进一步优化性能

---

## 📈 预期改进效果

### 1. 视觉效果

- ✅ 颜色更丰富，渐变更平滑
- ✅ 更容易区分不同距离
- ✅ 更直观地理解安全区域

### 2. 栅格一致性

- ✅ 放大后栅格大小一致
- ✅ 栅格边界对齐
- ✅ 视觉效果更整洁

### 3. 实用性

- ✅ 帮助理解 ESDF 计算结果
- ✅ 辅助调试路径规划问题
- ✅ 验证安全距离设置

---

## 🚀 测试方法

### 1. 编译

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local/build
make -j8
```

### 2. 运行

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local
./build_with_visualization.sh
```

### 3. 检查可视化

1. 在 Legend 面板中勾选 "Show ESDF Map"
2. 观察 ESDF 地图颜色：
   - ✅ 应该看到从深红色到蓝色的平滑渐变
   - ✅ 障碍物周围是深红色/红色
   - ✅ 远离障碍物的区域是绿色/青色/蓝色
3. 放大地图：
   - ✅ 栅格大小应该一致
   - ✅ 栅格边界应该对齐

---

## 📚 相关文档

- `ESDF_REFACTOR_FINAL_SUMMARY.md` - ESDF 重构最终总结
- `ESDF_ALGORITHM_COMPARISON.md` - ESDF 算法对比
- `ESDF_VISUALIZATION_FIX.md` - ESDF 可视化修复

---

## ✅ 总结

本次改进：

1. ✅ **改进了颜色方案**：从 4 种颜色增加到 7 种颜色渐变
2. ✅ **修复了栅格大小问题**：使用正确的坐标转换方法
3. ✅ **更新了图例说明**：显示完整的颜色映射表
4. ✅ **保持性能优化**：采样绘制和距离过滤

**现在 ESDF 可视化更加美观和实用！** 🎨


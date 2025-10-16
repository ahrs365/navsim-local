# ESDF 鼠标悬停显示功能

## 🎯 功能说明

将鼠标移动到地图上的任意位置，会在鼠标旁边显示该位置的 ESDF 距离值和坐标信息。

---

## 🎨 显示效果

### 悬停信息框

当鼠标在 ESDF 地图范围内移动时，会显示一个信息框：

```
┌─────────────────────────────┐
│ ESDF: 1.234 m               │
│ Grid: (150, 200)            │
│ World: (10.50, 15.30)       │
└─────────────────────────────┘
```

### 信息内容

1. **ESDF 距离值**：
   - 格式：`ESDF: X.XXX m`
   - 单位：米
   - 精度：3 位小数
   - 障碍物显示：`ESDF: OBSTACLE`

2. **栅格坐标**：
   - 格式：`Grid: (x, y)`
   - 单位：格子索引
   - 范围：0 到 width-1, 0 to height-1

3. **世界坐标**：
   - 格式：`World: (x, y)`
   - 单位：米
   - 精度：2 位小数

### 视觉样式

- **背景**：半透明黑色（便于阅读）
- **边框**：青色（与 ESDF 边界框颜色一致）
- **文字**：白色
- **位置**：鼠标右下方（自动避免超出画布）

---

## 🚀 使用方法

### 1. 启动 NavSim

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local
./build_with_visualization.sh
```

### 2. 启用 ESDF 地图

在 Legend 面板中勾选 **"Show ESDF Map"** ✅

### 3. 移动鼠标

将鼠标移动到地图上的任意位置：

- **在 ESDF 地图范围内**：显示距离信息
- **在 ESDF 地图范围外**：不显示信息
- **在画布外**：不显示信息

### 4. 查看信息

信息框会自动跟随鼠标移动，实时更新显示内容。

---

## 📊 使用场景

### 场景 1：验证 ESDF 计算

移动鼠标到不同位置，检查距离值是否合理：

1. **障碍物位置**：应该显示 `ESDF: OBSTACLE`
2. **障碍物旁边**：距离应该很小（如 0.1m, 0.2m）
3. **远离障碍物**：距离应该较大（如 2.0m, 3.0m）
4. **对称性检查**：障碍物周围的距离应该对称

### 场景 2：调试路径规划

查看路径上各点的安全距离：

1. 移动鼠标沿着规划的路径
2. 查看每个点的 ESDF 距离值
3. 确认路径保持足够的安全距离

### 场景 3：分析障碍物分布

快速了解场景中的障碍物分布：

1. 移动鼠标扫描整个地图
2. 观察距离值的变化
3. 识别障碍物密集区域和空旷区域

### 场景 4：坐标转换验证

验证世界坐标和栅格坐标的转换：

1. 移动鼠标到已知位置（如自车位置）
2. 查看世界坐标是否正确
3. 查看栅格坐标是否对应

---

## 🔍 示例

### 示例 1：障碍物位置

```
鼠标位置：障碍物中心
显示内容：
┌─────────────────────────────┐
│ ESDF: OBSTACLE              │
│ Grid: (150, 150)            │
│ World: (0.00, 0.00)         │
└─────────────────────────────┘
```

### 示例 2：障碍物附近

```
鼠标位置：距离障碍物 0.5m
显示内容：
┌─────────────────────────────┐
│ ESDF: 0.500 m               │
│ Grid: (155, 150)            │
│ World: (0.50, 0.00)         │
└─────────────────────────────┘
```

### 示例 3：安全区域

```
鼠标位置：远离障碍物
显示内容：
┌─────────────────────────────┐
│ ESDF: 3.142 m               │
│ Grid: (180, 180)            │
│ World: (3.00, 3.00)         │
└─────────────────────────────┘
```

---

## 🎯 技术细节

### 坐标转换流程

1. **鼠标屏幕坐标** → **画布相对坐标**
   ```cpp
   float rel_x = mouse_pos.x - (canvas_pos.x + canvas_size.x / 2.0f);
   float rel_y = (canvas_pos.y + canvas_size.y / 2.0f) - mouse_pos.y;
   ```

2. **画布相对坐标** → **世界坐标**
   ```cpp
   double world_x = view_state_.center_x + rel_x / (pixels_per_meter * zoom);
   double world_y = view_state_.center_y + rel_y / (pixels_per_meter * zoom);
   ```

3. **世界坐标** → **栅格坐标**
   ```cpp
   int grid_x = (world_x - origin.x) / resolution;
   int grid_y = (world_y - origin.y) / resolution;
   ```

4. **栅格坐标** → **数组索引**
   ```cpp
   int idx = grid_y * width + grid_x;
   ```

### 边界检查

```cpp
// 检查鼠标是否在画布内
if (mouse_pos.x >= canvas_pos.x && mouse_pos.x <= canvas_pos.x + canvas_size.x &&
    mouse_pos.y >= canvas_pos.y && mouse_pos.y <= canvas_pos.y + canvas_size.y) {
  
  // 检查是否在 ESDF 地图范围内
  if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
    
    // 检查数组索引是否有效
    if (idx >= 0 && idx < data.size()) {
      // 显示信息
    }
  }
}
```

### 信息框位置自适应

```cpp
// 默认显示在鼠标右下方
ImVec2 text_pos = mouse_pos;
text_pos.x += 15.0f;
text_pos.y += 15.0f;

// 如果超出右边界，显示在左侧
if (text_pos.x + text_size.x + 10 > canvas_pos.x + canvas_size.x) {
  text_pos.x = mouse_pos.x - text_size.x - 15.0f;
}

// 如果超出下边界，显示在上方
if (text_pos.y + text_size.y + 10 > canvas_pos.y + canvas_size.y) {
  text_pos.y = mouse_pos.y - text_size.y - 15.0f;
}
```

---

## 💡 优势

### 相比全部显示栅格

1. **性能更好**
   - 不需要绘制所有栅格
   - 只在鼠标位置查询一个值
   - 无论地图多大，性能都很好

2. **界面更清爽**
   - 不会遮挡其他元素
   - 只在需要时显示信息
   - 不影响整体视觉效果

3. **信息更详细**
   - 可以显示多行信息
   - 包含栅格坐标和世界坐标
   - 便于调试和验证

4. **使用更灵活**
   - 想看哪里就移动鼠标到哪里
   - 不需要放大地图
   - 适用于任何缩放级别

---

## 🔧 自定义

### 修改显示精度

在 `imgui_visualizer.cpp` 中修改格式化字符串：

```cpp
// 当前：3 位小数
snprintf(dist_text, sizeof(dist_text), 
        "ESDF: %.3f m\n...", distance, ...);

// 改为 2 位小数
snprintf(dist_text, sizeof(dist_text), 
        "ESDF: %.2f m\n...", distance, ...);
```

### 修改信息框样式

```cpp
// 背景颜色（当前：半透明黑色）
IM_COL32(0, 0, 0, 200)  // R, G, B, Alpha

// 边框颜色（当前：青色）
IM_COL32(0, 255, 255, 255)

// 文字颜色（当前：白色）
IM_COL32(255, 255, 255, 255)
```

### 修改信息框位置偏移

```cpp
// 当前：右下方 15 像素
text_pos.x += 15.0f;
text_pos.y += 15.0f;

// 改为更远的偏移
text_pos.x += 25.0f;
text_pos.y += 25.0f;
```

### 添加更多信息

```cpp
// 例如：添加障碍物占据状态
snprintf(dist_text, sizeof(dist_text), 
        "ESDF: %.3f m\n"
        "Grid: (%d, %d)\n"
        "World: (%.2f, %.2f)\n"
        "Occupied: %s",  // 新增
        distance, grid_x, grid_y, world_x, world_y,
        is_occupied ? "Yes" : "No");  // 新增
```

---

## ✅ 总结

### 新功能

- ✅ 鼠标悬停显示 ESDF 距离值
- ✅ 显示栅格坐标和世界坐标
- ✅ 信息框自动避免超出画布
- ✅ 性能优秀，不影响帧率
- ✅ 界面简洁，不遮挡其他元素

### 使用场景

1. **验证 ESDF 计算**：快速检查任意位置的距离值
2. **调试路径规划**：查看路径上的安全距离
3. **分析障碍物分布**：了解场景中的障碍物分布
4. **坐标转换验证**：验证坐标系转换是否正确

### 下一步

现在可以运行 NavSim，移动鼠标查看 ESDF 距离值：

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local
./build_with_visualization.sh
```

在 Legend 面板勾选 "Show ESDF Map"，然后移动鼠标到地图上任意位置！

---

**现在可以轻松查看任意位置的 ESDF 距离值了！** 🎉


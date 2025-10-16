# ESDF 可视化调试指南

## 🐛 问题：看不到 ESDF 栅格和文本

如果你在可视化界面中看不到 ESDF 地图，请按照以下步骤排查：

---

## 📋 调试步骤

### 步骤 1：确认 ESDF 地图已启用

1. 打开 NavSim 可视化界面
2. 查看右侧的 **Legend** 面板
3. 确认 **"Show ESDF Map"** 选项已勾选 ✅

**如果没有勾选**：
- 勾选该选项
- ESDF 地图应该立即显示

---

### 步骤 2：查看控制台调试信息

运行 NavSim 后，查看控制台输出，应该看到类似信息：

#### 2.1 ESDF Builder 统计信息（每 60 帧）

```
[ESDFBuilder] ESDF stats:
  Occupied cells: 1234
  Distance < 0.5m:  5000 cells
  Distance 0.5-1m:  3000 cells
  Distance 1-2m:    1500 cells
  Distance 2-3m:    500 cells
  Distance > 3m:    100 cells
  Min distance:     0.0m
  Max distance:     4.5m
```

**如果看不到这个信息**：
- ESDF Builder 插件可能未启用
- 检查 `config/default.json` 中的 `ESDFBuilder` 配置

#### 2.2 ESDF 可视化信息（每 60 帧）

```
[Viz] Drawing ESDF map: 300x300 @0.1m, origin=(-15.0, -15.0), data_size=90000
[Viz]   ESDF cells: drawn=5000, skipped=85000, sample_step=1, zoom=1.0
```

**如果看到 "ESDF map enabled but esdf_map_ is null!"**：
- ESDF 数据未传递到可视化
- 检查 `esdf_builder_plugin.cpp` 中的 `context.esdf_map = std::move(esdf_map_navsim);`

**如果看到 "drawn=0"**：
- 所有格子都被跳过了
- 可能是因为所有距离都 >= max_distance * 0.9

---

### 步骤 3：检查 ESDF 配置

查看 `config/default.json`：

```json
{
  "name": "ESDFBuilder",
  "enabled": true,  // ← 确保是 true
  "priority": 90,
  "params": {
    "resolution": 0.1,
    "map_width": 30.0,
    "map_height": 30.0,
    "max_distance": 5.0,
    "include_dynamic": true
  }
}
```

**检查点**：
- ✅ `"enabled": true`
- ✅ `resolution` 合理（0.05 - 0.2）
- ✅ `map_width` 和 `map_height` 合理（10 - 50）
- ✅ `max_distance` 合理（3 - 10）

---

### 步骤 4：检查视图位置

ESDF 地图是以自车为中心的，确保：

1. **自车在视野内**
   - 可以看到自车（红色三角形）
   - ESDF 地图应该围绕自车

2. **缩放级别合适**
   - 尝试缩小视图（鼠标滚轮向下）
   - 应该能看到 ESDF 地图的青色边界框

3. **ESDF 边界框可见**
   - 青色虚线矩形框
   - 如果看不到，说明 ESDF 地图不在视野内

---

### 步骤 5：检查距离过滤

当前代码会跳过距离 >= max_distance * 0.9 的格子：

```cpp
if (distance >= cfg.max_distance * 0.9) continue;
```

**如果所有格子都被跳过**：
- 说明所有格子距离都 >= 4.5m（假设 max_distance = 5.0）
- 可能是因为没有障碍物

**解决方法**：
1. 临时注释掉这行代码
2. 或者降低阈值：`if (distance >= cfg.max_distance * 0.5) continue;`

---

### 步骤 6：检查障碍物

ESDF 地图需要障碍物才能显示有意义的距离场：

1. **查看 BEV 障碍物**
   - 在 Legend 面板勾选 "Show BEV Obstacles"
   - 应该能看到圆形和矩形障碍物

2. **查看占据栅格**
   - 在 Legend 面板勾选 "Show Occupancy Grid"
   - 应该能看到灰色的占据栅格

**如果没有障碍物**：
- ESDF 地图会全是蓝色（或被跳过）
- 需要等待场景中出现障碍物

---

### 步骤 7：检查文本显示条件

距离值文本只在格子足够大时显示：

```cpp
if (cell_width > 30.0f && cell_height > 20.0f) {
  // 绘制文本
}
```

**要看到文本**：
1. 放大地图（鼠标滚轮向上）
2. 直到每个栅格的屏幕尺寸 > 30×20 像素
3. 通常需要放大到只显示几十个栅格

**调试方法**：
- 临时降低阈值：`if (cell_width > 10.0f && cell_height > 10.0f)`
- 重新编译测试

---

## 🔍 常见问题诊断

### 问题 1：控制台显示 "ESDF map enabled but esdf_map_ is null!"

**原因**：ESDF 数据未传递到可视化

**检查**：
1. `esdf_builder_plugin.cpp` 中是否有 `context.esdf_map = std::move(esdf_map_navsim);`
2. ESDFBuilder 插件是否正确加载
3. 插件优先级是否正确

**解决**：
```cpp
// 在 esdf_builder_plugin.cpp 的 process() 函数末尾
context.esdf_map = std::move(esdf_map_navsim);
return true;  // 确保返回 true
```

---

### 问题 2：控制台显示 "drawn=0, skipped=90000"

**原因**：所有格子都被距离过滤跳过了

**检查**：
1. 查看 ESDF 统计信息中的距离分布
2. 如果大部分距离 > 4.5m，会被跳过

**解决**：
```cpp
// 临时修改过滤条件（在 imgui_visualizer.cpp 中）
// if (distance >= cfg.max_distance * 0.9) continue;
if (distance >= cfg.max_distance * 0.5) continue;  // 降低阈值
```

---

### 问题 3：看到边界框但看不到栅格

**原因**：栅格被绘制但颜色太淡或被其他元素遮挡

**检查**：
1. 调整透明度（当前 Alpha = 150）
2. 检查绘制顺序（ESDF 应该在 BEV 障碍物之前）

**解决**：
```cpp
// 增加不透明度（在 imgui_visualizer.cpp 中）
uint32_t color = IM_COL32(r, g, b, 200);  // 从 150 改为 200
```

---

### 问题 4：看到栅格但看不到文本

**原因**：格子不够大

**检查**：
1. 查看控制台的 `zoom` 值
2. 计算格子屏幕尺寸：`cell_size = resolution * pixels_per_meter * zoom`

**解决**：
1. 继续放大地图
2. 或者降低文本显示阈值：
```cpp
if (cell_width > 15.0f && cell_height > 10.0f) {  // 降低阈值
  // 绘制文本
}
```

---

## 🛠️ 临时调试修改

### 修改 1：移除距离过滤

在 `imgui_visualizer.cpp` 中注释掉：

```cpp
// 跳过距离太大的格子（优化性能）
// if (distance >= cfg.max_distance * 0.9) {
//   cells_skipped++;
//   continue;
// }
```

### 修改 2：降低文本显示阈值

```cpp
// 只有当格子足够大时才绘制文本（避免文字重叠）
if (cell_width > 10.0f && cell_height > 10.0f) {  // 从 30, 20 改为 10, 10
  // 绘制文本
}
```

### 修改 3：增加不透明度

```cpp
uint32_t color = IM_COL32(r, g, b, 220);  // 从 150 改为 220
```

### 修改 4：强制绘制所有格子

```cpp
int sample_step = 1;  // 强制 sample_step = 1，不跳过任何格子
```

---

## ✅ 验证清单

运行 NavSim 后，依次检查：

- [ ] 控制台显示 `[ESDFBuilder] ESDF stats:` 信息
- [ ] 控制台显示 `[Viz] Drawing ESDF map:` 信息
- [ ] `drawn` 数量 > 0
- [ ] Legend 面板中 "Show ESDF Map" 已勾选
- [ ] 可以看到青色虚线边界框
- [ ] 可以看到彩色的 ESDF 栅格
- [ ] 放大后可以看到距离值文本

---

## 📞 如果仍然无法显示

请提供以下信息：

1. **控制台完整输出**（包括 ESDF 相关的所有信息）
2. **配置文件**（`config/default.json` 中的 ESDFBuilder 部分）
3. **截图**（显示 Legend 面板和主视图）
4. **调试信息**：
   - `drawn` 和 `skipped` 的数量
   - ESDF 统计信息中的距离分布
   - 是否看到边界框

---

## 🚀 快速测试

运行以下命令，查看控制台输出：

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local
./build_with_visualization.sh 2>&1 | grep -E "\[ESDF|\[Viz\]"
```

应该看到：
```
[ESDFBuilder] ESDF stats: ...
[Viz] Drawing ESDF map: ...
[Viz]   ESDF cells: drawn=...
```

如果看不到这些信息，说明插件未正确运行。


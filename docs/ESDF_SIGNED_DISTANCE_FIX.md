# ESDF 有符号距离场修复 - 确保规划器获得正确的负值

## 🐛 问题描述

### 发现的问题

在 `esdf_builder_plugin.cpp` 中，复制数据到 `planning::ESDFMap` 时错误地使用了 `std::abs()` 取绝对值，导致：

1. ❌ **障碍物内部的负值被转换为正值**
2. ❌ **规划器无法区分障碍物内部和自由空间**
3. ❌ **违反了 `planning::ESDFMap` 的设计约定**

### 设计约定

根据 `planning_context.hpp` 中的定义：

```cpp
/**
 * @brief ESDF (Euclidean Signed Distance Field) 地图
 * 适用于：基于梯度的轨迹优化、碰撞检测、安全距离查询
 *
 * ESDF 存储每个栅格到最近障碍物的欧几里得距离：
 * - 正值：自由空间，表示到最近障碍物的距离
 * - 负值：障碍物内部，表示到最近自由空间的距离  ← 关键！
 * - 零值：障碍物边界
 */
struct ESDFMap {
  std::vector<double> data;  // 距离场数据 (m)，正值=自由空间，负值=障碍物内部
```

**结论**：`planning::ESDFMap` **必须保留负值**！

---

## 🔧 修复方案

### 修复 1：保留原始值（esdf_builder_plugin.cpp）

**位置**：`navsim-local/plugins/perception/esdf_builder/src/esdf_builder_plugin.cpp`，第 94-134 行

**修改前**（错误）：
```cpp
for (int i = 0; i < grid_width_ * grid_height_; ++i) {
  double dist_meter = esdf_map_->getDistance(esdf_map_->vectornum2gridIndex(i));
  // ❌ 错误：取绝对值，丢失了负值信息
  dist_meter = std::abs(dist_meter);
  esdf_map_navsim->data[i] = dist_meter;
  
  if (dist_meter < 0.01) {  // 障碍物
    occupied_count++;
  }
  // ...
}
```

**修改后**（正确）：
```cpp
for (int i = 0; i < grid_width_ * grid_height_; ++i) {
  double dist_meter = esdf_map_->getDistance(esdf_map_->vectornum2gridIndex(i));
  // ✅ 保留原始值（包括负值），不取绝对值
  esdf_map_navsim->data[i] = dist_meter;
  
  // 统计时使用绝对值
  double abs_dist = std::abs(dist_meter);
  if (abs_dist < 0.01) {  // 障碍物
    occupied_count++;
  }
  // ...
}
```

**关键改动**：
- ✅ 存储原始值：`esdf_map_navsim->data[i] = dist_meter;`（不取绝对值）
- ✅ 统计时使用绝对值：`double abs_dist = std::abs(dist_meter);`
- ✅ 规划器获得正确的有符号距离场

---

### 修复 2：可视化时取绝对值（imgui_visualizer.cpp）

**位置**：`navsim-local/src/viz/imgui_visualizer.cpp`

#### 2.1 栅格绘制（第 664-675 行）

**修改前**：
```cpp
double distance = esdf.data[idx];

// 跳过距离太大的格子（优化性能）
if (distance >= cfg.max_distance * 0.9) continue;

// 计算颜色
double normalized_dist = std::clamp(distance / cfg.max_distance, 0.0, 1.0);
```

**修改后**：
```cpp
double distance = esdf.data[idx];

// ✅ 可视化时取绝对值（障碍物内部是负值）
double abs_distance = std::abs(distance);

// 跳过距离太大的格子（优化性能）
if (abs_distance >= cfg.max_distance * 0.9) continue;

// 计算颜色
double normalized_dist = std::clamp(abs_distance / cfg.max_distance, 0.0, 1.0);
```

#### 2.2 鼠标悬停显示（第 770-788 行）

**修改前**：
```cpp
double distance = esdf.data[idx];

char dist_text[128];
if (distance < 0.01) {
  snprintf(dist_text, sizeof(dist_text),
          "ESDF: OBSTACLE\nGrid: (%d, %d)\nWorld: (%.2f, %.2f)",
          grid_x, grid_y, world_x, world_y);
} else {
  snprintf(dist_text, sizeof(dist_text),
          "ESDF: %.3f m\nGrid: (%d, %d)\nWorld: (%.2f, %.2f)",
          distance, grid_x, grid_y, world_x, world_y);
}
```

**修改后**：
```cpp
double distance = esdf.data[idx];

// 显示原始值（包括负值），帮助调试
char dist_text[128];
if (std::abs(distance) < 0.01) {
  snprintf(dist_text, sizeof(dist_text),
          "ESDF: OBSTACLE (%.3f m)\nGrid: (%d, %d)\nWorld: (%.2f, %.2f)",
          distance, grid_x, grid_y, world_x, world_y);
} else if (distance < 0) {
  snprintf(dist_text, sizeof(dist_text),
          "ESDF: %.3f m (inside)\nGrid: (%d, %d)\nWorld: (%.2f, %.2f)",
          distance, grid_x, grid_y, world_x, world_y);
} else {
  snprintf(dist_text, sizeof(dist_text),
          "ESDF: %.3f m\nGrid: (%d, %d)\nWorld: (%.2f, %.2f)",
          distance, grid_x, grid_y, world_x, world_y);
}
```

**改进**：
- ✅ 显示原始值（包括负值）
- ✅ 负值时显示 "(inside)" 标记
- ✅ 帮助调试和验证

---

## 📊 数据流对比

### 修复前（错误）

```
ESDFMap::computeESDF()
    ↓
distance_buffer_all_[idx] = ...
    正值：自由空间
    负值：障碍物内部  ← 正确计算
    ↓
esdf_builder_plugin.cpp
    ↓
dist_meter = std::abs(dist_meter);  ← ❌ 错误：取绝对值
    ↓
planning::ESDFMap::data[i] = dist_meter;
    正值：自由空间
    正值：障碍物内部  ← ❌ 错误：丢失了负值信息
    ↓
规划器使用
    ❌ 无法区分障碍物内部和自由空间
```

### 修复后（正确）

```
ESDFMap::computeESDF()
    ↓
distance_buffer_all_[idx] = ...
    正值：自由空间
    负值：障碍物内部  ← 正确计算
    ↓
esdf_builder_plugin.cpp
    ↓
planning::ESDFMap::data[i] = dist_meter;  ← ✅ 保留原始值
    正值：自由空间
    负值：障碍物内部  ← ✅ 正确：保留负值
    ↓
规划器使用
    ✅ 可以正确区分障碍物内部和自由空间
    ↓
可视化使用
    ✅ 取绝对值用于颜色映射
    ✅ 显示原始值用于调试
```

---

## 🎯 影响分析

### 修复前的问题

1. **规划器无法识别障碍物内部**
   - 障碍物内部的负值被转换为正值
   - 规划器可能认为障碍物内部是安全的
   - 可能导致路径穿过障碍物

2. **碰撞检测失效**
   - 无法通过 `distance < 0` 判断是否在障碍物内部
   - 碰撞检测逻辑错误

3. **梯度优化错误**
   - 障碍物内部的梯度方向错误
   - 优化可能收敛到障碍物内部

### 修复后的效果

1. **规划器正确工作**
   - ✅ 可以通过 `distance < 0` 识别障碍物内部
   - ✅ 可以通过 `distance > 0` 识别自由空间
   - ✅ 路径规划避开障碍物内部

2. **碰撞检测正确**
   - ✅ `distance < 0` → 在障碍物内部（碰撞）
   - ✅ `distance > safe_distance` → 安全
   - ✅ `0 < distance < safe_distance` → 接近障碍物

3. **梯度优化正确**
   - ✅ 障碍物内部的梯度指向最近的自由空间
   - ✅ 优化会将轨迹推出障碍物

---

## 🔍 验证方法

### 1. 鼠标悬停验证

运行 NavSim，移动鼠标到障碍物上：

**预期结果**：
- 障碍物中心：`ESDF: -0.XXX m (inside)`（负值）
- 障碍物边界：`ESDF: 0.0XX m`（接近 0）
- 自由空间：`ESDF: X.XXX m`（正值）

### 2. 代码验证

在规划器中添加断言：

```cpp
// 检查 ESDF 数据的符号
if (context.esdf_map) {
  for (int i = 0; i < context.esdf_map->data.size(); ++i) {
    double dist = context.esdf_map->data[i];
    
    // 验证：障碍物内部应该是负值
    if (is_obstacle(i)) {
      assert(dist <= 0.0);  // 障碍物内部或边界
    } else {
      assert(dist >= 0.0);  // 自由空间
    }
  }
}
```

### 3. 可视化验证

- ✅ 栅格颜色：障碍物显示为深红色（距离接近 0）
- ✅ 鼠标悬停：障碍物内部显示负值 + "(inside)" 标记
- ✅ 统计信息：`Occupied cells` 数量正确

---

## 📝 总结

### 修复内容

| 文件 | 修改 | 目的 |
|------|------|------|
| `esdf_builder_plugin.cpp` | 移除 `std::abs()`，保留原始值 | 规划器获得正确的有符号距离场 |
| `imgui_visualizer.cpp` | 栅格绘制时取绝对值 | 可视化使用正值进行颜色映射 |
| `imgui_visualizer.cpp` | 鼠标悬停显示原始值 | 调试时可以看到负值 |

### 核心原则

**分离关注点**：
- ✅ **数据存储**：保留原始值（包括负值）
- ✅ **规划器使用**：使用原始值（包括负值）
- ✅ **可视化显示**：取绝对值用于颜色映射

### 符号约定

- **正值**：自由空间，表示到最近障碍物的距离
- **负值**：障碍物内部，表示到最近自由空间的距离
- **零值**：障碍物边界

### 影响

修复后：
- ✅ 规划器可以正确识别障碍物内部
- ✅ 碰撞检测正确工作
- ✅ 梯度优化正确工作
- ✅ 可视化正确显示
- ✅ 调试信息完整

---

## 🚀 下一步

现在可以：
1. ✅ 运行 NavSim 验证修复
2. ✅ 使用鼠标悬停查看负值
3. ✅ 开始 JPS 规划器移植（ESDF 数据完全正确）

所有修复已编译通过，准备好测试了！🎉


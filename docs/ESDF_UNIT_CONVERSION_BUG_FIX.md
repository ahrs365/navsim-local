# ESDF 单位转换 Bug 修复

## 🐛 问题描述

### 症状

鼠标悬停显示的 ESDF 距离值与实际距离不匹配，存在 **10 倍缩小** 的错误：

- **实际距离**：约 1.0 米
- **显示距离**：约 0.1 米
- **倍数关系**：缩小了 10 倍（分辨率 = 0.1m）

### 影响范围

这个 bug 影响了以下功能：

1. ❌ **可视化显示**：鼠标悬停显示的距离值错误
2. ❌ **碰撞检测**：`isOccWithSafeDis()` 函数判断错误
3. ❌ **距离查询**：`getDistanceReal()` 函数返回值错误
4. ⚠️ **路径规划**：如果 JPS 规划器使用上述函数，会导致规划错误

---

## 🔍 根本原因分析

### 数据流追踪

#### 1. ESDF 计算（正确）✅

在 `esdf_map.cpp` 的 `computeESDF()` 函数中：

```cpp
// Y 方向扫描（第 92 行）
fillESDF(
  [&](int x) { return tmp_buffer1[x * GLY_SIZE_ + y]; },
  [&](int x, double val) {
    distance_buffer_pos[x * GLY_SIZE_ + y] = grid_interval_ * std::sqrt(val);
    //                                        ^^^^^^^^^^^^^^
    //                                        已经乘以分辨率，转换为米单位
  },
  0,
  GLX_SIZE_ - 1,
  GLX_SIZE_
);
```

**结论**：`distance_buffer_all_` 存储的是 **米单位** 的距离值。

#### 2. 距离查询（正确）✅

在 `esdf_map.hpp` 的 `getDistance()` 函数中：

```cpp
inline double ESDFMap::getDistance(const Eigen::Vector2i& id) const {
  if (!isValidIndex(id)) return 0.0;
  return distance_buffer_all_[Index2Vectornum(id)];
  //     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //     直接返回，单位是米
}
```

**结论**：`getDistance()` 返回的是 **米单位** 的距离值。

#### 3. 数据复制（错误）❌

在 `esdf_builder_plugin.cpp` 的 `process()` 函数中（第 111 行）：

```cpp
double dist_grid = esdf_map_->getDistance(esdf_map_->vectornum2gridIndex(i));
//     ^^^^^^^^^ 已经是米单位

double dist_meter = std::abs(dist_grid) * resolution_;  // ❌ 错误！
//                                        ^^^^^^^^^^^
//                                        又乘以分辨率（0.1），导致缩小 10 倍
```

**问题**：`getDistance()` 返回的已经是米单位，不应该再乘以 `resolution_`。

---

## 🔧 修复方案

### 修复 1：数据复制（esdf_builder_plugin.cpp）

**位置**：`navsim-local/plugins/perception/esdf_builder/src/esdf_builder_plugin.cpp`，第 103-130 行

**修改前**：
```cpp
for (int i = 0; i < grid_width_ * grid_height_; ++i) {
  double dist_grid = esdf_map_->getDistance(esdf_map_->vectornum2gridIndex(i));
  double dist_meter = std::abs(dist_grid) * resolution_;  // ❌ 错误
  esdf_map_navsim->data[i] = dist_meter;
  // ...
}
```

**修改后**：
```cpp
for (int i = 0; i < grid_width_ * grid_height_; ++i) {
  double dist_meter = esdf_map_->getDistance(esdf_map_->vectornum2gridIndex(i));
  // ✅ FIX: getDistance() 已经返回米单位，不需要再乘以 resolution_
  // 只需要取绝对值（因为障碍物内部是负值）
  dist_meter = std::abs(dist_meter);
  esdf_map_navsim->data[i] = dist_meter;
  // ...
}
```

**关键改动**：
- 移除了 `* resolution_`
- 直接使用 `getDistance()` 的返回值
- 只对负值取绝对值

---

### 修复 2：安全距离检查（esdf_map.hpp）

**位置**：`navsim-local/plugins/perception/esdf_builder/include/esdf_map.hpp`，第 281-291 行

**修改前**：
```cpp
inline bool ESDFMap::isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis) const {
  if (!isValidIndex(index)) return true;
  return getDistance(index) < safe_dis / grid_interval_;  // ❌ 错误
  //                                      ^^^^^^^^^^^^^^
  //                                      不应该除以 grid_interval_
}
```

**修改后**：
```cpp
inline bool ESDFMap::isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis) const {
  if (!isValidIndex(index)) return true;
  // ✅ FIX: getDistance() 返回米单位，safe_dis 也是米单位，直接比较
  return getDistance(index) < safe_dis;
}
```

**关键改动**：
- 移除了 `/ grid_interval_`
- `getDistance()` 和 `safe_dis` 都是米单位，直接比较

---

### 修复 3：世界坐标距离查询（esdf_map.hpp）

**位置**：`navsim-local/plugins/perception/esdf_builder/include/esdf_map.hpp`，第 303-308 行

**修改前**：
```cpp
inline double ESDFMap::getDistanceReal(const Eigen::Vector2d& pos) const {
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos);
  if (!isValidIndex(idx)) return 0.0;
  return getDistance(idx) * grid_interval_;  // ❌ 错误
  //                        ^^^^^^^^^^^^^^
  //                        不应该乘以 grid_interval_
}
```

**修改后**：
```cpp
inline double ESDFMap::getDistanceReal(const Eigen::Vector2d& pos) const {
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos);
  if (!isValidIndex(idx)) return 0.0;
  // ✅ FIX: getDistance() 已经返回米单位，不需要再乘以 grid_interval_
  return getDistance(idx);
}
```

**关键改动**：
- 移除了 `* grid_interval_`
- `getDistance()` 已经返回米单位

---

## 📊 原始实现验证

### SDFmap 的实现（参考）

为了确认修复的正确性，我们检查了原始 `sdf_map.cpp` 的实现：

#### 1. getDistance() 实现

```cpp
// sdf_map.cpp, 第 721 行
inline double SDFmap::getDistance(const Eigen::Vector2i& id){
  return distance_buffer_all_[Index2Vectornum(id[0],id[1])];
  //     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //     直接返回，单位是米
}
```

#### 2. getDistanceReal() 实现

```cpp
// sdf_map.cpp, 第 848 行
double SDFmap::getDistanceReal(const Eigen::Vector2d& pos){
  Eigen::Vector2i idx = coord2gridIndex(pos);
  return distance_buffer_all_[idx.x() * GLY_SIZE_ + idx.y()];
  //     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //     直接返回，单位是米，不乘以 grid_interval_
}
```

#### 3. isOccWithSafeDis() 实现

```cpp
// sdf_map.cpp, 第 921 行
bool SDFmap::isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis){
  return distance_buffer_all_[Index2Vectornum(index)] < safe_dis;
  //                                                     ^^^^^^^^
  //                                                     直接比较，不除以 grid_interval_
}
```

#### 4. 可视化发布

```cpp
// sdf_map.cpp, 第 706 行
pt.intensity = std::max(min_dist, std::min(distance_buffer_all_[i], max_dist));
//                                          ^^^^^^^^^^^^^^^^^^^^
//                                          直接使用，单位是米
```

**结论**：原始实现确认了 `distance_buffer_all_` 存储的是米单位，所有访问函数都直接返回，不需要额外的单位转换。

---

## ✅ 验证方法

### 1. 单元测试

运行 `test_esdf_map` 验证基本功能：

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local/build
./test_esdf_map
```

**预期输出**：
```
Obstacle at (10, 10): distance = 0 (should be 0.0) ✅
Neighbor at (11, 10): distance = 0.1 (should be ~0.1m) ✅
Diagonal neighbor at (11, 11): distance = 0.141421 (should be ~0.14m) ✅
5 cells away at (15, 10): distance = 0.5 (should be ~0.5m) ✅
```

### 2. 可视化验证

运行 NavSim，使用鼠标悬停查看距离值：

```bash
cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local
./build_with_visualization.sh
```

**验证步骤**：
1. 在 Legend 面板勾选 "Show ESDF Map"
2. 将鼠标移动到障碍物附近
3. 查看显示的距离值是否合理

**预期结果**：
- 障碍物位置：`ESDF: OBSTACLE`
- 距离障碍物 1 格（0.1m）：`ESDF: 0.100 m`
- 距离障碍物 10 格（1.0m）：`ESDF: 1.000 m`

### 3. 手动测试

在已知位置测试：

1. **自车位置**：应该距离障碍物较远（> 1m）
2. **障碍物边缘**：应该显示很小的距离（< 0.2m）
3. **空旷区域**：应该显示较大的距离（> 2m）

---

## 📝 总结

### 修复的 Bug

1. ✅ **数据复制**：移除了错误的 `* resolution_` 操作
2. ✅ **安全距离检查**：移除了错误的 `/ grid_interval_` 操作
3. ✅ **世界坐标查询**：移除了错误的 `* grid_interval_` 操作

### 核心原则

**单位一致性原则**：

- `distance_buffer_all_` 存储的是 **米单位**
- `getDistance()` 返回的是 **米单位**
- `getDistanceReal()` 返回的是 **米单位**
- `safe_dis` 参数是 **米单位**
- 所有距离相关的操作都应该使用 **米单位**，不需要额外转换

### 未修改的部分

✅ **核心 ESDF 算法**：
- `fillESDF()` 函数：算法逻辑完全不变
- `computeESDF()` 函数：算法逻辑完全不变
- 只修复了单位转换错误，不影响算法正确性

### 影响

修复后，以下功能将正常工作：

1. ✅ **可视化显示**：鼠标悬停显示正确的距离值
2. ✅ **碰撞检测**：`isOccWithSafeDis()` 正确判断安全距离
3. ✅ **距离查询**：`getDistanceReal()` 返回正确的距离值
4. ✅ **路径规划**：JPS 规划器可以正确使用 ESDF 数据

---

## 🚀 下一步

现在可以：

1. **运行测试**：验证修复是否正确
2. **可视化验证**：使用鼠标悬停查看距离值
3. **开始 JPS 移植**：ESDF 数据现在是正确的，可以安全使用

所有修复已经编译通过，准备好测试了！🎉


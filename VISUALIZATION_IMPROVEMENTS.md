# 轨迹可视化改进说明

## 📋 改进内容

### **1. 优化起点标记**

**问题**: 绿色大圆点起点标记遮挡了下方的轨迹

**解决方案**:
- ❌ 移除大圆点起点标记 `'go', markersize=12`
- ✅ 使用小三角形标记起点 `'g^', markersize=8`
- ✅ 保留红色圆点终点标记

**效果**:
```python
# 修改前
ax1.plot(trajectory['x'][0], trajectory['y'][0], 'go', markersize=12, 
         label='Start', markeredgecolor='black', markeredgewidth=2)

# 修改后
ax1.plot(trajectory['x'][0], trajectory['y'][0], 'g^', markersize=8, 
         label='Start', markeredgecolor='black', markeredgewidth=1)
```

---

### **2. 添加运动学限制线**

**功能**: 在速度、角速度、加速度曲线上绘制对应的运动学限制线

#### **速度曲线 (Chart 2)**

添加限制线：
- `max_vel`: 最大速度（红色虚线）
- `min_vel`: 最小速度（红色虚线）

```python
max_vel = meta.get('max_vel', meta.get('max_velocity', None))
min_vel = meta.get('min_vel', None)
if max_vel is not None:
    ax2.axhline(y=max_vel, color='r', linestyle='--', linewidth=2, alpha=0.7, 
                label=f'Max V: {max_vel:.2f} m/s')
if min_vel is not None:
    ax2.axhline(y=min_vel, color='r', linestyle='--', linewidth=2, alpha=0.7, 
                label=f'Min V: {min_vel:.2f} m/s')
```

#### **角速度曲线 (Chart 3)**

添加限制线：
- `max_omega`: 最大角速度（红色虚线）
- `-max_omega`: 最小角速度（红色虚线）

```python
max_omega = meta.get('max_omega', None)
if max_omega is not None:
    ax3.axhline(y=max_omega, color='r', linestyle='--', linewidth=2, alpha=0.7, 
                label=f'Max ω: {max_omega:.2f} rad/s')
    ax3.axhline(y=-max_omega, color='r', linestyle='--', linewidth=2, alpha=0.7, 
                label=f'Min ω: {-max_omega:.2f} rad/s')
```

#### **加速度曲线 (Chart 4)**

添加限制线：
- `max_acc`: 最大加速度（红色虚线）
- `-max_deceleration`: 最大减速度（红色虚线）

```python
max_acc = meta.get('max_acc', meta.get('max_acceleration', None))
max_dec = meta.get('max_deceleration', None)
if max_acc is not None:
    ax4.axhline(y=max_acc, color='r', linestyle='--', linewidth=2, alpha=0.7, 
                label=f'Max A: {max_acc:.2f} m/s²')
if max_dec is not None:
    ax4.axhline(y=-max_dec, color='r', linestyle='--', linewidth=2, alpha=0.7, 
                label=f'Max D: {max_dec:.2f} m/s²')
```

**效果**:
- ✅ 可以直观看到轨迹是否超过运动学限制
- ✅ 便于调试优化器参数
- ✅ 验证约束是否被正确执行

---

### **3. 高曲率区域画中画**

**功能**: 检测曲率很大的位置（可能是倒车或急转弯），创建画中画放大显示

#### **检测高曲率区域**

```python
curvature_threshold = 50.0  # 曲率阈值 (1/m)
high_curv_indices = np.where(np.abs(trajectory['curvature']) > curvature_threshold)[0]
```

#### **分组连续的高曲率点**

```python
high_curv_regions = []
if len(high_curv_indices) > 0:
    current_region = [high_curv_indices[0]]
    for i in range(1, len(high_curv_indices)):
        if high_curv_indices[i] - high_curv_indices[i-1] < 50:  # 间隔小于50个点认为是同一区域
            current_region.append(high_curv_indices[i])
        else:
            high_curv_regions.append(current_region)
            current_region = [high_curv_indices[i]]
    high_curv_regions.append(current_region)
```

#### **创建画中画**

对每个高曲率区域（最多3个）：

1. **选择中心点**: 取区域中心点作为画中画的焦点
2. **扩展范围**: 显示中心点前后各100个点（提供上下文）
3. **放置位置**: 在主图的空白区域放置画中画
   - 第1个: `[0.55, 0.55, 0.2, 0.2]` (右上)
   - 第2个: `[0.15, 0.55, 0.2, 0.2]` (左上)
   - 第3个: `[0.35, 0.35, 0.2, 0.2]` (中间)

4. **画中画内容**:
   - 轨迹线（蓝色）
   - 速度颜色映射（散点）
   - 朝向箭头（红色）
   - 高曲率点标记（红色星号）
   - 标题显示曲率值

5. **主图标记**: 在主图上用黄色边框的红色星号标记高曲率位置

```python
# 创建画中画
ax_inset = fig.add_axes(inset_pos)
ax_inset.plot(trajectory['x'][start_idx:end_idx], 
             trajectory['y'][start_idx:end_idx], 
             'b-', linewidth=2, alpha=0.8)
ax_inset.scatter(trajectory['x'][start_idx:end_idx], 
                trajectory['y'][start_idx:end_idx],
                c=trajectory['vx'][start_idx:end_idx], 
                cmap='viridis', s=20, alpha=0.6)

# 标记高曲率点
ax_inset.plot(trajectory['x'][center_idx], trajectory['y'][center_idx], 
             'r*', markersize=15, markeredgecolor='black', markeredgewidth=1)

# 绘制朝向箭头
for i in range(start_idx, end_idx, max(1, (end_idx - start_idx) // 10)):
    dx = 0.1 * np.cos(trajectory['yaw'][i])
    dy = 0.1 * np.sin(trajectory['yaw'][i])
    ax_inset.arrow(trajectory['x'][i], trajectory['y'][i], dx, dy,
                  head_width=0.05, head_length=0.03, fc='red', ec='red', alpha=0.7)

ax_inset.set_aspect('equal')
ax_inset.grid(True, alpha=0.3)
ax_inset.set_title(f'High Curvature Region {inset_idx+1}\nκ={trajectory["curvature"][center_idx]:.1f} 1/m', 
                  fontsize=8, fontweight='bold')
```

**效果**:
- ✅ 自动检测高曲率区域（倒车、急转弯等）
- ✅ 放大显示细节，便于分析
- ✅ 显示朝向箭头，清楚看到车辆运动方向
- ✅ 支持多个高曲率区域（最多3个）
- ✅ 在主图上标记对应位置

---

## 🎨 可视化效果对比

### **修改前**

- ❌ 绿色大圆点遮挡起点附近的轨迹
- ❌ 速度/加速度曲线没有限制线，难以判断是否超限
- ❌ 高曲率区域（倒车）难以看清细节

### **修改后**

- ✅ 小三角形标记起点，不遮挡轨迹
- ✅ 速度/角速度/加速度曲线显示限制线（红色虚线）
- ✅ 高曲率区域自动创建画中画放大显示
- ✅ 画中画显示朝向箭头和速度颜色映射
- ✅ 主图上标记高曲率位置

---

## 📊 使用示例

### **步骤 1: 生成轨迹日志**

```bash
cd navsim-local
./build/navsim_local_debug --scenario scenarios/map1.json --planner JpsPlanner --perception EsdfBuilder
```

### **步骤 2: 可视化轨迹**

```bash
# 保存为图片
python3 visualize_trajectory_save.py

# 或交互式查看
python3 visualize_trajectory.py
```

### **步骤 3: 查看结果**

打开 `minco_trajectory_visualization.png`，你会看到：

1. **XY 轨迹图**:
   - 小三角形标记起点（绿色）
   - 红色圆点标记终点
   - 速度颜色映射（蓝色=慢，黄色=快）
   - 朝向箭头（红色）
   - 高曲率位置标记（红色星号，黄色边框）
   - 画中画放大显示高曲率区域

2. **速度曲线**:
   - 蓝色实线：实际速度
   - 红色虚线：最大/最小速度限制

3. **角速度曲线**:
   - 绿色实线：实际角速度
   - 红色虚线：最大/最小角速度限制

4. **加速度曲线**:
   - 红色实线：实际加速度
   - 红色虚线：最大加速度/最大减速度限制

---

## 🔧 参数调整

### **曲率阈值**

如果需要调整高曲率检测的阈值，修改以下参数：

```python
curvature_threshold = 50.0  # 曲率阈值 (1/m)
```

- 增大阈值：只检测更极端的曲率（更少的画中画）
- 减小阈值：检测更多的高曲率区域（更多的画中画）

### **画中画数量**

```python
num_insets = min(len(high_curv_regions), 3)  # 最多显示3个画中画
```

可以修改为显示更多或更少的画中画。

### **画中画范围**

```python
start_idx = max(0, center_idx - 100)  # 中心点前100个点
end_idx = min(len(trajectory['x']), center_idx + 100)  # 中心点后100个点
```

可以调整范围以显示更多或更少的上下文。

---

## ✅ 完成！

所有改进已实现并测试通过：
- ✅ 起点标记优化（不遮挡轨迹）
- ✅ 运动学限制线（速度、角速度、加速度）
- ✅ 高曲率区域画中画（自动检测和放大）

**现在可以更清晰地分析轨迹，特别是倒车和急转弯等复杂行为！** 🎉


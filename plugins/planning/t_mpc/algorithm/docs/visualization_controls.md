# 可视化控制功能说明

## 📋 概述

MPC Planner 的可视化界面现在支持通过**可点击的图例**动态控制各个可视化元素的显示/隐藏。

**特点**：
- ✅ **直接点击图例**即可控制显示/隐藏
- ✅ **紧凑美观**，不占用额外空间
- ✅ **视觉反馈**，隐藏的元素图例会变暗（透明度 30%）
- ✅ **实时生效**，下一帧立即更新

---

## 🎨 界面布局

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MPC Planner Visualization                        │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌────────────────────────────────────────────────────────────────┐ │
│  │                                                                │ │
│  │         Main Visualization (主可视化窗口)                       │ │
│  │                                                                │ │
│  │                                          ┌──────────────────┐ │ │
│  │                                          │ 图例 (可点击)     │ │ │
│  │                                          │ ━━━━━━━━━━━━━━━ │ │ │
│  │                                          │ ● Reference Path │ │ │
│  │                                          │ ● Ego History    │ │ │
│  │                                          │ ● MPC Trajectory │ │ │
│  │                                          │ ● Guidance Path  │ │ │
│  │                                          │ ● Static Obs...  │ │ │
│  │                                          │ ● Dynamic Obs... │ │ │
│  │                                          │ ● Predictions    │ │ │
│  │                                          │ ● Robot Footp... │ │ │
│  │                                          │ ● Goal           │ │ │
│  │                                          └──────────────────┘ │ │
│  └────────────────────────────────────────────────────────────────┘ │
│                                                                     │
│  ┌──────────────────────┐  ┌──────────────────────┐               │
│  │  Planned Trajectory  │  │  History Plots       │               │
│  │  (规划轨迹图)         │  │  (历史数据图)         │               │
│  └──────────────────────┘  └──────────────────────┘               │
│                                                                     │
│                                    [Pause] [Start]                 │
└─────────────────────────────────────────────────────────────────────┘
```

---

## ✅ 可控制的可视化元素

### **1. Reference Path（参考路径）**
- **描述**: 黑色虚线，表示全局参考路径
- **默认状态**: 显示 ✓
- **颜色**: 黑色 (`k`)
- **线型**: 虚线 (`--`)

### **2. Ego History（自车历史轨迹）**
- **描述**: 蓝色实线，表示机器人已经走过的路径
- **默认状态**: 显示 ✓
- **颜色**: 蓝色 (`b`)
- **线型**: 实线 (`-`)

### **3. MPC Trajectories（MPC 轨迹）**
- **描述**: 多条彩色轨迹，表示 T-MPC 并行优化的候选轨迹
- **默认状态**: 显示 ✓
- **特点**:
  - 选中的轨迹：粗线（linewidth=3.0）
  - 候选轨迹：细线（linewidth=1.6）
  - 不同颜色表示不同的拓扑类别

### **4. Guidance Paths（引导路径）**
- **描述**: 虚线，表示全局引导规划器生成的引导轨迹
- **默认状态**: 显示 ✓
- **线型**: 虚线 (`--`)
- **颜色**: 根据拓扑类别自动分配

### **5. Static Obstacles（静态障碍物）**
- **描述**: 橙色圆圈，表示静态障碍物
- **默认状态**: 显示 ✓
- **颜色**: 橙色 (`#ff8c00`)
- **线宽**: 1.8

### **6. Dynamic Obstacles（动态障碍物）**
- **描述**: 红色圆圈，表示动态障碍物
- **默认状态**: 显示 ✓
- **颜色**: 红色 (`#d62728`)
- **线宽**: 1.4

### **7. Predictions（障碍物预测轨迹）**
- **描述**: 红色点线，表示动态障碍物的预测轨迹
- **默认状态**: 显示 ✓
- **颜色**: 红色 (`#d62728`)
- **线型**: 点线 (`:`)

### **8. Robot Footprint（机器人足迹）**
- **描述**: 蓝色圆圈，表示机器人的物理形状（多圆盘模型）
- **默认状态**: 显示 ✓
- **颜色**: 皇家蓝 (`#4169e1`)
- **线宽**: 2.2
- **包含**: 机器人中心点（深蓝色散点）

### **9. Goal（目标点）**
- **描述**: 金色散点，表示导航目标位置
- **默认状态**: 显示 ✓
- **颜色**: 金色 (`gold`)
- **大小**: 220

---

## 🎮 使用方法

### **启动程序**

```bash
cd build
./mpc_planner_main
```

### **控制可视化元素**

#### **方法：点击图例**

1. **隐藏元素**:
   - 在右上角的图例中，**点击**想要隐藏的元素名称
   - 图例文字会变暗（透明度降至 30%）
   - 下一帧该元素将不再显示

2. **显示元素**:
   - 再次**点击**已隐藏的元素名称
   - 图例文字恢复正常亮度
   - 下一帧该元素将重新显示

3. **视觉反馈**:
   - **正常显示**: 图例文字清晰（透明度 100%）
   - **已隐藏**: 图例文字变暗（透明度 30%）

#### **示例操作**

```
1. 点击 "MPC Trajectory" → MPC 轨迹隐藏，图例变暗
2. 点击 "Guidance Path" → 引导路径隐藏，图例变暗
3. 再次点击 "MPC Trajectory" → MPC 轨迹重新显示，图例恢复
```

### **暂停/继续仿真**

- **Pause 按钮**: 暂停仿真，可以仔细观察当前状态
- **Start 按钮**: 继续仿真

---

## 🔧 技术实现

### **核心数据结构**

```python
# Python 端（matplotlib）
plt._mpc_visibility = {
    'Reference Path': True,
    'Ego History': True,
    'MPC Trajectories': True,
    'Guidance Paths': True,
    'Static Obstacles': True,
    'Dynamic Obstacles': True,
    'Predictions': True,
    'Robot Footprint': True,
    'Goal': True
}
```

### **C++ 端检查可见性**

```cpp
bool MatplotlibVisualizer::isVisible(const std::string &element_name) const
{
    // 查询 Python 端的可见性状态
    std::string script =
        "import matplotlib.pyplot as plt\n"
        "result = plt._mpc_visibility.get('" + element_name + "', True)\n";
    PyRun_SimpleString(script.c_str());
    
    // 获取结果
    PyObject *main_module = PyImport_AddModule("__main__");
    PyObject *result = PyObject_GetAttrString(main_module, "result");
    bool visible = PyObject_IsTrue(result);
    Py_DECREF(result);
    return visible;
}
```

### **绘制前检查**

```cpp
// 示例：绘制参考路径前检查可见性
if (!_ref_x.empty() && isVisible("Reference Path"))
{
    std::map<std::string, std::string> ref_opts;
    ref_opts["color"] = "k";
    ref_opts["linestyle"] = "--";
    ref_opts["label"] = "Reference Path";
    plt::plot(_ref_x, _ref_y, ref_opts);
}
```

---

## 📊 使用场景

### **场景 1: 专注于 MPC 轨迹优化**

**隐藏**:
- ☐ Reference Path
- ☐ Guidance Paths
- ☐ Predictions

**保留**:
- ☑ MPC Trajectories
- ☑ Static Obstacles
- ☑ Dynamic Obstacles
- ☑ Robot Footprint
- ☑ Goal

**效果**: 清晰地观察 T-MPC 的多轨迹并行优化过程

---

### **场景 2: 分析引导规划**

**隐藏**:
- ☐ MPC Trajectories
- ☐ Predictions
- ☐ Robot Footprint

**保留**:
- ☑ Reference Path
- ☑ Guidance Paths
- ☑ Static Obstacles
- ☑ Dynamic Obstacles
- ☑ Goal

**效果**: 专注于全局引导规划器生成的不同拓扑类别的路径

---

### **场景 3: 观察避障行为**

**隐藏**:
- ☐ Reference Path
- ☐ Guidance Paths

**保留**:
- ☑ MPC Trajectories
- ☑ Static Obstacles
- ☑ Dynamic Obstacles
- ☑ Predictions
- ☑ Robot Footprint
- ☑ Ego History

**效果**: 清晰地看到机器人如何根据障碍物预测调整轨迹

---

### **场景 4: 简化视图（演示模式）**

**隐藏**:
- ☐ Reference Path
- ☐ Guidance Paths
- ☐ Predictions
- ☐ Robot Footprint

**保留**:
- ☑ MPC Trajectories（只显示选中的）
- ☑ Static Obstacles
- ☑ Dynamic Obstacles
- ☑ Ego History
- ☑ Goal

**效果**: 最简洁的视图，适合演示和录制视频

---

## 🎨 界面调整

### **窗口大小**

```python
fig = plt.figure(figsize=(12.0, 6.4))  # 宽度增加到 12.0 以容纳勾选框
```

### **布局调整**

```python
gs = fig.add_gridspec(2, 3, height_ratios=[3, 2], width_ratios=[3, 2, 1])
#                        ^                                          ^
#                     3 列                                    第3列给勾选框
```

### **勾选框位置**

```python
check_ax = fig.add_axes([0.87, 0.10, 0.12, 0.78])
#                        ^     ^     ^     ^
#                        左    下    宽    高
```

---

## 🚀 性能优化

### **避免不必要的绘制**

通过在绘制前检查可见性，可以：
- **减少绘图调用**: 不绘制隐藏的元素
- **提高帧率**: 减少 matplotlib 的渲染负担
- **降低 CPU 使用**: 特别是在复杂场景中

### **示例**

```cpp
// 如果隐藏了引导路径，完全跳过循环
if (isVisible("Guidance Paths"))
{
    for (const auto &path : guidance_paths)
    {
        // 绘制引导路径
    }
}
// 如果 isVisible 返回 false，整个循环被跳过
```

---

## 📝 代码位置

### **主要修改文件**

- **`main.cpp`**:
  - `MatplotlibVisualizer::initialize()`: 初始化勾选框
  - `MatplotlibVisualizer::isVisible()`: 检查可见性
  - `MatplotlibVisualizer::update()`: 绘制前检查可见性

### **关键代码段**

1. **初始化勾选框** (第 148-224 行)
2. **可见性检查函数** (第 679-698 行)
3. **绘制时的可见性检查** (分布在 update 函数中)

---

## 🎯 总结

### **新增功能**

✅ **9 个可控制的可视化元素**  
✅ **实时切换显示/隐藏**  
✅ **直观的勾选框界面**  
✅ **性能优化（避免绘制隐藏元素）**  
✅ **灵活的场景配置**

### **使用建议**

1. **调试时**: 隐藏不相关的元素，专注于特定功能
2. **演示时**: 使用简化视图，突出关键信息
3. **分析时**: 根据需要组合不同的元素
4. **录制时**: 选择最清晰的视图配置

### **未来扩展**

可以考虑添加：
- 预设配置（一键切换到常用场景）
- 保存/加载可见性配置
- 更多可视化元素（如速度矢量、加速度等）
- 颜色和样式自定义

---

**享受更灵活的可视化体验！** 🎉


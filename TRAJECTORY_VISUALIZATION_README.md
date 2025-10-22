# MINCO 轨迹可视化工具

## 📋 概述

本工具用于可视化 MINCO 轨迹优化器生成的轨迹，包括：
- XY 平面轨迹（带速度颜色映射和朝向箭头）
- 速度曲线 (vx vs time)
- 角速度曲线 (omega vs time)
- 加速度曲线 (acceleration vs time)
- 曲率曲线 (curvature vs path_length)
- 朝向角曲线 (yaw vs time)
- 起点/终点状态
- 车辆几何参数
- 运动学约束
- 优化器参数

---

## 📁 文件说明

### 1. **轨迹日志文件**

**文件名**: `minco_trajectory_full.log`

**生成方式**: 运行 `navsim_local_debug` 时自动生成

**内容**:
- 轨迹统计信息（总点数、总时间、总长度）
- 起点状态（位置、速度）
- 终点状态（位置）
- 车辆几何参数（轴距、车长、车宽等）
- 车辆动力学约束（最大速度、加速度等）
- 优化器约束（安全距离、权重等）
- 轨迹点数据（位置、速度、加速度、曲率等）

**格式**:
```
# ============================================================
# MINCO Trajectory Full Log
# ============================================================
#
# [Trajectory Statistics]
# Total points: 8801
# Total time: 17.807 s
# Total length: 19.5391 m
#
# [Start State]
# start_x: 0 m
# start_y: 0 m
# start_yaw: 0 rad
# start_vx: 0 m/s
# ...
#
# [Trajectory Points]
# ============================================================
0, 0.00000000, 0.00000000, 0.00000000, 0.50000000, ...
1, 0.00102441, 0.00000000, 0.00000004, 0.49997949, ...
...
```

### 2. **可视化脚本**

#### **visualize_trajectory.py** (交互式版本)

**用途**: 在窗口中显示可视化图表，可以交互式缩放、平移

**使用方法**:
```bash
cd navsim-local
python3 visualize_trajectory.py [log_file]
```

**参数**:
- `log_file` (可选): 轨迹日志文件路径，默认为 `minco_trajectory_full.log`

**示例**:
```bash
# 使用默认日志文件
python3 visualize_trajectory.py

# 使用指定日志文件
python3 visualize_trajectory.py my_trajectory.log
```

#### **visualize_trajectory_save.py** (保存图片版本)

**用途**: 生成可视化图表并保存为 PNG 图片文件

**使用方法**:
```bash
cd navsim-local
python3 visualize_trajectory_save.py [log_file] [output_file]
```

**参数**:
- `log_file` (可选): 轨迹日志文件路径，默认为 `minco_trajectory_full.log`
- `output_file` (可选): 输出图片文件路径，默认为 `minco_trajectory_visualization.png`

**示例**:
```bash
# 使用默认参数
python3 visualize_trajectory_save.py

# 指定输出文件名
python3 visualize_trajectory_save.py minco_trajectory_full.log my_plot.png
```

---

## 🎨 可视化内容

### **布局**

图表采用 3x3 网格布局：

```
┌─────────────────────────┬─────────┐
│                         │ Chart 2 │
│                         │ Velocity│
│      Chart 1            ├─────────┤
│   XY Trajectory         │ Chart 3 │
│     (2x2 grid)          │ Omega   │
├─────────┬─────────┬─────┼─────────┤
│ Chart 4 │ Chart 5 │Chart│ Chart 6 │
│  Accel  │Curvature│  6  │  Yaw    │
└─────────┴─────────┴─────┴─────────┘
```

### **图表详情**

#### **Chart 1: XY 平面轨迹** (左上，占 2x2)
- 轨迹点用速度颜色映射（蓝色=慢，黄色=快）
- 绿色圆圈标记起点
- 红色圆圈标记终点
- 红色箭头显示朝向（每隔一定间隔）
- 四个角落显示信息框：
  - **左上**: 起点和终点状态
  - **右上**: 车辆几何参数
  - **左下**: 优化器约束
  - **右下**: 轨迹统计信息

#### **Chart 2: 速度曲线** (右上)
- X 轴: 时间 (s)
- Y 轴: 线速度 (m/s)
- 显示速度随时间的变化

#### **Chart 3: 角速度曲线** (右中)
- X 轴: 时间 (s)
- Y 轴: 角速度 (rad/s)
- 显示角速度随时间的变化

#### **Chart 4: 加速度曲线** (左下)
- X 轴: 时间 (s)
- Y 轴: 加速度 (m/s²)
- 显示加速度随时间的变化

#### **Chart 5: 曲率曲线** (中下)
- X 轴: 路径长度 (m)
- Y 轴: 曲率 (1/m)
- 过滤掉异常大的曲率值（速度接近 0 时的数值问题）

#### **Chart 6: 朝向角曲线** (右下)
- X 轴: 时间 (s)
- Y 轴: 朝向角 (deg)
- 显示朝向角随时间的变化

---

## 🚀 完整工作流程

### **步骤 1: 生成轨迹日志**

```bash
cd navsim-local
./build/navsim_local_debug --scenario scenarios/map1.json --planner JpsPlanner --perception EsdfBuilder
```

这会生成 `minco_trajectory_full.log` 文件。

### **步骤 2: 可视化轨迹**

**选项 A: 交互式查看**
```bash
python3 visualize_trajectory.py
```

**选项 B: 保存为图片**
```bash
python3 visualize_trajectory_save.py
```

生成的图片文件: `minco_trajectory_visualization.png`

---

## 📊 元数据字段

日志文件中包含以下元数据字段：

### **轨迹统计**
- `Total points`: 轨迹点总数
- `Total time`: 轨迹总时间 (s)
- `Total length`: 轨迹总长度 (m)

### **起点状态**
- `start_x`, `start_y`, `start_yaw`: 起点位置和朝向
- `start_vx`, `start_vy`, `start_omega`: 起点速度

### **终点状态**
- `goal_x`, `goal_y`, `goal_yaw`: 终点位置和朝向

### **车辆几何**
- `chassis_model`: 底盘类型 (differential, ackermann, etc.)
- `wheelbase`: 轴距 (m)
- `track_width`: 轮距 (m)
- `front_overhang`, `rear_overhang`: 前后悬 (m)
- `body_length`, `body_width`: 车体长宽 (m)
- `wheel_radius`: 轮半径 (m)

### **车辆动力学约束**
- `max_velocity`: 最大速度 (m/s)
- `max_acceleration`: 最大加速度 (m/s²)
- `max_deceleration`: 最大减速度 (m/s²)
- `max_steer_angle`: 最大转向角 (rad)
- `max_steer_rate`: 最大转向角速度 (rad/s)
- `max_jerk`: 最大急动度 (m/s³)
- `max_curvature`: 最大曲率 (1/m)

### **优化器约束**
- `max_vel`, `min_vel`: 速度约束 (m/s)
- `max_acc`: 加速度约束 (m/s²)
- `max_omega`: 角速度约束 (rad/s)
- `max_domega`: 角加速度约束 (rad/s²)
- `max_centripetal_acc`: 最大向心加速度 (m/s²)
- `safe_distance`: 安全距离 (m)
- `final_min_safe_distance`: 最终最小安全距离 (m)

### **优化器权重**
- `time_weight`: 时间权重
- `acc_weight`: 加速度权重
- `domega_weight`: 角加速度权重
- `collision_weight`: 碰撞权重
- `moment_weight`: 力矩权重
- `mean_time_weight`: 平均时间权重

---

## 🔧 依赖项

```bash
pip3 install numpy matplotlib
```

---

## 📝 示例输出

运行可视化脚本后，会看到类似以下输出：

```
============================================================
MINCO Trajectory Visualization (Save to File)
============================================================
📂 加载轨迹文件: minco_trajectory_full.log
💾 输出图片文件: minco_trajectory_visualization.png
✅ 成功加载 8801 个轨迹点
   时间范围: [0.000, 17.807] s
   路径长度: [19.539] m
   速度范围: [-0.121, 1.646] m/s
   元数据字段: 43 个

📊 生成可视化图表...

✅ 可视化图表已保存到: minco_trajectory_visualization.png

✅ 完成!
```

---

## 🎯 使用技巧

1. **查看特定区域**: 使用交互式版本 (`visualize_trajectory.py`)，可以缩放和平移图表

2. **对比不同轨迹**: 运行多次规划，保存不同的日志文件，然后分别可视化对比

3. **调试优化器**: 通过查看速度、加速度、曲率曲线，可以发现优化器的问题

4. **验证约束**: 检查速度、加速度是否超过约束限制

5. **分析性能**: 通过总时间、总长度等统计信息评估轨迹质量

---

## ✅ 完成！

现在你可以：
1. 运行 `navsim_local_debug` 生成轨迹日志
2. 使用 `visualize_trajectory.py` 或 `visualize_trajectory_save.py` 可视化轨迹
3. 查看完整的轨迹信息，包括起点、终点、车辆参数、约束等


# 配置文件指南

## 📋 概述

本项目使用**统一的配置文件**，位于 `mpc_planner_jackalsimulator/config/settings.yaml`。

## 📁 配置文件位置

### **唯一配置文件**
```
mpc_planner_jackalsimulator/config/settings.yaml
```

所有程序（主程序、测试、脚本）都使用这个配置文件。

### **已删除的配置**
- ~~`test/config/settings.yaml`~~ （已删除）

## 🚀 运行程序

### **方法 1: 使用构建脚本（推荐）**

```bash
./build_run_sim.sh
```

脚本会自动：
1. 配置环境变量（ACADOS_SOURCE_DIR, LD_LIBRARY_PATH）
2. 编译项目
3. 使用默认配置运行程序

### **方法 2: 手动运行**

#### **从项目根目录运行**
```bash
cd build
cmake ..
make -j$(nproc)
./mpc_planner_main
```

程序会自动查找配置文件：
- `mpc_planner_jackalsimulator/config/settings.yaml`

#### **从 build 目录运行**
```bash
cd build
./mpc_planner_main
```

程序会自动查找：
- `../mpc_planner_jackalsimulator/config/settings.yaml`

#### **指定配置路径**
```bash
./mpc_planner_main /path/to/config/directory
# 或
./mpc_planner_main /path/to/settings.yaml
```

## ⚙️ 配置文件结构

### **核心参数**

```yaml
name: "jackal"
N: 30                    # 预测时域
integrator_step: 0.2     # 积分步长 [s]
n_discs: 1               # 机器人建模圆盘数量（必须与求解器生成时一致）

enable_output: true
control_frequency: 20    # 控制频率 [Hz]
```

### **机器人参数**

```yaml
robot_radius: 0.325      # 机器人半径 [m] (应与 robot.width / 2 一致)
robot:
  length: 0.65           # 车辆总长 [m]
  width: 0.65            # 车辆总宽 [m]
  com_to_back: 0.0       # 质心到后部的距离 [m]
```

### **求解器设置**

```yaml
solver_settings:
  solver: "acados"       # acados 或 forces
  acados:
    iterations: 10
    solver_type: SQP_RTI # SQP_RTI (默认) 或 SQP
  tolstat: 1e-3          # 静态容差
```

### **MPC 权重**

```yaml
weights:
  goal: 1.0
  goal_x: 1.0
  goal_y: 1.0
  velocity: 0.55
  acceleration: 0.34
  angular_velocity: 0.85
  reference_velocity: 2.0
  contour: 0.05
  lag: 0.75
  slack: 10000.0
  terminal_angle: 100.0
  terminal_contouring: 10.0
```

### **道路约束**

```yaml
road:
  two_way: false         # 道路是否双向
  width: 6.0             # 道路宽度 [m]
```

### **T-MPC 设置**

```yaml
t-mpc:
  use_t-mpc++: true                    # 使用 T-MPC++
  enable_constraints: true             # 启用同伦约束
  highlight_selected: true             # 高亮选中的轨迹
  warmstart_with_mpc_solution: false   # 使用 MPC 解进行热启动
```

## ⚠️ 重要注意事项

### **1. n_discs 参数必须一致**

`n_discs` 参数决定了 ACADOS 求解器的参数数量，必须在以下两个地方保持一致：

1. **求解器生成时**: `mpc_planner_jackalsimulator/config/settings.yaml`
2. **运行时**: 同一个文件

**如果修改了 `n_discs`**：
```bash
# 1. 修改配置文件
vim mpc_planner_jackalsimulator/config/settings.yaml
# 修改 n_discs: 1 为其他值

# 2. 重新生成求解器
python3 mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py

# 3. 重新编译
cd build
cmake ..
make -j$(nproc)

# 4. 运行
./mpc_planner_main
```

### **2. robot_radius 应与 robot.width 一致**

```yaml
robot_radius: 0.325  # 应该等于 robot.width / 2
robot:
  width: 0.65        # robot_radius = 0.65 / 2 = 0.325
```

### **3. 动力学模型参数**

车辆的轴距（wheelbase）等参数在 `solver_generator/solver_model.py` 中定义：

```python
# BicycleModel2ndOrder
wheel_base = 2.79  # [m]
```

如需修改，需要：
1. 修改 `solver_generator/solver_model.py`
2. 重新生成求解器
3. 重新编译

## 📊 配置示例

### **小型机器人（Jackal）**

```yaml
n_discs: 1
robot_radius: 0.325
robot:
  length: 0.65
  width: 0.65
  com_to_back: 0.0
```

### **大型车辆**

```yaml
n_discs: 4
robot_radius: 1.125
robot:
  length: 4.54
  width: 2.25
  com_to_back: 2.27
```

## 🔧 自动配置查找

程序会按以下顺序自动查找配置文件：

1. `mpc_planner_jackalsimulator/config/settings.yaml` （从项目根目录）
2. `../mpc_planner_jackalsimulator/config/settings.yaml` （从 build 目录）
3. `../../mpc_planner_jackalsimulator/config/settings.yaml` （从 build/pure_cpp 目录）

如果找不到配置文件，程序会输出错误信息并列出搜索路径。

## 📝 配置文件修改流程

### **修改机器人尺寸**

```bash
# 1. 编辑配置文件
vim mpc_planner_jackalsimulator/config/settings.yaml

# 2. 修改参数
robot_radius: 0.5
robot:
  length: 1.0
  width: 1.0
  com_to_back: 0.5

# 3. 如果修改了 n_discs，需要重新生成求解器
python3 mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py

# 4. 重新编译
cd build && cmake .. && make -j$(nproc)

# 5. 运行
./mpc_planner_main
```

### **修改 MPC 权重**

```bash
# 1. 编辑配置文件
vim mpc_planner_jackalsimulator/config/settings.yaml

# 2. 修改权重
weights:
  velocity: 1.0        # 增加速度权重
  acceleration: 0.5    # 增加加速度权重

# 3. 无需重新生成求解器，直接运行
cd build
./mpc_planner_main
```

### **修改控制频率**

```bash
# 1. 编辑配置文件
vim mpc_planner_jackalsimulator/config/settings.yaml

# 2. 修改频率
control_frequency: 10  # 从 20 Hz 改为 10 Hz

# 3. 无需重新生成求解器，直接运行
cd build
./mpc_planner_main
```

## 🎯 总结

### **优点**

✅ **统一配置** - 只有一个配置文件，避免混淆  
✅ **自动查找** - 程序智能查找配置文件位置  
✅ **灵活运行** - 可从任何目录运行程序  
✅ **易于维护** - 修改配置只需编辑一个文件  

### **关键点**

1. **唯一配置文件**: `mpc_planner_jackalsimulator/config/settings.yaml`
2. **n_discs 一致性**: 修改后需重新生成求解器
3. **自动查找**: 程序会自动查找配置文件
4. **灵活运行**: 支持从不同目录运行

### **常用命令**

```bash
# 快速运行（推荐）
./build_run_sim.sh

# 手动运行
cd build && ./mpc_planner_main

# 指定配置
./mpc_planner_main /path/to/config

# 重新生成求解器
python3 mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py
```

现在你可以轻松管理配置文件，无需担心多个配置文件不一致的问题！🚀


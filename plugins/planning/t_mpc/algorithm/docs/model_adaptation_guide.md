# 动力学模型自适应指南

## 📋 概述

本项目现在支持**自动检测动力学模型**，无需手动修改 C++ 代码即可切换不同的车辆模型。

## 🎯 支持的模型

### 1. **单轮模型（Unicycle Model）**
- **类名**: `ContouringSecondOrderUnicycleModel`
- **状态数量**: 5
- **状态变量**: `x`, `y`, `psi`, `v`, `spline`
- **控制输入**: `a` (加速度), `w` (角速度)
- **适用场景**: 差速驱动机器人

### 2. **带松弛变量的单轮模型**
- **类名**: `ContouringSecondOrderUnicycleModelWithSlack`
- **状态数量**: 5
- **状态变量**: `x`, `y`, `psi`, `v`, `spline`
- **控制输入**: `a`, `w`, `slack`
- **适用场景**: 需要软约束的差速驱动机器人

### 3. **自行车模型（Bicycle Model）** ⭐
- **类名**: `BicycleModel2ndOrder`
- **状态数量**: 6
- **状态变量**: `x`, `y`, `psi`, `v`, `delta` (转向角), `spline`
- **控制输入**: `a`, `w` (转向角速度), `slack`
- **适用场景**: 阿克曼转向车辆（汽车、卡车等）
- **参数**:
  - 轴距: 2.79m
  - 最大转向角: ±0.55 rad

### 4. **曲率感知自行车模型**
- **类名**: `BicycleModel2ndOrderCurvatureAware`
- **状态数量**: 6
- **状态变量**: `x`, `y`, `psi`, `v`, `delta`, `spline`
- **控制输入**: `a`, `w`, `slack`
- **适用场景**: 需要考虑路径曲率的阿克曼车辆

---

## 🔄 如何切换模型

### **步骤 1: 修改求解器生成脚本**

编辑 `mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py`:

```python
# 从单轮模型切换到自行车模型
from solver_generator.solver_model import BicycleModel2ndOrder

# 在 main() 函数中
model = BicycleModel2ndOrder()  # 替换原来的模型
```

### **步骤 2: 重新生成求解器**

```bash
python3 mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py
```

### **步骤 3: 重新编译 C++ 代码**

```bash
cd build
cmake ..
make -j$(nproc)
```

### **步骤 4: 运行程序**

```bash
./mpc_planner_main ../test/config
```

程序会自动检测模型类型并输出：

```
[INFO] ========================================
[INFO] 动力学模型信息
[INFO] ========================================
[INFO] 模型类型: BicycleModel2ndOrder
[INFO] 状态数量: 6
[INFO] 控制输入数量: 3
[INFO] ----------------------------------------
[INFO] 状态变量:
[INFO]   ✓ x
[INFO]   ✓ y
[INFO]   ✓ psi
[INFO]   ✓ v
[INFO]   ✓ delta
[INFO]   ✓ spline
[INFO] 控制输入:
[INFO]   ✓ a
[INFO]   ✓ w
[INFO]   ✓ slack
[INFO] ========================================
```

---

## ⚙️ 配置参数适配

### **重要：`n_discs` 参数必须一致**

在切换模型后，确保以下两个配置文件中的 `n_discs` 参数一致：

1. **求解器生成配置**: `mpc_planner_jackalsimulator/config/settings.yaml`
2. **运行时配置**: `test/config/settings.yaml`

```yaml
# 两个文件中都应该设置相同的值
n_discs: 4  # 或 2, 3, 5 等
```

**为什么重要？**
- `n_discs` 决定了 ACADOS 求解器的参数数量
- 不一致会导致参数数量不匹配，程序崩溃

---

## 🧪 自适应机制

### **ModelDetector 类**

项目使用 `ModelDetector` 类自动检测模型类型：

```cpp
#include <mpc_planner_solver/model_detector.h>

// 自动检测模型
ModelDetector detector;
detector.printModelInfo();

// 查询模型特性
if (detector.hasState("delta")) {
    // 自行车模型
    double wheelbase = detector.getWheelBase();
    double max_steering = detector.getMaxSteeringAngle();
}
```

### **自适应状态初始化**

```cpp
void initializeState() {
    // 基础状态（所有模型都有）
    state_.set("x", 0.0);
    state_.set("y", 0.0);
    state_.set("psi", 0.0);
    state_.set("v", 0.5);
    
    // 自适应：根据模型自动初始化额外状态
    if (model_detector_->hasState("delta")) {
        state_.set("delta", 0.0);  // 自行车模型
    }
    
    if (model_detector_->hasState("slack")) {
        state_.set("slack", 0.0);  // 松弛变量
    }
    
    state_.set("spline", 0.0);
}
```

### **自适应运动学积分**

```cpp
void integrateState(double v, double w, double dt) {
    if (model_detector_->hasState("delta")) {
        // 自行车模型运动学
        double wheelbase = model_detector_->getWheelBase();
        double beta = atan(0.5 * tan(delta));
        x += v * cos(psi + beta) * dt;
        y += v * sin(psi + beta) * dt;
        psi += (v / (wheelbase/2)) * sin(beta) * dt;
        delta += w * dt;
    } else {
        // 单轮模型运动学
        x += v * cos(psi) * dt;
        y += v * sin(psi) * dt;
        psi += w * dt;
    }
}
```

---

## 📊 模型对比

| 特性 | 单轮模型 | 自行车模型 |
|------|---------|-----------|
| **状态数量** | 5 | 6 |
| **转向方式** | 直接角速度控制 | 转向角控制 |
| **适用车辆** | 差速驱动 | 阿克曼转向 |
| **计算复杂度** | 低 | 中 |
| **转向灵活性** | 高（原地转向） | 低（需要转弯半径） |
| **真实性** | 低（简化模型） | 高（接近真实车辆） |

---

## ⚠️ 常见问题

### **1. 参数数量不匹配错误**

```
acados_update_params: trying to set 175 parameters for external functions. 
External function has 179 parameters. Exiting.
```

**解决方案**: 检查 `n_discs` 参数是否一致

### **2. 状态初始化失败**

```
State variable 'delta' not found
```

**解决方案**: 确保已重新生成求解器并重新编译

### **3. QP 求解器警告**

```
SQP_RTI: QP solver returned error status 3
```

**说明**: 这是正常的，表示某些迭代中遇到数值问题，但不影响运行

---

## 🚀 最佳实践

1. **修改模型前先备份**
   ```bash
   cp -r mpc_planner_solver mpc_planner_solver.backup
   ```

2. **验证配置一致性**
   ```bash
   grep "n_discs" mpc_planner_jackalsimulator/config/settings.yaml
   grep "n_discs" test/config/settings.yaml
   ```

3. **清理旧的构建文件**
   ```bash
   rm -rf build
   mkdir build && cd build
   cmake .. && make -j$(nproc)
   ```

4. **检查模型检测输出**
   - 运行程序时查看 `[INFO] 动力学模型信息` 输出
   - 确认状态变量和控制输入符合预期

---

## 📚 相关文件

- **模型定义**: `solver_generator/solver_model.py`
- **求解器生成**: `mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py`
- **模型检测器**: `mpc_planner_solver/include/mpc_planner_solver/model_detector.h`
- **主程序**: `main.cpp`
- **配置文件**: 
  - `mpc_planner_jackalsimulator/config/settings.yaml`
  - `test/config/settings.yaml`

---

## 🎓 总结

通过 `ModelDetector` 类，项目实现了：

✅ **自动模型检测** - 无需手动修改代码  
✅ **自适应状态初始化** - 根据模型自动初始化状态  
✅ **自适应运动学积分** - 根据模型选择正确的运动学方程  
✅ **详细的模型信息输出** - 方便调试和验证  

现在你可以随时切换不同的动力学模型，只需：
1. 修改求解器生成脚本
2. 重新生成求解器
3. 重新编译
4. 运行程序

程序会自动适配新的模型！🚀


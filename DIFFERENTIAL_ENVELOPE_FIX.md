# 差速底盘车辆包络修复文档

## 📋 问题描述

### 修复前的问题

**navsim-local**：
- ❌ 差速底盘绘制为圆形 + 方向指示线（不准确）
- ❌ 无法精确表示车辆的实际包络
- ❌ 碰撞检测不准确

**navsim-online**：
- ❌ 默认配置错误：`wheelbase = 0.5`（应该是 0）
- ❌ 默认配置错误：`body_length = 0.6`（应该是 `front_overhang + rear_overhang`）
- ❌ 车体位置计算错误（未基于驱动轴中点）
- ❌ 万向轮位置计算错误

---

## 🎯 差速底盘的标准定义

### 坐标系基准

- **原点**：驱动轴中点（两个驱动轮中心连线的中点）
- **X 轴**：向前为正
- **Y 轴**：向左为正
- **Z 轴**：向上为正

### 几何参数

- **前悬**：`FOH` (front_overhang) - 从驱动轴到车体前端的距离
- **后悬**：`ROH` (rear_overhang) - 从驱动轴到车体后端的距离
- **车宽**：`W` (width)
- **轴距**：`wheelbase = 0`（差速底盘只有一根驱动轴，不是两根轴）
- **车体长度**：`body_length = FOH + ROH`（由前悬和后悬决定，不是独立参数）

### 车辆包络的四个角点

逆时针顺序：
```
P1 (前左) = (+FOH,  +W/2)
P2 (前右) = (+FOH,  -W/2)
P3 (后右) = (-ROH,  -W/2)
P4 (后左) = (-ROH,  +W/2)
```

### 具体示例

假设参数：
- 前悬 FOH = 0.3 m
- 后悬 ROH = 0.2 m
- 车宽 W = 0.5 m
- 轴距 wheelbase = 0 m（固定值）
- 车体长度 = 0.3 + 0.2 = 0.5 m（计算得出）

四个角点：
```
P1 (前左) = (0.3,  +0.25)
P2 (前右) = (0.3,  -0.25)
P3 (后右) = (-0.2, -0.25)
P4 (后左) = (-0.2, +0.25)
```

---

## 🔧 修复方案

### 1. navsim-local 可视化修复

**文件**：`navsim-local/src/viz/imgui_visualizer.cpp`（第 782-843 行）

**修复前的代码**：
```cpp
if (ego_.chassis_model == "differential") {
  // 🤖 差速底盘：圆形机器人 + 方向指示
  double radius = ego_.kinematics.body_width / 2.0;
  auto ego_pos = worldToScreen(ego_.pose.x, ego_.pose.y);
  
  // 绘制圆形本体
  draw_list->AddCircleFilled(...);  // ❌ 不准确
  
  // 绘制方向指示线
  draw_list->AddLine(...);
}
```

**修复后的代码**：
```cpp
if (ego_.chassis_model == "differential") {
  // 🤖 差速底盘：精确矩形包络
  // 🔧 标准定义：驱动轴中点为原点
  double half_width = ego_.kinematics.body_width / 2.0;
  
  // 🔧 前保险杠 X 坐标 = front_overhang（从驱动轴开始）
  double x_front = ego_.kinematics.front_overhang;
  // 🔧 后保险杠 X 坐标 = -rear_overhang（从驱动轴开始，向后为负）
  double x_rear = -ego_.kinematics.rear_overhang;
  
  // 🔧 计算车辆的四个角点（在车辆局部坐标系中，驱动轴中点为原点）
  std::vector<std::pair<double, double>> corners_local = {
    {x_front, half_width},   // P1: 前左 = (front_overhang, +width/2)
    {x_front, -half_width},  // P2: 前右 = (front_overhang, -width/2)
    {x_rear, -half_width},   // P3: 后右 = (-rear_overhang, -width/2)
    {x_rear, half_width}     // P4: 后左 = (-rear_overhang, +width/2)
  };
  
  // 转换到世界坐标系并绘制
  // ...
  
  // 🔧 绘制车头方向指示（黄色圆点）
  // 🔧 绘制驱动轴位置（红色小圆点，原点）
}
```

**关键改进**：
- ✅ 使用精确的矩形包络（四个角点）
- ✅ 基于驱动轴中点（原点）计算
- ✅ 绘制车头方向指示（黄色圆点）
- ✅ 绘制驱动轴位置（红色圆点）

---

### 2. navsim-online 配置参数修复

**文件**：`navsim-online/server/main.py`（第 91-109 行）

**修复前的配置**：
```python
chassis: Dict[str, Any] = field(
    default_factory=lambda: {
        "model": "differential",
        "wheelbase": 0.5,  # ❌ 应该是 0
        "track_width": 0.4,
        "limits": {...},
        "geometry": {
            "body_length": 0.6,  # ❌ 应该是 front_overhang + rear_overhang
            "body_width": 0.5,
            "body_height": 0.3,
            "wheel_radius": 0.08,
            "wheel_width": 0.05,
            "front_overhang": 0.05,  # ❌ 太小
            "rear_overhang": 0.05,   # ❌ 太小
            "caster_count": 2,
            "track_width_ratio": 0.0,
        },
    }
)
```

**修复后的配置**：
```python
chassis: Dict[str, Any] = field(
    default_factory=lambda: {
        "model": "differential",
        "wheelbase": 0.0,  # ✅ 差速底盘轴距固定为 0
        "track_width": 0.4,
        "limits": {...},
        "geometry": {
            "body_length": 0.5,  # ✅ = front_overhang + rear_overhang
            "body_width": 0.5,
            "body_height": 0.3,
            "wheel_radius": 0.08,
            "wheel_width": 0.05,
            "front_overhang": 0.3,  # ✅ 合理的前悬
            "rear_overhang": 0.2,   # ✅ 合理的后悬
            "caster_count": 2,
            "track_width_ratio": 0.0,
        },
    }
)
```

**关键改进**：
- ✅ `wheelbase` 固定为 0（差速底盘只有一根驱动轴）
- ✅ `body_length = 0.5 = front_overhang(0.3) + rear_overhang(0.2)`
- ✅ 合理的前悬和后悬参数

---

### 3. navsim-online Web 前端配置修复

**文件**：`navsim-online/web/index.html`

#### 3.1 CHASSIS_CONFIGS 默认配置修复（第 2120-2138 行）

**修复前的配置**：
```javascript
'differential': {
  model: 'differential',
  wheelbase: 0.5,  // ❌ 应该是 0
  track_width: 0.4,
  limits: { ... },
  geometry: {
    body_length: 0.6,      // ❌ 应该是 0.5
    front_overhang: 0.05,  // ❌ 太小
    rear_overhang: 0.05,   // ❌ 太小
    ...
  }
}
```

**修复后的配置**：
```javascript
'differential': {
  model: 'differential',
  wheelbase: 0.0,  // ✅ 差速底盘轴距固定为 0
  track_width: 0.4,
  limits: { ... },
  geometry: {
    body_length: 0.5,      // ✅ = 0.3 + 0.2
    front_overhang: 0.3,   // ✅ 合理值
    rear_overhang: 0.2,    // ✅ 合理值
    ...
  }
}
```

#### 3.2 配置编辑 UI 修复（第 5820-5920 行）

**修复内容**：

1. **openConfigModal 函数**：
   - 差速底盘的 `wheelbase` 字段设置为只读（`readOnly = true`）
   - 差速底盘的 `body_length` 字段设置为只读（`readOnly = true`）
   - 添加灰色背景和提示信息
   - 监听前悬/后悬输入变化，实时更新 `body_length` 显示值

2. **saveConfig 函数**：
   - 差速底盘强制 `wheelbase = 0.0`
   - 差速底盘自动计算 `body_length = front_overhang + rear_overhang`

3. **closeConfigModal 函数**：
   - 清理差速底盘的事件监听器

**关键代码**：
```javascript
// openConfigModal 中
if (chassisType === 'differential') {
  // 设置 wheelbase 为只读
  wheelbaseInput.readOnly = true;
  wheelbaseInput.title = '差速底盘轴距固定为 0（只有一根驱动轴）';
  wheelbaseInput.style.backgroundColor = '#f3f4f6';

  // 设置 body_length 为只读
  bodyLengthInput.readOnly = true;
  bodyLengthInput.title = '车体长度 = 前悬 + 后悬（自动计算）';
  bodyLengthInput.style.backgroundColor = '#f3f4f6';

  // 监听前悬/后悬变化，实时更新 body_length
  updateBodyLengthHandler = () => {
    const frontOverhang = parseFloat(frontOverhangInput.value) || 0;
    const rearOverhang = parseFloat(rearOverhangInput.value) || 0;
    bodyLengthInput.value = (frontOverhang + rearOverhang).toFixed(2);
  };
  frontOverhangInput.addEventListener('input', updateBodyLengthHandler);
  rearOverhangInput.addEventListener('input', updateBodyLengthHandler);
}

// saveConfig 中
if (currentEditingChassis === 'differential') {
  newConfig.wheelbase = 0.0;
  newConfig.geometry.body_length = frontOverhang + rearOverhang;
}
```

#### 3.3 自车可视化修复（第 2320-2383 行）

**修复前的代码**：
```javascript
if (model === 'differential') {
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(geom.body_length, geom.body_height, geom.body_width),
    bodyMaterial
  );
  body.position.y = geom.body_height / 2;  // ❌ 未考虑驱动轴偏移
  group.add(body);
  
  // 万向轮位置错误
  frontCaster.position.set(geom.body_length / 2 - geom.front_overhang, ...);  // ❌
}
```

**修复后的代码**：
```javascript
if (model === 'differential') {
  // 🔧 标准定义：驱动轴中点为原点
  const bodyOffsetX = (geom.front_overhang - geom.rear_overhang) / 2;
  
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(geom.body_length, geom.body_height, geom.body_width),
    bodyMaterial
  );
  body.position.set(bodyOffsetX, geom.body_height / 2, 0);  // ✅ X 方向偏移
  group.add(body);
  
  // 🔧 驱动轮位置：X=0（驱动轴位置）
  leftWheel.position.set(0, geom.wheel_radius, wheelOffset);
  rightWheel.position.set(0, geom.wheel_radius, -wheelOffset);
  
  // 🔧 万向轮位置（基于驱动轴）
  frontCaster.position.set(geom.front_overhang, casterRadius, 0);  // ✅ 从驱动轴向前
  rearCaster.position.set(-geom.rear_overhang, casterRadius, 0);   // ✅ 从驱动轴向后
}
```

#### 3.4 终点标记修复（第 2598-2648 行）

同样的修复应用于终点标记（红色车模）。

**关键改进**：
- ✅ 车体位置基于驱动轴中点（X 方向偏移）
- ✅ 驱动轮位置在驱动轴（X = 0）
- ✅ 万向轮位置基于驱动轴计算

---

## 📊 修复效果对比

### navsim-local

| 项目 | 修复前 | 修复后 |
|------|--------|--------|
| **可视化形状** | 圆形 ❌ | 精确矩形 ✅ |
| **包络准确性** | 不准确 ❌ | 准确 ✅ |
| **驱动轴标记** | 无 ❌ | 红色圆点 ✅ |
| **车头标记** | 黄色线 ❌ | 黄色圆点 ✅ |

### navsim-online

| 项目 | 修复前 | 修复后 |
|------|--------|--------|
| **wheelbase** | 0.5 ❌ | 0.0 ✅ |
| **body_length** | 0.6 ❌ | 0.5 (FOH+ROH) ✅ |
| **front_overhang** | 0.05 ❌ | 0.3 ✅ |
| **rear_overhang** | 0.05 ❌ | 0.2 ✅ |
| **车体位置** | 未偏移 ❌ | 基于驱动轴 ✅ |
| **万向轮位置** | 错误 ❌ | 正确 ✅ |

---

## 🧪 测试步骤

### 1. 重启 navsim-local

```bash
cd navsim-local
cmake --build build -j$(nproc)
./build/navsim_algo
```

### 2. 刷新 Web 页面

```bash
# 在浏览器中按 Ctrl+F5 强制刷新
```

### 3. 选择差速底盘

在 Web 界面中选择 "Differential Drive" 底盘类型。

### 4. 验证 navsim-local

- ✅ 车辆显示为矩形轮廓（不是圆形）
- ✅ 驱动轴位置有红色圆点（原点）
- ✅ 车头有黄色圆点
- ✅ 车辆尺寸符合配置参数

### 5. 验证 navsim-online

- ✅ 车体长度 = 0.5 m（前悬 0.3 + 后悬 0.2）
- ✅ 驱动轮在车体中部（驱动轴位置）
- ✅ 前万向轮在车体前端（距离驱动轴 0.3 m）
- ✅ 后万向轮在车体后端（距离驱动轴 0.2 m）

### 6. 验证一致性

- ✅ 前端和本地的车辆轮廓应该完全一致
- ✅ 车辆包络应该是精确的矩形
- ✅ 驱动轴位置应该在车体中部偏后（因为前悬 > 后悬）

---

## 📝 总结

### 修复内容

1. **navsim-local**：
   - ✅ 将圆形可视化改为精确矩形包络
   - ✅ 基于驱动轴中点计算四个角点
   - ✅ 添加驱动轴位置标记（红色圆点）
   - ✅ 添加车头方向标记（黄色圆点）

2. **navsim-online 服务器配置**：
   - ✅ 修正 `wheelbase = 0`（差速底盘固定值）
   - ✅ 修正 `body_length = 0.5`（前悬 + 后悬）
   - ✅ 修正前悬和后悬参数

3. **navsim-online Web 前端配置**：
   - ✅ 修正 `CHASSIS_CONFIGS` 中的默认配置
   - ✅ `wheelbase` 字段设置为只读（不可编辑）
   - ✅ `body_length` 字段设置为只读（自动计算）
   - ✅ 监听前悬/后悬变化，实时更新 `body_length` 显示值
   - ✅ 保存时强制 `wheelbase = 0` 和 `body_length = front_overhang + rear_overhang`

4. **navsim-online Web 前端可视化**：
   - ✅ 修正车体位置（基于驱动轴中点）
   - ✅ 修正驱动轮位置（X = 0）
   - ✅ 修正万向轮位置（基于驱动轴）

### 关键原则

- **坐标系基准**：驱动轴中点为原点
- **轴距固定**：差速底盘 `wheelbase = 0`
- **车体长度**：`body_length = front_overhang + rear_overhang`
- **包络计算**：四个角点 = `(±FOH, ±W/2)` 和 `(±ROH, ±W/2)`

---

## 🎉 预期效果

修复后，差速底盘将使用与阿克曼底盘相同的精确矩形包络计算方式，确保：
- ✅ 可视化准确
- ✅ 碰撞检测准确
- ✅ 前端和本地一致
- ✅ 符合标准定义


# Bug 修复：前端显示轨迹

## 🐛 问题描述

**用户报告**: 在完整运行场景下，本地算法生成了轨迹，但前端页面没有显示绿色轨迹线。

---

## 🔍 根本原因

### Topic 名称不匹配

**本地算法发送**:
```cpp
j["topic"] = "room/" + room_id_ + "/plan";
// 例如: "room/demo/plan"
```

**前端期望**:
```javascript
} else if (topic.endsWith('/plan_update')) {
    handlePlanUpdate(data, topic);
```

**结果**: 前端收到 `room/demo/plan` 消息，但不匹配 `/plan_update` 后缀，所以不会调用 `handlePlanUpdate()`。

### 数据字段不匹配

**本地算法发送**:
```json
{
  "data": {
    "points": [...]  // 使用 points 字段
  }
}
```

**前端期望**:
```javascript
if (Array.isArray(data?.trajectory) && data.trajectory.length) {
    // 期望 trajectory 字段
}
```

### 字段名称不匹配

**本地算法发送**:
```json
{
  "x": 0.0,
  "y": 0.0,
  "theta": 0.0,  // 使用 theta
  "t": 0.0
}
```

**前端期望**:
```javascript
{
  x: Number(pt.x ?? 0),
  y: Number(pt.y ?? 0),
  yaw: Number(pt.yaw ?? 0),  // 期望 yaw
  t: Number(pt.t ?? idx * 0.1),
}
```

---

## ✅ 解决方案

### 修改内容

修改 `navsim-local/src/bridge.cpp` 中的 `plan_to_json()` 函数：

**修改 1: Topic 名称**
```cpp
// 修改前
j["topic"] = "room/" + room_id_ + "/plan";

// 修改后
j["topic"] = "room/" + room_id_ + "/plan_update";
```

**修改 2: 数据字段**
```cpp
// 修改前
nlohmann::json points = nlohmann::json::array();
// ...
data["points"] = points;

// 修改后
nlohmann::json trajectory = nlohmann::json::array();
// ...
data["trajectory"] = trajectory;
```

**修改 3: 点的字段名称**
```cpp
// 修改前
point["theta"] = pt.yaw();

// 修改后
point["yaw"] = pt.yaw();
```

### 完整的修改后代码

```cpp
nlohmann::json Bridge::Impl::plan_to_json(const proto::PlanUpdate& plan, double compute_ms) {
  nlohmann::json j;
  // 修改为 plan_update 以匹配前端期望
  j["topic"] = "room/" + room_id_ + "/plan_update";

  // 构造 data
  nlohmann::json data;
  data["schema_ver"] = "1.0.0";
  data["tick_id"] = plan.tick_id();
  data["stamp"] = plan.stamp();
  data["n_points"] = plan.trajectory_size();
  data["compute_ms"] = compute_ms;

  // 转换 trajectory（前端期望 trajectory 字段，不是 points）
  nlohmann::json trajectory = nlohmann::json::array();
  double s = 0.0;  // 累积弧长

  for (int i = 0; i < plan.trajectory_size(); ++i) {
    const auto& pt = plan.trajectory(i);

    nlohmann::json point;
    point["x"] = pt.x();
    point["y"] = pt.y();
    point["yaw"] = pt.yaw();  // 前端期望 yaw 字段
    point["t"] = pt.t();

    // 计算 s（累积弧长）
    if (i > 0) {
      const auto& prev_pt = plan.trajectory(i - 1);
      double dx = pt.x() - prev_pt.x();
      double dy = pt.y() - prev_pt.y();
      s += std::sqrt(dx * dx + dy * dy);
    }
    point["s"] = s;

    // 计算 kappa（曲率，暂时填 0.0）
    point["kappa"] = 0.0;

    // 计算 v（速度，暂时填常数 0.8 m/s）
    point["v"] = 0.8;

    trajectory.push_back(point);
  }

  // 前端期望 trajectory 字段
  data["trajectory"] = trajectory;

  // 添加 summary（占位值）
  data["summary"] = {
    {"min_dyn_dist", 1.5},
    {"max_kappa", 0.3},
    {"total_length", s}
  };

  j["data"] = data;
  return j;
}
```

---

## 🔧 编译和测试

### 编译

```bash
cd navsim-local
cmake --build build
```

**预期输出**:
```
[ 93%] Building CXX object CMakeFiles/navsim_algo.dir/src/bridge.cpp.o
[ 95%] Linking CXX executable navsim_algo
[100%] Built target navsim_algo
```

### 测试

```bash
# 终端 1: 启动服务器
cd navsim-online
bash run_navsim.sh

# 终端 2: 启动本地算法
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo

# 浏览器: 打开前端
# http://127.0.0.1:8000/index.html
# 点击"连接 WebSocket"按钮
```

### 预期结果

**前端页面**:
- ✅ 右上角显示"已连接"（绿色）
- ✅ 3D 场景显示自车（蓝色）
- ✅ 3D 场景显示目标点（红色）
- ✅ **3D 场景显示绿色轨迹线**（这是修复的关键）
- ✅ 自车沿着绿色轨迹移动
- ✅ 话题控制台显示 `room/demo/plan_update` 消息

**本地算法日志**:
```
[Bridge] Sent plan with 190 points, compute_ms=0.1ms
```

**话题控制台**:
```
Topic: /room/demo/plan_update
Data: {
  "schema_ver": "1.0.0",
  "tick_id": 123,
  "trajectory": [
    {"x": 0.0, "y": 0.0, "yaw": 0.0, "t": 0.0, ...},
    ...
  ]
}
```

---

## 📊 修改前后对比

### 修改前

| 项目 | 值 | 结果 |
|------|-----|------|
| **Topic** | `room/demo/plan` | ❌ 不匹配 `/plan_update` |
| **数据字段** | `points` | ❌ 前端期望 `trajectory` |
| **点字段** | `theta` | ❌ 前端期望 `yaw` |
| **前端显示** | 无轨迹 | ❌ 失败 |

### 修改后

| 项目 | 值 | 结果 |
|------|-----|------|
| **Topic** | `room/demo/plan_update` | ✅ 匹配 |
| **数据字段** | `trajectory` | ✅ 匹配 |
| **点字段** | `yaw` | ✅ 匹配 |
| **前端显示** | 绿色轨迹线 | ✅ 成功 |

---

## 🎯 其他两个问题的答案

### 问题 2: 哪个按钮是开始仿真的？

**答案**: 没有"开始仿真"按钮！

**说明**:
- 仿真在客户端连接时**自动开始**
- 服务器自动生成 world_tick（20Hz）
- 前端自动渲染接收到的数据
- "播放/暂停"按钮只控制前端显示，不控制仿真

**自车运动**:
- 修复后，前端会根据 `plan_update` 自动让自车沿轨迹移动
- 不需要点击任何按钮

### 问题 3: 自车位置更新是否正常？

**当前状态**: 
- ✅ **前端显示**: 自车会沿着 plan_update 的轨迹移动（前端本地计算）
- ❌ **服务器状态**: 服务器的 ego_pose 不会根据 plan 更新

**说明**:
- 前端接收到 `plan_update` 后，会启动轨迹回放
- 前端的 `advanceEgoAlongTrajectory()` 函数会根据时间 t 插值计算自车位置
- 这是**前端本地的显示**，服务器的 ego_pose 仍然按照默认运动模型更新

**影响**:
- ✅ 前端显示正常（自车沿轨迹移动）
- ⚠️ 服务器的 world_tick.ego.pose 不会反映规划结果
- ⚠️ 如果前端断开，服务器的自车位置会回到默认运动模型

**完整解决方案**（可选，未实现）:
- 服务器接收 `plan_update` 并跟踪执行
- 或者服务器接收 `ego_cmd` 并应用控制命令
- 这需要修改服务器代码

---

## 📝 总结

### 修复的问题

✅ **问题 1**: 前端显示轨迹 - **已修复**

### 回答的问题

✅ **问题 2**: 开始仿真按钮 - **无需按钮，自动运行**

✅ **问题 3**: 自车位置更新 - **前端正常，服务器未实现**

### 修改的文件

- `navsim-local/src/bridge.cpp` - 修改 `plan_to_json()` 函数

### 测试状态

- ✅ 编译成功
- ⏳ 等待用户测试

---

**Bug 修复完成！请重新运行系统并查看前端是否显示绿色轨迹线。** 🎉

---

**文档结束**


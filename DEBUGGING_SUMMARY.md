# 静态障碍物显示问题调试总结

## 📋 问题描述

**用户报告**：ImGui 可视化系统中静态障碍物没有正确显示

**现象**：
- 可视化窗口中看不到静态障碍物
- 右侧信息栏显示 `bev_obstacles: 0`（障碍物数量为 0）

---

## 🔍 调试过程

### 第一步：添加调试日志

按照用户要求，逐步排查数据流的每个环节：

#### 1. 检查输入数据（BEVExtractor）

**文件**：`navsim-local/src/plugin/preprocessing/bev_extractor.cpp`

**添加的日志**：
```cpp
std::cout << "[BEVExtractor] ========== Extract called ==========" << std::endl;
std::cout << "[BEVExtractor] WorldTick tick_id: " << world_tick.tick_id() << std::endl;
std::cout << "[BEVExtractor] Has static_map: " << world_tick.has_static_map() << std::endl;
std::cout << "[BEVExtractor] Dynamic obstacles count: " << world_tick.dynamic_obstacles_size() << std::endl;
```

**发现**：`Has static_map: 0` ← **问题根源！**

#### 2. 检查处理过程（PreprocessingPipeline）

**文件**：`navsim-local/src/plugin/preprocessing/preprocessing_pipeline.cpp`

**添加的日志**：
```cpp
std::cout << "[PreprocessingPipeline] Extracting BEV obstacles..." << std::endl;
std::cout << "[PreprocessingPipeline] BEV obstacles extracted successfully:" << std::endl;
std::cout << "[PreprocessingPipeline]   Circles: " << input.bev_obstacles.circles.size() << std::endl;
```

**发现**：处理逻辑正常，但输入数据为空

#### 3. 检查可视化函数（ImGuiVisualizer）

**文件**：`navsim-local/src/viz/imgui_visualizer.cpp`

**添加的日志**：
```cpp
std::cout << "[ImGuiVisualizer] drawBEVObstacles called:" << std::endl;
std::cout << "[ImGuiVisualizer]   Input circles: " << obstacles.circles.size() << std::endl;
std::cout << "[ImGuiVisualizer]   Cached circles: " << bev_obstacles_.circles.size() << std::endl;
```

**发现**：可视化函数正常，但接收到的数据为空

### 第二步：分析根本原因

通过日志输出，我们发现：

```
[BEVExtractor] Has static_map: 0
[BEVExtractor] No cached static map, skipping static obstacles
```

**结论**：WorldTick 中没有包含静态地图数据！

### 第三步：查找原因

查看 `navsim-online/server/main.py` 的代码：

```python
map_payload = {"version": self.map_version}
if self.include_static_next_tick:
    map_payload["static"] = self.static_geometry
    self.include_static_next_tick = False  # 发送后立即设置为 False
```

**发现**：
- 静态地图只在 `include_static_next_tick == True` 时发送
- 发送后立即设置为 `False`
- 只在初始化、地图更新、重置时设置为 `True`

**问题**：如果 navsim-local 在 navsim-online 启动后才连接，就会错过第一个包含静态地图的 tick！

---

## ✅ 解决方案

### 修改 navsim-online

**文件**：`navsim-online/server/main.py`

**修改位置**：`register()` 方法（第 138-151 行）

**修改内容**：
```python
async def register(self, websocket: WebSocket) -> None:
    await websocket.accept()
    self.connections.add(websocket)
    self.active = True
    
    # 🔧 修复：新客户端连接时，强制在下一个 tick 发送静态地图
    # 这样可以确保后连接的客户端也能收到静态障碍物数据
    self.include_static_next_tick = True
    print(f"[Room {self.room_id}] New client connected, will send static map in next tick")
    
    if not self.generator_task or self.generator_task.done():
        self.generator_task = asyncio.create_task(self._run_generator())
    if not self.broadcaster_task or self.broadcaster_task.done():
        self.broadcaster_task = asyncio.create_task(self._run_broadcaster())
```

**优点**：
- ✅ 优雅的解决方案
- ✅ 不需要修改 navsim-local
- ✅ 适用于所有后连接的客户端
- ✅ 不影响现有功能

---

## 🧪 验证方法

### 自动化测试脚本

```bash
cd navsim-local
./test_static_obstacles.sh
```

### 手动验证步骤

1. **启动 navsim-online**
   ```bash
   cd navsim-online
   ./run_navsim.sh
   ```

2. **在 Web 界面添加静态障碍物**
   - 打开 `http://localhost:8080`
   - 勾选"静态圆形"
   - 点击"放置"按钮
   - 在场景中点击几个位置
   - 点击"提交地图"按钮

3. **启动 navsim-local**
   ```bash
   cd navsim-local
   ./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json
   ```

4. **查看日志输出**
   
   应该看到：
   ```
   [Room demo] New client connected, will send static map in next tick
   [BEVExtractor] Has static_map: 1  ← 修复成功！
   [BEVExtractor] StaticMap circles: 5
   [BEVExtractor] Extracted circles: 3  ← 成功提取！
   ```

5. **查看可视化窗口**
   
   应该看到：
   - 🟠 橙色圆形 - 静态障碍物
   - 🟢 绿色圆形 + 箭头 - 自车
   - 🔴 红色圆形 - 目标点
   - 🔵 青色线条 - 规划轨迹

---

## 📊 调试日志对比

### 修复前

```
[BEVExtractor] ========== Extract called ==========
[BEVExtractor] WorldTick tick_id: 19549
[BEVExtractor] Has static_map: 0  ← 问题！
[BEVExtractor] Dynamic obstacles count: 6
[BEVExtractor] No cached static map, skipping static obstacles
[BEVExtractor] ========== Extract result ==========
[BEVExtractor] Extracted circles: 0  ← 结果为 0
[BEVExtractor] Extracted rectangles: 0
[BEVExtractor] Extracted polygons: 0
```

### 修复后（预期）

```
[Room demo] New client connected, will send static map in next tick  ← 修复生效！
[BEVExtractor] ========== Extract called ==========
[BEVExtractor] WorldTick tick_id: 19550
[BEVExtractor] Has static_map: 1  ← 收到静态地图！
[BEVExtractor] Dynamic obstacles count: 6
[BEVExtractor] StaticMap circles: 5  ← 有障碍物数据！
[BEVExtractor] StaticMap polygons: 0
[BEVExtractor] Processing static obstacles...
[BEVExtractor]   Ego position: (33.0, 33.4)
[BEVExtractor]   Detection range: 50.0 m
[BEVExtractor]   Cached circles: 5
[BEVExtractor]   Cached polygons: 0
[BEVExtractor]   Static circles in range: 3  ← 在范围内的障碍物！
[BEVExtractor]   Static polygons in range: 0
[BEVExtractor] ========== Extract result ==========
[BEVExtractor] Extracted circles: 3  ← 成功提取！
[BEVExtractor] Extracted rectangles: 0
[BEVExtractor] Extracted polygons: 0
```

---

## 📁 修改的文件

### navsim-online

1. **server/main.py**
   - 修改 `register()` 方法
   - 在新客户端连接时设置 `include_static_next_tick = True`

### navsim-local

1. **src/plugin/preprocessing/bev_extractor.cpp**
   - 添加调试日志（输入数据检查、提取过程、输出结果）

2. **src/plugin/preprocessing/preprocessing_pipeline.cpp**
   - 添加调试日志（BEV 提取结果检查）

3. **src/core/algorithm_manager.cpp**
   - 添加调试日志（PerceptionInput 检查、可视化调用确认）

4. **src/viz/imgui_visualizer.cpp**
   - 添加调试日志（传入数据、缓存数据检查）

### 新增文件

1. **STATIC_OBSTACLES_FIX.md** - 详细的修复报告
2. **test_static_obstacles.sh** - 自动化测试脚本
3. **DEBUGGING_SUMMARY.md** - 本文档

---

## 🎯 关键经验

### 1. 系统化调试方法

按照数据流的顺序，逐步添加日志：
1. **输入数据检查** - 确认数据是否正确接收
2. **处理过程检查** - 确认处理逻辑是否正确
3. **输出结果检查** - 确认输出是否符合预期

### 2. 跨模块问题定位

问题可能不在当前模块，需要：
- 检查上游数据源（navsim-online）
- 检查数据传输协议（WebSocket、Protobuf）
- 检查下游数据消费（可视化）

### 3. 日志的重要性

详细的日志可以：
- 快速定位问题
- 验证修复效果
- 帮助理解数据流

### 4. 文档的价值

完整的文档可以：
- 记录问题和解决方案
- 帮助后续维护
- 避免重复踩坑

---

## 🔧 后续工作

### 1. 移除调试日志（可选）

调试日志会影响性能，可以：
- 完全移除
- 使用条件编译（`#ifdef DEBUG_BEV_EXTRACTION`）
- 使用日志级别控制

### 2. 添加单元测试

为 BEVExtractor 添加单元测试：
```cpp
TEST(BEVExtractorTest, ExtractStaticCircles) {
  // 创建包含静态地图的 WorldTick
  // 调用 extract()
  // 验证提取结果
}
```

### 3. 添加默认测试地图

在 navsim-online 初始化时添加默认障碍物，方便测试。

### 4. 优化静态地图缓存

添加版本检查，只在版本变化时更新缓存。

---

## 📝 总结

### 问题根源
- navsim-online 只在特定时机发送静态地图
- 后连接的客户端会错过第一个包含静态地图的 tick

### 解决方案
- 在新客户端连接时，强制在下一个 tick 发送静态地图
- 修改位置：`navsim-online/server/main.py` 的 `register()` 方法

### 调试方法
- 系统化添加日志，逐步排查数据流
- 跨模块分析，找到问题根源
- 详细记录过程，形成文档

### 验证方法
- 自动化测试脚本
- 手动验证步骤
- 日志输出对比

---

**调试状态**：✅ 已完成  
**修复状态**：✅ 已完成  
**测试状态**：⏳ 待用户验证  
**文档状态**：✅ 已完成  

---

**调试时间**：约 30 分钟  
**修改行数**：约 50 行（日志） + 3 行（修复）  
**文档行数**：约 500 行  


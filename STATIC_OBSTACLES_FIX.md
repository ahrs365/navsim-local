# 静态障碍物显示问题修复报告

## 🔍 问题诊断

### 问题现象
- 可视化窗口中看不到静态障碍物
- 右侧信息栏显示 `bev_obstacles: 0`（障碍物数量为 0）

### 根本原因

通过添加详细的调试日志，我们发现了问题的根源：

```
[BEVExtractor] Has static_map: 0
[BEVExtractor] No cached static map, skipping static obstacles
```

**WorldTick 中没有包含静态地图数据！**

### 数据流分析

1. ✅ **navsim-online → navsim-local**：WebSocket 连接正常
2. ✅ **WorldTick 接收**：每个 tick 都正常接收
3. ❌ **静态地图缺失**：`world_tick.has_static_map() == false`
4. ❌ **BEV 提取失败**：没有缓存的静态地图，跳过提取
5. ❌ **可视化为空**：`bev_obstacles.circles.size() == 0`

### 为什么静态地图缺失？

查看 `navsim-online/server/main.py` 的代码（第 195-198 行）：

```python
map_payload = {"version": self.map_version}
if self.include_static_next_tick:
    map_payload["static"] = self.static_geometry
    self.include_static_next_tick = False  # 发送后立即设置为 False
```

**静态地图只在 `include_static_next_tick == True` 时发送，并且发送后立即设置为 `False`**。

这个标志只在以下情况设置为 `True`：
1. **初始化时**（第 119 行）：`include_static_next_tick: bool = True`
2. **地图更新时**（第 291 行）：用户在 Web 界面更新地图
3. **仿真重置时**（第 496 行）：用户点击"重置"按钮

**问题**：如果 navsim-local 在 navsim-online 启动后才连接，就会错过第一个包含静态地图的 tick！

---

## ✅ 解决方案

### 方案 1：修改 navsim-online（已实施）

**修改位置**：`navsim-online/server/main.py` 第 138-151 行

**修改内容**：在新客户端连接时，强制在下一个 tick 发送静态地图

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

## 🧪 验证步骤

### 步骤 1：启动 navsim-online

```bash
cd navsim-online
./run_navsim.sh
```

### 步骤 2：在 Web 界面添加静态障碍物

1. 打开浏览器访问 `http://localhost:8080`
2. 在左侧工具栏找到"障碍物编辑"部分
3. 勾选"静态圆形"
4. 点击"放置"按钮
5. 在场景中点击几个位置，添加几个静态圆形障碍物
6. 点击"提交地图"按钮

### 步骤 3：启动 navsim-local（带可视化）

```bash
cd navsim-local
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/with_visualization.json
```

### 步骤 4：查看日志输出

应该看到类似这样的日志：

```
[Room demo] New client connected, will send static map in next tick
[BEVExtractor] ========== Extract called ==========
[BEVExtractor] WorldTick tick_id: 12345
[BEVExtractor] Has static_map: 1  ← 应该是 1！
[BEVExtractor] Dynamic obstacles count: 6
[BEVExtractor] StaticMap circles: 5  ← 应该有圆形障碍物！
[BEVExtractor] StaticMap polygons: 0
[BEVExtractor] Processing static obstacles...
[BEVExtractor]   Ego position: (x, y)
[BEVExtractor]   Detection range: 50.0 m
[BEVExtractor]   Cached circles: 5
[BEVExtractor]   Cached polygons: 0
[BEVExtractor]   Static circles in range: 3  ← 应该有在范围内的障碍物！
[BEVExtractor]   Static polygons in range: 0
[BEVExtractor] ========== Extract result ==========
[BEVExtractor] Extracted circles: 3  ← 应该 > 0！
[BEVExtractor] Extracted rectangles: 0
[BEVExtractor] Extracted polygons: 0
```

### 步骤 5：查看可视化窗口

应该能看到：
- 🟠 **橙色圆形** - 静态障碍物
- 🟢 **绿色圆形 + 箭头** - 自车
- 🔴 **红色圆形** - 目标点
- 🔵 **青色线条** - 规划轨迹

---

## 📊 调试日志说明

### 添加的调试日志位置

1. **BEVExtractor** (`navsim-local/src/plugin/preprocessing/bev_extractor.cpp`)
   - 输入数据检查：WorldTick 是否包含静态地图
   - 提取过程：静态圆形、多边形的数量和范围检查
   - 输出结果：提取到的障碍物数量

2. **PreprocessingPipeline** (`navsim-local/src/plugin/preprocessing/preprocessing_pipeline.cpp`)
   - BEV 提取结果检查
   - 障碍物数量统计

3. **AlgorithmManager** (`navsim-local/src/core/algorithm_manager.cpp`)
   - PerceptionInput 中的障碍物数量
   - 可视化函数调用确认

4. **ImGuiVisualizer** (`navsim-local/src/viz/imgui_visualizer.cpp`)
   - 传入的障碍物数量
   - 缓存后的障碍物数量

### 日志输出示例（修复前）

```
[BEVExtractor] Has static_map: 0  ← 问题！
[BEVExtractor] No cached static map, skipping static obstacles
[BEVExtractor] Extracted circles: 0  ← 结果为 0
```

### 日志输出示例（修复后）

```
[Room demo] New client connected, will send static map in next tick  ← 修复生效！
[BEVExtractor] Has static_map: 1  ← 收到静态地图！
[BEVExtractor] StaticMap circles: 5  ← 有障碍物数据！
[BEVExtractor] Extracted circles: 3  ← 成功提取！
```

---

## 🔧 后续优化建议

### 1. 移除调试日志（生产环境）

调试日志会影响性能，建议在验证修复后移除或使用条件编译：

```cpp
#ifdef DEBUG_BEV_EXTRACTION
  std::cout << "[BEVExtractor] ..." << std::endl;
#endif
```

### 2. 添加静态地图版本检查

在 `BEVExtractor` 中缓存静态地图版本号，只在版本变化时更新缓存：

```cpp
if (world_tick.has_static_map()) {
  uint32_t new_version = world_tick.static_map().version();
  if (new_version != cached_map_version_) {
    cached_static_map_ = world_tick.static_map();
    cached_map_version_ = new_version;
    has_cached_static_map_ = true;
    std::cout << "[BEVExtractor] Updated static map cache (version " 
              << new_version << ")" << std::endl;
  }
}
```

### 3. 添加默认测试地图

在 navsim-online 初始化时，添加一些默认的静态障碍物，方便测试：

```python
static_geometry: Dict[str, Any] = field(
    default_factory=lambda: {
        "polygons": [],
        "circles": [
            {"x": 5.0, "y": 5.0, "r": 0.5},
            {"x": 10.0, "y": 3.0, "r": 0.8},
            {"x": 15.0, "y": 8.0, "r": 0.6},
        ],
        "origin": {"x": 0.0, "y": 0.0},
        "resolution": 0.1,
    }
)
```

### 4. 添加可视化配置选项

在 `config/with_visualization.json` 中添加障碍物显示配置：

```json
{
  "visualization": {
    "enabled": true,
    "obstacles": {
      "show_static": true,
      "show_dynamic": true,
      "static_color": "orange",
      "dynamic_color": "red",
      "opacity": 0.7
    }
  }
}
```

---

## 📝 总结

### 问题根源
- navsim-online 只在特定时机发送静态地图（初始化、地图更新、重置）
- 后连接的客户端会错过第一个包含静态地图的 tick

### 解决方案
- 在新客户端连接时，强制在下一个 tick 发送静态地图
- 修改位置：`navsim-online/server/main.py` 的 `register()` 方法

### 验证方法
1. 在 Web 界面添加静态障碍物
2. 启动 navsim-local
3. 查看日志确认收到静态地图
4. 查看可视化窗口确认显示障碍物

### 后续工作
- 移除调试日志（或使用条件编译）
- 添加静态地图版本检查
- 添加默认测试地图
- 添加可视化配置选项

---

**修复状态**：✅ 已完成
**测试状态**：⏳ 待验证
**文档状态**：✅ 已完成


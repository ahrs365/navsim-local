# 可视化线程模型修复

## 📝 问题描述

启动本地可视化模式后，规划日志已经在打印，但可视化窗口是黑屏。

## 🔍 根本原因

**SDL2 线程安全违规**：

```
主线程                          仿真线程
  |                                |
  | SDL_CreateWindow() ✅          |
  | 窗口在主线程创建                |
  |                                |
  | 启动仿真线程 ----------------> |
  |                                | beginFrame() ❌
  |                                | SDL_PollEvent() ❌
  |                                | endFrame() ❌
  |                                | SDL_RenderPresent() ❌
  | sleep(100ms)                   |
  | 主线程空闲 ⏸️                  | 跨线程渲染失败
  ↓                                ↓
  
结果：窗口黑屏 ❌
```

**SDL2 的要求**：
> 窗口的创建、事件处理和渲染必须在同一个线程中

## ✅ 解决方案：方案1 - 条件线程模型

根据是否启用可视化，选择不同的线程模型：

### 可视化模式（主线程运行）

```
主线程
  |
  | SDL_CreateWindow() ✅
  | 窗口在主线程创建
  |
  | run_simulation_loop() ✅
  | 仿真循环在主线程运行
  |   |
  |   | beginFrame() ✅
  |   | SDL_PollEvent() ✅
  |   | 规划算法执行 ✅
  |   | drawXXX() ✅
  |   | endFrame() ✅
  |   | SDL_RenderPresent() ✅
  |   |
  |   | 所有操作在同一线程 ✅
  ↓
  
结果：窗口正常显示 ✅
```

### 无可视化模式（仿真线程运行）

```
主线程                    仿真线程
  |                          |
  | 启动仿真线程 ----------> |
  |                          | run_simulation_loop()
  |                          | 规划算法执行
  | while (!interrupt) {     |
  |   sleep(100ms)           |
  | }                        |
  |                          |
  | stop_simulation_loop()   |
  | join()                   |
  ↓                          ↓
  
结果：性能最优 ✅
```

## 🔧 代码修改

### 1. 修改 `navsim_algo.cpp`

```cpp
// 5. 运行仿真循环
// 根据是否启用可视化选择不同的线程模型
if (args.enable_visualization) {
  // 🎨 可视化模式：主线程运行仿真循环
  // SDL2 要求窗口的创建、事件处理和渲染必须在同一个线程中
  std::cout << "[Main] Running simulation loop in main thread (visualization enabled)" << std::endl;
  std::cout << "[Main] Press Ctrl+C or close the window to stop" << std::endl;
  
  // 传递 g_interrupt 信号，让仿真循环能够响应 Ctrl+C
  algorithm_manager.run_simulation_loop(&navsim::g_interrupt);
  
  std::cout << "[Main] Local simulation ended" << std::endl;
} else {
  // 无可视化模式：仿真循环在单独的线程中运行
  std::cout << "[Main] Running simulation loop in separate thread (no visualization)" << std::endl;
  std::cout << "[Main] Press Ctrl+C to stop" << std::endl;
  
  std::thread sim_thread([&algorithm_manager]() {
    algorithm_manager.run_simulation_loop();
  });

  // 等待中断信号
  while (!navsim::g_interrupt.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "[Main] Shutting down..." << std::endl;

  // 停止仿真循环
  algorithm_manager.stop_simulation_loop();

  // 清理
  sim_thread.join();

  std::cout << "[Main] Local simulation ended" << std::endl;
}
```

### 2. 修改 `algorithm_manager.hpp`

```cpp
/**
 * @brief 运行本地仿真循环（新的主循环）
 * 集成本地仿真器，在同一进程内运行仿真和算法
 * @param external_interrupt 外部中断标志（可选，用于响应 Ctrl+C 等信号）
 * @return 是否成功启动
 */
bool run_simulation_loop(const std::atomic<bool>* external_interrupt = nullptr);
```

### 3. 修改 `algorithm_manager.cpp`

```cpp
bool AlgorithmManager::run_simulation_loop(const std::atomic<bool>* external_interrupt) {
  // ... 初始化代码 ...
  
  while (!simulation_should_stop_.load()) {
    // 🎨 检查可视化窗口是否被关闭
    if (visualizer_ && visualizer_->shouldClose()) {
      std::cout << "[AlgorithmManager] Visualizer window closed, stopping simulation..." << std::endl;
      break;
    }

    // 🛑 检查外部中断信号（Ctrl+C）
    if (external_interrupt && external_interrupt->load()) {
      std::cout << "[AlgorithmManager] External interrupt received, stopping simulation..." << std::endl;
      break;
    }

    // ... 仿真循环逻辑 ...
  }
  
  return true;
}
```

## 🎯 修复效果

### 修复前

```
启动程序
  ↓
主线程：创建 SDL2 窗口
  ↓
主线程：启动仿真线程
  ↓
仿真线程：尝试渲染 ❌ 跨线程操作失败
  ↓
窗口黑屏 ❌
  ↓
规划日志打印 ✅（但窗口仍然黑屏）
```

### 修复后

```
启动程序
  ↓
主线程：创建 SDL2 窗口
  ↓
主线程：运行仿真循环
  ↓
主线程：第一帧渲染 ✅ 同一线程，立即成功
  ↓
窗口立即显示内容 ✅
  ↓
规划日志和可视化同步更新 ✅
```

## 🚀 测试步骤

### 1. 重新编译

```bash
cd navsim-local/build
make -j$(nproc)
```

### 2. 运行可视化模式

```bash
./navsim_algo --local-sim --scenario=../scenarios/map1.json --visualize
```

**预期输出**：

```
=== NavSim Local Simulation Mode ===
Scenario: ../scenarios/map1.json
Visualization: ENABLED (ImGui)
====================================
[Main] Starting local simulation...
[Main] Running simulation loop in main thread (visualization enabled)
[Main] Press Ctrl+C or close the window to stop
[AlgorithmManager] Starting local simulation loop...
```

**预期效果**：
- ✅ 窗口立即显示内容（无黑屏）
- ✅ 规划日志和可视化同步更新
- ✅ 无闪烁现象
- ✅ Ctrl+C 或关闭窗口都能正常退出

### 3. 运行无可视化模式

```bash
./navsim_algo --local-sim --scenario=../scenarios/map1.json
```

**预期输出**：

```
=== NavSim Local Simulation Mode ===
Scenario: ../scenarios/map1.json
Visualization: DISABLED
====================================
[Main] Starting local simulation...
[Main] Running simulation loop in separate thread (no visualization)
[Main] Press Ctrl+C to stop
```

**预期效果**：
- ✅ 仿真在单独线程运行
- ✅ 主线程等待中断信号
- ✅ Ctrl+C 正常退出

## 📊 技术细节

### 为什么可视化模式必须在主线程？

**SDL2 的设计哲学**：
- SDL2 是为游戏设计的，游戏通常在主线程运行主循环
- 事件循环（`SDL_PollEvent`）必须在创建窗口的线程中调用
- 渲染（`SDL_RenderPresent`）必须在创建渲染器的线程中调用

**违反规则的后果**：
- 渲染失败（黑屏）
- 不稳定的显示（闪烁）
- 可能的崩溃

### 为什么无可视化模式仍然使用单独的线程？

**原因**：
1. **保持主线程响应**：主线程可以快速响应 Ctrl+C 信号
2. **架构一致性**：与 WebSocket 模式保持一致
3. **未来扩展**：可以在主线程添加其他功能（如命令行交互）

## 🎓 设计原则

1. **SDL2 线程安全规则**：窗口的创建、事件处理和渲染必须在同一个线程中
2. **条件线程模型**：根据运行模式选择最合适的线程模型
3. **信号传递**：通过 `external_interrupt` 参数传递外部中断信号
4. **优雅退出**：支持 Ctrl+C 和关闭窗口两种退出方式

## 📝 相关文件

- `navsim-local/apps/navsim_algo.cpp` - 主程序入口，线程模型选择
- `navsim-local/platform/include/core/algorithm_manager.hpp` - 接口定义
- `navsim-local/platform/src/core/algorithm_manager.cpp` - 仿真循环实现
- `navsim-local/platform/src/viz/imgui_visualizer.cpp` - SDL2 可视化实现

## ✅ 总结

通过采用**条件线程模型**，我们解决了 SDL2 线程安全问题：

- **可视化模式**：主线程运行仿真循环，确保 SDL2 所有操作在同一线程
- **无可视化模式**：仿真线程运行循环，保持性能和架构一致性

这个方案简单、有效，符合 SDL2 的设计要求，同时保持了代码的清晰性和可维护性。


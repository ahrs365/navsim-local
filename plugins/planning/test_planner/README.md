# TestPlanner Planner Plugin

Test planner for scaffolding tool validation

## 📋 概述

- **插件名称**: TestPlanner
- **插件类型**: 规划器 (Planner)
- **版本**: 1.0.0
- **作者**: NavSim Team

## 🏗️ 架构

本插件采用三层解耦架构：

```
test_planner_plugin/
├── algorithm/              # 算法层（纯算法，无平台依赖）
│   ├── test_planner.hpp
│   └── test_planner.cpp
├── adapter/                # 适配器层（平台接口适配）
│   ├── test_planner_plugin.hpp
│   ├── test_planner_plugin.cpp
│   └── register.cpp
├── tests/                  # 测试（可选）
│   └── test_test_planner.cpp
├── CMakeLists.txt
└── README.md
```

### 算法层 (algorithm/)

- **职责**: 实现核心规划算法
- **依赖**: 仅依赖 Eigen 和 STL
- **特点**: 可复用到其他项目

### 适配器层 (adapter/)

- **职责**: 实现平台插件接口，转换数据格式
- **依赖**: 依赖平台 API 和算法层
- **特点**: 薄适配层，逻辑在算法层

## 🚀 快速开始

### 编译

```bash
cd navsim-local
mkdir -p build && cd build
cmake ..
make test_planner_plugin
```

### 使用

```bash
# 使用本地调试工具测试
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --planner TestPlanner
```

## ⚙️ 配置参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_velocity` | double | 2.0 | 最大速度 (m/s) |
| `max_acceleration` | double | 2.0 | 最大加速度 (m/s²) |
| `step_size` | double | 0.1 | 步长 (m) |
| `max_iterations` | int | 1000 | 最大迭代次数 |

### 配置示例

```json
{
  "TestPlanner": {
    "max_velocity": 3.0,
    "max_acceleration": 2.5,
    "step_size": 0.05,
    "max_iterations": 5000
  }
}
```

## 📊 性能

TODO: 添加性能测试结果

## 🧪 测试

```bash
# 运行单元测试
cd build
ctest -R test_planner
```

## 📝 开发指南

### 修改算法

1. 编辑 `algorithm/test_planner.cpp` 中的 `plan()` 方法
2. 添加您的算法逻辑
3. 重新编译并测试

### 添加配置参数

1. 在 `algorithm/test_planner.hpp` 的 `Config` 结构体中添加参数
2. 在 `Config::fromJson()` 中添加 JSON 解析逻辑
3. 在 `adapter/test_planner_plugin.cpp` 的 `initialize()` 中打印新参数

### 添加依赖

如果需要额外的依赖（如栅格地图、ESDF 等）：

1. 在 `adapter/test_planner_plugin.hpp` 的 `getMetadata()` 中设置：
   ```cpp
   metadata.requires_occupancy_grid = true;
   metadata.requires_esdf_map = true;
   ```

2. 在 `plan()` 方法中检查依赖：
   ```cpp
   if (!context.occupancy_grid) {
     result.failure_reason = "No occupancy grid available";
     return false;
   }
   ```

## 📚 相关文档

- [插件开发指南](../../docs/PLUGIN_DEVELOPMENT.md)
- [三层架构说明](../../REFACTORING_PROPOSAL.md)
- [本地调试模式](../../docs/LOCAL_DEBUG_MODE.md)

## 📄 许可证

TODO: 添加许可证信息


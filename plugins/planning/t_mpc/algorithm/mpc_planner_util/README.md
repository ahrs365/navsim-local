# mpc_planner_util

MPC 规划器工具库 - 纯 C++ 实现（无 ROS 依赖）

## 概述

这是一个纯 C++ 的 MPC 规划器工具库，提供参数管理和可视化接口功能。已完全移除 ROS 依赖，可以在非 ROS 环境中使用。

## 功能模块

### 1. 参数管理 (parameters.h + load_yaml.hpp)
- YAML 配置文件加载
- 单例模式配置管理
- 无 ROS 依赖

### 2. 可视化接口 (data_visualization.h/cpp)
- 提供空实现的可视化接口
- 兼容原有 API
- 在非 ROS 环境下不执行实际可视化

## 依赖

### 必需依赖
- **Eigen3**: 线性代数库
- **yaml-cpp**: YAML 配置文件解析（位于 `../third_party/yaml-cpp`）
- **ros_tools_no_ros**: 无 ROS 版本的 ros_tools 库
- **mpc_planner_types**: MPC 规划器数据类型定义

### 可选依赖
- 无

## 编译

### 前置条件

1. 确保 `ros_tools_no_ros` 已编译：
```bash
cd ../ros_tools_no_ros
./build.sh
```

2. 确保 `third_party/yaml-cpp` 存在

### 编译步骤

```bash
./build.sh
```

或手动编译：

```bash
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### 编译输出

- 共享库: `build/libmpc_planner_util.so`

## 使用示例

```cpp
#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/data_visualization.h>

int main() {
    // 初始化配置
    Configuration::getInstance().initialize("config.yaml");
    
    // 访问配置参数
    auto& config = CONFIG;
    double param = config["some_param"].as<double>();
    
    // 使用可视化接口（空实现）
    MPCPlanner::visualizeTrajectory(trajectory, "trajectory");
    
    return 0;
}
```

## 目录结构

```
mpc_planner_util/
├── CMakeLists.txt              # CMake 配置文件
├── build.sh                    # 构建脚本
├── README.md                   # 本文件
├── include/
│   └── mpc_planner_util/
│       ├── data_visualization.h      # 可视化接口
│       ├── load_yaml.hpp             # YAML 加载工具
│       ├── parameters.h              # 参数管理
│       └── visualization_dummy.h     # 空可视化实现
└── src/
    └── data_visualization.cpp        # 可视化实现
```

## 与 ROS 版本的区别

| 特性 | ROS 版本 | 纯 C++ 版本 |
|------|---------|------------|
| 编译系统 | catkin/ament_cmake | 纯 CMake |
| 日志系统 | ROS 日志 | ros_tools_no_ros 日志 |
| 可视化 | ROS Marker | 空实现 |
| 依赖 | roscpp/rclcpp | 仅 C++ 库 |

## 技术细节

### 条件编译

代码使用条件编译支持 ROS 和非 ROS 环境：

```cpp
#ifdef MPC_PLANNER_ROS
  // ROS 版本代码
#else
  // 纯 C++ 版本代码
#endif
```

在纯 C++ 版本中，不定义 `MPC_PLANNER_ROS` 宏。

### 空可视化实现

`visualization_dummy.h` 提供了与 ROS 版本兼容的空接口：
- `RosTools::ROSMarkerPublisher` - 空发布器
- `RosTools::DummyMarker` - 空标记
- `VISUALS` - 全局可视化管理器

所有可视化函数调用都会被编译但不执行实际操作。

## 注意事项

1. **可视化功能**: 当前版本的可视化函数为空实现，不会产生任何可视化输出
2. **依赖顺序**: 必须先编译 `ros_tools_no_ros`，再编译本库
3. **yaml-cpp**: 使用项目中的 third_party 版本，不依赖系统安装

## 未来改进

- [ ] 添加非 ROS 可视化后端（如 matplotlib-cpp）
- [ ] 提供更多配置管理工具
- [ ] 添加单元测试
- [ ] 提供 Python 绑定

## 许可证

与主项目相同

## 贡献

欢迎提交 Issue 和 Pull Request


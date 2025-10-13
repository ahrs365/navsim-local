# NavSim CMake 配置文件

## ⚠️ 当前状态：未使用

这些 CMake 配置文件**目前没有被使用**。

---

## 📁 文件说明

### 1. `NavSimPluginConfig.cmake.in`

**用途**: 外部插件开发者使用 `find_package(NavSim)` 时的配置模板

**预期用法**:
```cmake
# 外部插件的 CMakeLists.txt
find_package(NavSim REQUIRED)
target_link_libraries(my_plugin PUBLIC NavSim::navsim_plugin_framework)
```

**当前状态**: ❌ 未安装，无法使用

### 2. `NavSimPluginHelpers.cmake`

**用途**: 提供便捷的 CMake 函数简化插件开发

**预期用法**:
```cmake
navsim_add_perception_plugin(
    NAME my_plugin
    SOURCES src/my_plugin.cpp
    HEADERS include/my_plugin.hpp
)
```

**当前状态**: ❌ 未被包含，无法使用

---

## 🤔 为什么没有使用？

### 1. **内置插件不需要**

内置插件（`plugins/` 目录）直接在主项目中编译：

```cmake
# plugins/perception/grid_map_builder/CMakeLists.txt
add_library(grid_map_builder_plugin SHARED
    src/grid_map_builder_plugin.cpp
    src/register.cpp)

target_link_libraries(grid_map_builder_plugin
    PUBLIC navsim_plugin_framework)
```

不需要 `find_package(NavSim)` 或辅助函数。

### 2. **没有安装配置**

主 `CMakeLists.txt` 中没有：
- `install(TARGETS ...)` - 安装库文件
- `install(EXPORT ...)` - 导出目标
- `install(FILES ...)` - 安装 CMake 配置文件

### 3. **外部插件可以直接使用动态加载**

外部开发者可以：
1. 编译插件为 `.so` 文件
2. 复制到 `plugins/` 目录
3. 在配置文件中启用

不需要 `find_package(NavSim)`。

---

## 💡 建议

### **选项 1: 删除这些文件（推荐）** ✅

**原因**:
- 当前项目不需要
- 外部插件通过动态加载即可使用
- 减少维护负担

**操作**:
```bash
# 删除整个 cmake 目录
$ rm -rf cmake/
```

### **选项 2: 完善安装配置（如果需要）**

如果你想支持外部开发者使用 `find_package(NavSim)`，需要在主 `CMakeLists.txt` 中添加：

```cmake
# 1. 安装库文件
install(TARGETS navsim_plugin_framework navsim_proto
    EXPORT NavSimTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

# 2. 安装头文件
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp")

# 3. 导出目标
install(EXPORT NavSimTargets
    FILE NavSimTargets.cmake
    NAMESPACE NavSim::
    DESTINATION lib/cmake/NavSim)

# 4. 配置并安装 Config 文件
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/NavSimPluginConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/NavSimConfig.cmake
    INSTALL_DESTINATION lib/cmake/NavSim)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/NavSimConfig.cmake
    cmake/NavSimPluginHelpers.cmake
    DESTINATION lib/cmake/NavSim)
```

**但这需要大量工作，且当前不需要。**

---

## 🎯 推荐方案

### **使用动态插件加载（已实现）**

外部开发者开发插件的步骤：

#### 1. 创建插件项目

```bash
my_custom_plugin/
├── CMakeLists.txt
├── include/
│   └── my_custom_plugin.hpp
└── src/
    ├── my_custom_plugin.cpp
    └── register.cpp
```

#### 2. 编写 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_custom_plugin)

# 直接指定 NavSim 的头文件路径
set(NAVSIM_INCLUDE_DIR "/path/to/navsim-local/include")

# 创建动态库
add_library(my_custom_plugin SHARED
    src/my_custom_plugin.cpp
    src/register.cpp)

target_include_directories(my_custom_plugin
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${NAVSIM_INCLUDE_DIR})

# 设置动态库版本
set_target_properties(my_custom_plugin PROPERTIES
    OUTPUT_NAME "my_custom_plugin"
    VERSION 1.0.0
    SOVERSION 1)

target_compile_features(my_custom_plugin PUBLIC cxx_std_17)
```

#### 3. 编译插件

```bash
$ cmake -B build
$ cmake --build build
# 生成 libmy_custom_plugin.so
```

#### 4. 安装插件

```bash
# 复制到 NavSim 插件目录
$ cp build/libmy_custom_plugin.so /path/to/navsim-local/build/plugins/
```

#### 5. 配置使用

```json
// config/default.json
{
  "perception_plugins": [
    {
      "name": "MyCustomPlugin",
      "enabled": true
    }
  ]
}
```

#### 6. 运行

```bash
$ ./navsim_algo
[DynamicPluginLoader] Loading plugin 'MyCustomPlugin' from: ./build/plugins/libmy_custom_plugin.so
[DynamicPluginLoader] Successfully loaded plugin: MyCustomPlugin
```

**无需 `find_package(NavSim)`！**

---

## 📊 对比

| 方案 | 优点 | 缺点 | 推荐 |
|------|------|------|------|
| **删除 cmake/** | 简单，减少维护 | 无 | ✅ **推荐** |
| **完善安装配置** | 更"正规" | 工作量大，当前不需要 | ❌ 不推荐 |
| **保持现状** | 无需改动 | 文件无用，容易混淆 | ⚠️ 可接受 |

---

## 🚀 行动建议

### **立即执行**

```bash
# 删除未使用的 cmake 目录
$ cd /home/gao/workspace/pnc_project/ahrs-simulator/navsim-local
$ rm -rf cmake/

# 提交更改
$ git add -A
$ git commit -m "chore: Remove unused CMake config files

These files were intended for external plugin developers using
find_package(NavSim), but are not needed because:
1. Built-in plugins are compiled directly in the main project
2. External plugins can use dynamic loading without find_package
3. Reduces maintenance burden"
```

### **或者保留（如果未来需要）**

如果你认为未来可能需要支持 `find_package(NavSim)`，可以：

1. 保留这些文件
2. 在文件顶部添加注释说明当前未使用
3. 创建 TODO 任务

---

## ❓ 常见问题

### Q: 外部插件开发者怎么办？

**A**: 使用动态加载，不需要 `find_package(NavSim)`。参考 `external_plugins/README.md`。

### Q: 这些文件会影响编译吗？

**A**: 不会。它们只是模板文件，不参与编译。

### Q: 删除后能恢复吗？

**A**: 可以。Git 历史中有这些文件，随时可以恢复。

---

**最后更新**: 2025-10-13  
**建议**: 删除这些文件，使用动态插件加载


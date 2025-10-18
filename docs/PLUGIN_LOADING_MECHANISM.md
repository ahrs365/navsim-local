# 插件加载和注册机制详解

本文档详细解释 navsim-local 的插件加载和注册机制，包括设计原理、实现细节和常见问题。

---

## 📋 目录

1. [设计目标](#设计目标)
2. [统一加载方式](#统一加载方式)
3. [双重注册机制](#双重注册机制)
4. [ESDFBuilder 警告问题分析](#esdfbuilder-警告问题分析)
5. [解决方案](#解决方案)
6. [最佳实践](#最佳实践)

---

## 🎯 设计目标

根据 `REFACTORING_PROPOSAL.md`，插件系统的设计目标是：

### ✅ 已实现的目标

1. **统一加载方式**: 所有插件都编译为 `.so` 动态库
2. **动态加载**: 使用 `dlopen/dlsym` 在运行时加载插件
3. **短名称支持**: 支持 `JpsPlanner` 而不是完整路径
4. **灵活的搜索路径**: 支持多个插件目录
5. **共享注册表**: 所有插件共享同一个注册表单例

### 🔄 设计权衡

为了同时支持**动态加载**和**静态链接**，系统采用了**双重注册机制**：
- **动态注册**: 通过 `extern "C"` 导出的注册函数
- **静态注册**: 通过静态初始化器自动注册

---

## ✅ 统一加载方式

### 1. 所有插件都是 `.so` 文件

**验证**:
```bash
$ find build/plugins -name "*.so" -type f
build/plugins/planning/straight_line/libstraight_line_planner_plugin.so
build/plugins/planning/astar/liba_star_planner_plugin.so
build/plugins/planning/jps_planner_plugin/libjps_planner_plugin.so
build/plugins/perception/grid_map_builder/libgrid_map_builder_plugin.so
build/plugins/perception/esdf_builder/libesdf_builder_plugin.so
```

✅ **结论**: 所有 5 个插件都已编译为 `.so` 动态库。

---

### 2. 动态加载机制

**实现**: `DynamicPluginLoader::loadPlugin()`

```cpp
// 1. 解析插件路径（支持短名称）
std::string lib_path = resolvePluginPath(plugin_name);

// 2. 使用 dlopen 加载动态库
void* handle = dlopen(lib_path.c_str(), RTLD_NOW | RTLD_GLOBAL);

// 3. 查找注册函数
std::string register_func_name = "register" + plugin_name + "Plugin";
RegisterFunc register_func = (RegisterFunc)dlsym(handle, register_func_name.c_str());

// 4. 调用注册函数（如果找到）
if (register_func) {
  register_func();
}
```

**关键参数**:
- `RTLD_NOW`: 立即解析所有符号（而不是延迟解析）
- `RTLD_GLOBAL`: 使符号对后续加载的库可用（重要！确保共享注册表）

✅ **结论**: 动态加载机制已完全实现并正常工作。

---

### 3. 短名称解析

**实现**: `DynamicPluginLoader::resolvePluginPath()`

**转换规则**:
```
插件名称 (CamelCase)  →  库文件名 (snake_case)
─────────────────────────────────────────────
GridMapBuilder        →  libgrid_map_builder_plugin.so
AStarPlanner          →  liba_star_planner_plugin.so
ESDFBuilder           →  libesdf_builder_plugin.so
JpsPlanner            →  libjps_planner_plugin.so
```

**搜索顺序**:
1. `build/plugins/planning/` (递归)
2. `plugins/planning/` (递归)
3. `build/plugins/perception/` (递归)
4. `plugins/perception/` (递归)
5. `~/.navsim/plugins/`
6. `./external_plugins/{name}/build/`
7. `$NAVSIM_PLUGIN_PATH/`

✅ **结论**: 短名称解析已完全实现，支持灵活的插件组织结构。

---

## 🔄 双重注册机制

### 为什么需要双重注册？

系统设计支持两种使用场景：

| 场景 | 链接方式 | 注册方式 | 使用场景 |
|------|---------|---------|---------|
| **场景 1** | 动态加载 | 动态注册 | `navsim_local_debug` 运行时加载插件 |
| **场景 2** | 静态链接 | 静态注册 | `navsim_planning` 编译时链接插件 |

### 动态注册 (Dynamic Registration)

**代码**:
```cpp
// 导出 C 风格的注册函数，供动态加载器使用
extern "C" {
  void registerGridMapBuilderPlugin() {
    navsim::plugins::perception::registerGridMapBuilderPlugin();
  }
}
```

**工作流程**:
1. `dlopen()` 加载 `.so` 文件
2. `dlsym()` 查找 `registerXxxPlugin` 函数
3. 调用注册函数，将插件工厂注册到全局注册表

**优点**:
- 运行时灵活加载/卸载
- 不需要重新编译主程序
- 支持外部插件

---

### 静态注册 (Static Registration)

**代码**:
```cpp
// 静态初始化器 - 确保在程序启动时注册（用于静态链接）
namespace {
  struct GridMapBuilderPluginInitializer {
    GridMapBuilderPluginInitializer() {
      navsim::plugins::perception::registerGridMapBuilderPlugin();
    }
  };
  static GridMapBuilderPluginInitializer g_grid_map_builder_initializer;
}
```

**工作流程**:
1. 程序启动时，C++ 运行时自动调用静态初始化器
2. 静态初始化器调用注册函数
3. 插件自动注册到全局注册表

**优点**:
- 无需显式调用注册函数
- 适用于静态链接场景
- 编译时确定所有插件

---

### 双重注册的协同工作

**关键设计**:
```cpp
void registerGridMapBuilderPlugin() {
  static bool registered = false;  // ← 防止重复注册
  if (!registered) {
    plugin::PerceptionPluginRegistry::getInstance().registerPlugin(...);
    registered = true;
  }
}
```

**场景分析**:

#### 场景 1: 动态加载（navsim_local_debug）
```
1. dlopen() 加载 .so 文件
2. 静态初始化器自动执行 → 注册插件 (registered = true)
3. dlsym() 查找注册函数
4. 调用注册函数 → 检测到已注册，跳过
```

#### 场景 2: 静态链接（navsim_planning）
```
1. 程序启动
2. 静态初始化器自动执行 → 注册插件 (registered = true)
3. 无需显式调用注册函数
```

✅ **结论**: 双重注册机制是**有意设计**，不是冗余，确保两种场景都能正常工作。

---

## ⚠️ ESDFBuilder 警告问题分析

### 警告信息

```
[DynamicPluginLoader] Warning: Cannot find register function 'registerESDFBuilderPlugin': 
/home/.../libesdf_builder_plugin.so: undefined symbol: registerESDFBuilderPlugin
[DynamicPluginLoader] Plugin may use static registration
```

### 根本原因

**问题**: 注册函数名称不匹配

| 组件 | 期望的函数名 | 实际的函数名 |
|------|------------|------------|
| DynamicPluginLoader | `registerESDFBuilderPlugin` | `registerEsdfBuilderPlugin` |
| 插件名称 | `ESDFBuilder` | - |
| 注册函数 | - | `registerEsdfBuilderPlugin` |

**代码对比**:

```cpp
// DynamicPluginLoader.cpp (line 256)
std::string register_func_name = "register" + plugin_name + "Plugin";
// plugin_name = "ESDFBuilder"
// register_func_name = "registerESDFBuilderPlugin"  ← 期望

// esdf_builder/src/register.cpp (line 31)
extern "C" {
  void registerEsdfBuilderPlugin() {  // ← 实际
    navsim::plugins::perception::registerEsdfBuilderPlugin();
  }
}
```

**验证**:
```bash
$ nm -D build/plugins/perception/esdf_builder/libesdf_builder_plugin.so | grep register
registerEsdfBuilderPlugin  ← 实际导出的符号（小写 'esdf'）
```

### 为什么插件仍然能工作？

**答案**: 静态注册机制作为后备方案

```cpp
// 静态初始化器在 dlopen() 时自动执行
namespace {
  struct EsdfBuilderPluginInitializer {
    EsdfBuilderPluginInitializer() {
      navsim::plugins::perception::registerEsdfBuilderPlugin();  // ← 自动注册
    }
  };
  static EsdfBuilderPluginInitializer esdf_builder_plugin_initializer;
}
```

**工作流程**:
1. `dlopen()` 加载 `libesdf_builder_plugin.so`
2. 静态初始化器自动执行 → 插件注册成功 ✅
3. `dlsym()` 查找 `registerESDFBuilderPlugin` → 找不到 ⚠️
4. 打印警告，但继续执行（因为插件已通过静态注册）

✅ **结论**: 警告不影响功能，但应该修复以保持一致性。

---

## 🔧 解决方案

### 方案 1: 修复注册函数名称（推荐）✅

**修改**: `esdf_builder/src/register.cpp`

```cpp
extern "C" {
  void registerESDFBuilderPlugin() {  // ← 改为大写 ESDF
    navsim::plugins::perception::registerEsdfBuilderPlugin();
  }
}
```

**优点**:
- 保持命名一致性
- 消除警告信息
- 不影响现有功能

**缺点**:
- 需要修改插件代码

---

### 方案 2: 改进动态加载器的名称解析

**修改**: `DynamicPluginLoader::loadPlugin()`

```cpp
// 尝试多种命名变体
std::vector<std::string> register_func_candidates = {
  "register" + plugin_name + "Plugin",           // registerESDFBuilderPlugin
  "register" + toLowerCamelCase(plugin_name) + "Plugin",  // registerEsdfBuilderPlugin
};

for (const auto& func_name : register_func_candidates) {
  RegisterFunc register_func = (RegisterFunc)dlsym(handle, func_name.c_str());
  if (register_func) {
    register_func();
    break;
  }
}
```

**优点**:
- 更灵活，容错性更强
- 支持多种命名风格

**缺点**:
- 增加复杂度
- 可能掩盖真正的命名错误

---

### 方案 3: 移除动态注册，仅保留静态注册

**修改**: 删除 `extern "C"` 部分，仅保留静态初始化器

**优点**:
- 简化代码
- 消除命名不一致问题

**缺点**:
- 失去显式控制注册时机的能力
- 依赖 C++ 静态初始化顺序（可能有问题）

❌ **不推荐**: 违背了"统一加载方式"的设计目标

---

### 方案 4: 移除静态注册，仅保留动态注册

**修改**: 删除静态初始化器部分

**优点**:
- 更清晰的控制流
- 避免重复注册

**缺点**:
- 不支持静态链接场景
- 如果忘记调用注册函数，插件不可用

❌ **不推荐**: 失去了对 `navsim_planning` 的支持

---

## ✅ 推荐方案

### 统一命名规范 + 保留双重注册

**原则**:
1. **保留双重注册机制**（支持动态加载和静态链接）
2. **统一命名规范**（消除不一致）
3. **添加命名检查**（编译时或测试时验证）

**实施步骤**:

#### 步骤 1: 修复 ESDFBuilder 的注册函数名称

```cpp
// esdf_builder/src/register.cpp
extern "C" {
  void registerESDFBuilderPlugin() {  // ← 大写 ESDF
    navsim::plugins::perception::registerEsdfBuilderPlugin();
  }
}
```

#### 步骤 2: 添加命名规范文档

在 `templates/README.md` 中明确规定：
- 插件名称使用 PascalCase（如 `ESDFBuilder`）
- 注册函数名称为 `register{PluginName}Plugin`（如 `registerESDFBuilderPlugin`）
- 保持大小写一致

#### 步骤 3: 添加自动化测试

```cpp
// 测试注册函数是否存在
TEST(PluginTest, RegistrationFunctionExists) {
  void* handle = dlopen("libesdf_builder_plugin.so", RTLD_NOW);
  ASSERT_NE(handle, nullptr);
  
  void* func = dlsym(handle, "registerESDFBuilderPlugin");
  EXPECT_NE(func, nullptr) << "Registration function not found";
  
  dlclose(handle);
}
```

---

## 📚 最佳实践

### 1. 插件开发者

**DO ✅**:
- 使用插件模板生成代码
- 保持注册函数名称与插件名称一致
- 同时提供动态注册和静态注册

**DON'T ❌**:
- 不要手动修改注册函数名称
- 不要删除静态初始化器
- 不要假设注册顺序

---

### 2. 插件使用者

**DO ✅**:
- 优先使用短名称（如 `GridMapBuilder`）
- 检查插件是否成功加载
- 使用 `--verbose` 查看详细日志

**DON'T ❌**:
- 不要依赖警告信息判断插件是否可用
- 不要混用静态链接和动态加载同一个插件

---

## 🎯 总结

### 回答您的问题

#### 1. 统一加载方式是否已经完全实现？

✅ **是的**，已完全实现：
- 所有 5 个插件都编译为 `.so` 文件
- 动态加载机制完全工作
- 支持短名称和灵活的搜索路径

#### 2. 为什么会出现 ESDFBuilder 警告？

⚠️ **注册函数名称不匹配**：
- 期望: `registerESDFBuilderPlugin`（大写 ESDF）
- 实际: `registerEsdfBuilderPlugin`（小写 esdf）

#### 3. 静态注册是什么意思？

🔄 **静态初始化器自动注册**：
- 在 `dlopen()` 时自动执行
- 作为动态注册的后备方案
- 支持静态链接场景

#### 4. 是否与"统一加载方式"矛盾？

❌ **不矛盾**：
- 双重注册是**有意设计**
- 确保动态加载和静态链接都能工作
- 静态注册不影响动态加载

#### 5. 如何修复？

✅ **推荐方案**：
- 修复 `registerEsdfBuilderPlugin` → `registerESDFBuilderPlugin`
- 保留双重注册机制
- 添加命名规范和测试

#### 6. 修复优先级？

🟡 **中等优先级**：
- 不影响功能（静态注册作为后备）
- 但应该修复以保持一致性
- 可以在阶段 2 开发工具时一并处理


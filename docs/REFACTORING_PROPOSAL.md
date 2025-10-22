# NavSim-Local 插件平台化重构方案

**版本**: 1.0
**日期**: 2025-10-17
**状态**: 📋 方案讨论阶段

---

## 🎯 关键决策总结 (TL;DR)

| 决策项 | 选择 | 说明 |
|--------|------|------|
| **架构模式** | ✅ 三层解耦架构 | algorithm (用户) + adapter (自动生成) + platform (平台) |
| **Adapter 生成** | ✅ 完全自动生成 | 脚手架工具自动生成,用户无需修改 |
| **数据结构** | ✅ Eigen + STL | 不定义自定义结构,使用标准库 |
| **项目性质** | ✅ 完全开源 | 用户可直接访问和修改所有源码 |
| **平台引用** | ✅ 直接引用头文件 | 不使用 `find_package()`,直接 `#include` |
| **工具目录** | ✅ `tools/` | 不使用 `sdk/`,避免误解为独立 SDK 包 |

**核心价值**: 用户只需编写纯算法代码 (使用 Eigen + STL),无需了解平台细节,算法可独立测试和复用。

---

## 📋 目标

将 navsim-local 从一个仿真工具转型为**可扩展的插件开发平台**,实现:

1. ✅ **平台核心与插件分离** - 稳定的平台,灵活的插件
2. ✅ **算法与平台解耦** 🆕 - 三层架构,算法可独立测试和复用
3. ✅ **本地独立调试** 🆕 - 静态场景测试工具,无需 navsim-online
4. ✅ **简化开发流程** - 脚手架工具,5 分钟创建插件
5. ✅ **标准化分发** - 插件打包和安装机制
6. ✅ **社区生态** - 支持第三方插件共享

## 🔓 开源项目说明

**navsim-local 是完全开源的项目**,用户可以:
- ✅ 直接访问和查看所有平台源码
- ✅ 修改平台代码以满足特定需求
- ✅ 贡献代码到主仓库

**提供的开发工具**:
- ✅ 脚手架工具 (`tools/create_plugin.py`) - 快速创建插件项目
- ✅ CMake 辅助函数 (`cmake/NavSimPluginHelpers.cmake`) - 简化构建配置
- ✅ 插件模板 (`templates/`) - 标准项目结构
- ❌ **不是**独立的 SDK 包或封装库
- ❌ **不是**通过 `find_package()` 安装的外部依赖

**用户开发插件时**:
- 可以直接引用平台头文件 (`#include "plugin/framework/..."`)
- 可以查看平台实现细节
- 可以根据需要修改平台代码
- 脚手架工具帮助用户**快速上手**,但并非强制使用

## 🌟 核心创新: 三层解耦架构

**传统方案的问题**:
```cpp
// ❌ 算法与平台高度耦合
class MyPlanner : public PlannerPluginInterface {
  bool plan(const PlanningContext& context, Trajectory& trajectory) {
    // 用户必须理解平台特定的数据结构
    // 算法无法独立测试
    // 代码无法复用到其他项目
  }
};
```

**本方案的创新**:
```cpp
// ✅ 用户只需编写纯算法 (algorithm/)
class MyPlanner {
  PlannerOutput plan(const PlannerInput& input) {
    // 使用标准数据结构 (Eigen, std::vector)
    // 完全独立于平台
    // 可独立测试和复用
  }
};

// ✅ Adapter 自动生成 (adapter/) - 用户无需关心
class MyPlannerAdapter : public PlannerPluginInterface {
  bool plan(const PlanningContext& context, Trajectory& trajectory) {
    auto input = convertInput(context);      // 平台 → 算法
    auto output = algorithm_.plan(input);    // 调用纯算法
    convertOutput(output, trajectory);       // 算法 → 平台
  }
private:
  MyPlanner algorithm_;  // 用户算法实例
};
```

**优势**:
- 🎯 **零学习成本** - 用户只需理解标准数据结构,无需学习平台 API
- 🧪 **易于测试** - 算法可独立测试,无需启动平台
- ♻️ **高度复用** - 算法代码可用于其他项目或平台
- 🔧 **易于维护** - 算法逻辑与平台逻辑清晰分离

---

## 🔍 现状分析

### 已有基础 ✅

经过代码审查,发现项目**已经具备良好的插件系统基础**:

- ✅ 清晰的插件接口 (`PerceptionPluginInterface`, `PlannerPluginInterface`)
- ✅ 插件注册机制 (工厂模式 + 单例注册表)
- ✅ 动态插件加载 (`DynamicPluginLoader`)
- ✅ 配置驱动的插件管理 (JSON 配置)
- ✅ ImGui 本地可视化
- ✅ 外部插件支持框架 (`external_plugins/`)

### 存在的问题 ❌

1. **必须依赖 navsim-online** - 无法独立调试插件
2. **缺乏开发工具** - 没有脚手架,手动创建项目繁琐
3. **没有标准化打包** - 插件分发没有统一流程
4. **文档分散** - 缺少系统的开发者指南

---

## 🏗️ 重构方案

### 方案 0: 保留在线模式 (现有功能)

**目标**: 保持与 navsim-online 的联合仿真能力,**不影响现有功能**

#### 0.1 在线模式说明

**在线模式**是当前的主要运行方式,通过 WebSocket 与 navsim-online 进行联合仿真:

```
navsim-online (前端 + 服务器)  ←→  navsim-local (算法)
     ↓                                    ↓
  可视化界面                          规划 + 感知
  场景编辑                            插件系统
  仿真控制                            算法执行
```

#### 0.2 在线模式配置

**配置文件**: `config/default.json`

用于配置插件、算法参数、可视化等。

**统一的配置格式** ✅:

```json
{
  "perception": {
    "plugins": [
      {
        "plugin": "GridMapBuilder",  // 短名称，自动查找
        "params": {
          "map_width": 30.0,
          "map_height": 30.0,
          "resolution": 0.1
        }
      },
      {
        "plugin": "/home/user/MyPerception/build/libmy_perception.so",  // 完整路径
        "params": {
          "custom_param": 1.0
        }
      }
    ]
  },
  "planning": {
    "primary_planner": {
      "plugin": "JpsPlanner",  // 短名称，自动查找
      "params": {
        "verbose": true,
        "safe_dis": 0.3,
        "max_vel": 1.5
      }
    },
    "fallback_planner": {
      "plugin": "/home/user/MyFallbackPlanner/build/libmy_fallback.so",  // 完整路径
      "params": {}
    },
    "enable_fallback": true
  },
  "algorithm": {
    "max_computation_time_ms": 25.0,
    "verbose_logging": true,
    "enable_visualization": true
  }
}
```

**配置说明**:

1. **`plugin` 字段** - 统一的插件指定方式:
   - **短名称**: `"JpsPlanner"` - 系统自动在默认目录中查找
   - **完整路径**: `"/path/to/plugin.so"` - 直接加载指定的 `.so` 文件

2. **插件查找规则**:
   - 如果 `plugin` 值包含 `/` 或以 `.so` 结尾 → 视为完整路径
   - 否则视为短名称，按以下顺序查找:
     1. `plugins/planning/lib{name}.so`
     2. `plugins/perception/lib{name}.so`
     3. `~/.navsim/plugins/lib{name}.so`
     4. `./external_plugins/{name}/build/lib{name}.so`
     5. `$NAVSIM_PLUGIN_PATH` 环境变量指定的目录

3. **`perception.plugins`** - 感知插件列表:
   - 支持多个插件同时运行
   - 可以混合使用平台插件和用户插件

4. **`planning.primary_planner`** - 主规划器:
   - 使用短名称或完整路径
   - 示例: `{"plugin": "JpsPlanner", "params": {...}}`

5. **`planning.fallback_planner`** - 降级规划器:
   - 当主规划器失败时使用
   - `enable_fallback` 控制是否启用降级

6. **`algorithm`** - 算法管理器配置:
   - `max_computation_time_ms`: 最大计算时间
   - `verbose_logging`: 是否输出详细日志
   - `enable_visualization`: 是否启用 ImGui 本地可视化

**核心优势**:
- ✅ **统一加载方式**: 所有插件都是 `.so` 文件，无需区分类型
- ✅ **简洁配置**: 只需一个 `plugin` 字段
- ✅ **灵活性**: 支持短名称和完整路径
- ✅ **自动查找**: 平台插件无需写完整路径

#### 0.2.1 插件查找机制 🔍

**核心思想**: 统一加载方式,所有插件都编译为 `.so` 文件

**查找规则**:

1. **判断是否为完整路径**:
   - 如果 `plugin` 值包含 `/` → 视为完整路径
   - 如果 `plugin` 值以 `.so` 结尾 → 视为完整路径
   - 否则 → 视为短名称,进入自动查找流程

2. **自动查找顺序** (短名称):
   ```
   1. plugins/planning/lib{name}.so       # 平台规划器插件
   2. plugins/perception/lib{name}.so     # 平台感知插件
   3. ~/.navsim/plugins/lib{name}.so      # 用户全局插件
   4. external_plugins/{name}/build/lib{name}.so  # 外部插件
   5. $NAVSIM_PLUGIN_PATH/lib{name}.so    # 环境变量指定的目录
   ```

3. **查找示例**:
   ```cpp
   // 短名称: "JpsPlanner"
   // 查找路径:
   // 1. plugins/planning/libJpsPlanner.so  ✅ 找到!
   // 2. plugins/perception/libJpsPlanner.so
   // ...

   // 短名称: "MyPlanner"
   // 查找路径:
   // 1. plugins/planning/libMyPlanner.so
   // 2. plugins/perception/libMyPlanner.so
   // 3. ~/.navsim/plugins/libMyPlanner.so
   // 4. external_plugins/MyPlanner/build/libMyPlanner.so  ✅ 找到!

   // 完整路径: "/home/user/MyPlanner/build/libmy_planner.so"
   // 直接加载,不进行查找
   ```

**环境变量配置**:
```bash
# 添加自定义插件目录
export NAVSIM_PLUGIN_PATH=/home/user/my_plugins

# 现在可以使用短名称引用该目录下的插件
# 配置: {"plugin": "CustomPlanner"}
# 查找: $NAVSIM_PLUGIN_PATH/libCustomPlanner.so
```

**优势**:
- ✅ **简洁**: 平台插件只需写短名称 (如 `"JpsPlanner"`)
- ✅ **灵活**: 用户插件可以用短名称或完整路径
- ✅ **可扩展**: 支持环境变量自定义查找路径
- ✅ **统一**: 所有插件都是 `.so` 文件,无需区分类型

#### 0.3 在线模式使用方式

**启动 navsim-online**:
```bash
cd navsim-online
./run_navsim.sh

# 服务器启动在:
# - WebSocket: ws://127.0.0.1:8080/ws
# - 前端: http://127.0.0.1:8000/index.html
```

**启动 navsim-local**:
```bash
cd navsim-local

# 方式 1: 使用脚本 (推荐)
./build_with_visualization.sh

# 方式 2: 手动运行
./build/navsim_algo ws://127.0.0.1:8080/ws demo --config=config/default.json

# 参数说明:
# - ws://127.0.0.1:8080/ws  : WebSocket 服务器地址
# - demo                    : Room ID (房间标识)
# - --config=...            : 配置文件路径
```

**工作流程**:
1. navsim-online 提供场景、障碍物、自车状态
2. navsim-local 接收 `world_tick` 消息
3. 感知插件处理障碍物数据
4. 规划器插件生成轨迹
5. navsim-local 发送 `plan_update` 消息
6. navsim-online 可视化轨迹并更新仿真

#### 0.4 在线模式 vs 本地调试模式

| 维度 | 在线模式 | 本地调试模式 (新增) |
|------|---------|-------------------|
| **程序** | `navsim_algo` | `navsim_local_debug` |
| **依赖** | 需要 navsim-online | 完全独立 |
| **场景来源** | navsim-online 实时提供 | 静态 JSON 文件 |
| **可视化** | navsim-online 前端 + ImGui | 只有 ImGui |
| **仿真控制** | navsim-online 控制 | 无仿真循环 |
| **插件加载** | 统一的 `.so` 加载方式 ✅ | 统一的 `.so` 加载方式 ✅ |
| **配置文件** | `config/default.json` | `config/plugins.json` |
| **用途** | 完整的联合仿真 | 快速测试插件 |

**重要**:
- 重构方案**不会影响在线模式**,两种模式可以共存
- **统一的插件加载方式** - 所有插件都是 `.so` 文件 ✅
- **支持短名称和完整路径** - 灵活配置 ✅
- 在线模式: 使用 `navsim_algo` 程序
- 本地调试模式: 使用 `navsim_local_debug` 程序 (新增)

---

### 方案 1: 本地调试模式 (核心)

**目标**: 支持无需 navsim-online 的独立调试,提供**静态场景测试工具**

#### 1.1 设计理念

**简化设计** - 不需要复杂的动态仿真:
- ❌ **不需要**模拟 navsim-online 的动态仿真行为
- ❌ **不需要**时间步更新、播放/暂停/单步控制
- ❌ **不需要**仿真循环
- ✅ **只需要**加载静态场景 → 调用规划器 → 可视化结果

**核心价值**: 快速验证插件在特定场景下的规划结果

#### 1.2 新增组件

```
navsim-local/
├── src/core/
│   └── scenario_loader.cpp        # 🆕 场景加载器
├── scenarios/                      # 🆕 测试场景库
│   ├── simple_corridor.json
│   ├── urban_intersection.json
│   ├── parking.json
│   └── ...
└── apps/
    └── navsim_local_debug.cpp     # 🆕 本地调试主程序
```

#### 1.3 场景定义格式

**重要**: JSON 格式与 **navsim-online 的地图保存功能保存的格式一致**,方便从在线系统导出场景用于离线测试。

```json
{
  "scenario": {
    "name": "Simple Corridor",
    "description": "直线走廊避障测试"
  },
  "ego": {
    "pose": {"x": 5.0, "y": 25.0, "theta": 0.0},
    "goal": {"x": 45.0, "y": 25.0, "theta": 0.0},
    "velocity": {"vx": 0.0, "vy": 0.0, "omega": 0.0}
  },
  "obstacles": {
    "static_circles": [
      {"center": {"x": 15.0, "y": 25.0}, "radius": 2.0},
      {"center": {"x": 35.0, "y": 25.0}, "radius": 1.5}
    ],
    "static_polygons": [
      {
        "vertices": [
          {"x": 20.0, "y": 20.0},
          {"x": 22.0, "y": 20.0},
          {"x": 22.0, "y": 30.0},
          {"x": 20.0, "y": 30.0}
        ]
      }
    ]
  },
  "map": {
    "width": 50.0,
    "height": 50.0,
    "resolution": 0.1
  }
}
```

**注意**:
- 格式与 navsim-online 保存的场景格式完全一致
- 可以直接使用在线系统导出的场景文件
- 只包含静态信息,不包含动态轨迹

#### 1.4 插件配置

**配置文件**: `config/plugins.json`

用于指定使用哪些插件以及插件的参数配置。

**配置格式**:

```json
{
  "plugins": {
    "planner": {
      "plugin": "AStarPlanner",  // 短名称，自动查找
      "params": {
        "step_size": 0.1,
        "max_iterations": 1000,
        "goal_tolerance": 0.5
      }
    },
    "perception": {
      "plugin": "/home/user/MyPerception/build/libmy_perception.so",  // 完整路径
      "params": {
        "detection_range": 50.0
      }
    }
  }
}
```

**配置说明**:

1. **`plugin`** - 统一的插件指定方式:
   - **短名称**: `"AStarPlanner"` - 系统自动查找
   - **完整路径**: `"/path/to/plugin.so"` - 直接加载

2. **插件查找规则**:
   - 如果 `plugin` 值包含 `/` 或以 `.so` 结尾 → 视为完整路径
   - 否则视为短名称，按以下顺序查找:
     1. `plugins/planning/lib{name}.so`
     2. `plugins/perception/lib{name}.so`
     3. `~/.navsim/plugins/lib{name}.so`
     4. `./external_plugins/{name}/build/lib{name}.so`
     5. `$NAVSIM_PLUGIN_PATH` 环境变量指定的目录

3. **`params`** - 插件参数:
   - 传递给插件的配置参数
   - 插件在 `initialize()` 方法中接收这些参数
   - 不同插件有不同的参数

**配置示例**:

**示例 1: 使用平台插件 (短名称)**
```json
{
  "plugins": {
    "planner": {
      "plugin": "AStarPlanner",  // 自动查找 plugins/planning/libAStarPlanner.so
      "params": {
        "step_size": 0.1
      }
    }
  }
}
```

**示例 2: 使用用户自定义插件 (完整路径)**
```json
{
  "plugins": {
    "planner": {
      "plugin": "/home/user/my_projects/MyPlanner/build/libmy_planner.so",
      "params": {
        "algorithm": "rrt_star",
        "max_iterations": 5000
      }
    }
  }
}
```

**示例 3: 混合使用**
```json
{
  "plugins": {
    "planner": {
      "plugin": "/home/user/MyPlanner/build/libmy_planner.so",  // 用户插件
      "params": {}
    },
    "perception": {
      "plugin": "GridMapBuilder",  // 平台插件
      "params": {}
    }
  }
}
```

#### 1.5 使用方式

**简单流程**: 加载场景 → 规划一次 → 查看结果

```bash
# 基本用法: 使用配置文件指定插件
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --config config/plugins.json

# 或者通过命令行参数直接指定插件 (覆盖配置文件)
./build/navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --plugin /home/user/MyPlanner/build/libmy_planner.so

# 从 navsim-online 导出的场景
./build/navsim_local_debug \
  --scenario exported_from_online.json \
  --config config/plugins.json

# 批量测试多个场景
for scene in scenarios/*.json; do
  ./build/navsim_local_debug \
    --scenario $scene \
    --config config/plugins.json \
    --no-gui
done
```

**命令行参数优先级**:
- `--plugin` 参数会覆盖配置文件中的插件设置
- 适合快速测试不同的插件,无需修改配置文件

**工作流程**:
1. 加载插件配置 (`config/plugins.json`)
2. 加载场景 JSON 文件
3. 构造 `PlanningContext` (包含起点、终点、障碍物)
4. 调用规划器插件的 `plan()` 方法
5. 得到 `Trajectory` 结果
6. 使用 ImGui 可视化场景和规划结果
7. 用户可以查看路径、调整参数、重新规划

#### 1.6 核心实现

**ScenarioLoader** - 加载静态场景:
```cpp
class ScenarioLoader {
public:
  // 从 JSON 加载场景 (格式与 navsim-online 一致)
  bool loadFromJson(const std::string& json_file);

  // 获取场景数据
  const ScenarioData& getScenario() const;

  // 转换为 PlanningContext
  planning::PlanningContext toPlanningContext() const;
};
```

**AlgorithmManager** - 插件管理:
```cpp
class AlgorithmManager {
public:
  // 从配置文件加载插件
  bool loadPlugins(const std::string& config_file);

  // 或者直接加载指定的插件
  bool loadPlugin(const std::string& plugin_type,
                  const std::string& plugin_spec);

  // 调用规划器
  bool plan(const planning::PlanningContext& context,
            planning::Trajectory& trajectory);

private:
  // 插件查找路径
  std::vector<std::string> plugin_search_paths_;

  // 已加载的插件
  std::map<std::string, std::shared_ptr<PluginInterface>> loaded_plugins_;

  // 解析插件规格 (短名称或完整路径)
  std::string resolvePluginPath(const std::string& plugin_spec);
};
```

**插件配置加载逻辑**:
```cpp
bool AlgorithmManager::loadPlugins(const std::string& config_file) {
  // 1. 读取 JSON 配置
  nlohmann::json config = readJsonFile(config_file);

  // 2. 加载规划器插件
  auto planner_config = config["plugins"]["planner"];
  std::string plugin_spec = planner_config["plugin"];  // 短名称或完整路径

  // 3. 解析插件路径
  std::string plugin_path = resolvePluginPath(plugin_spec);

  // 4. 动态加载插件
  planner_ = loadDynamicPlugin(plugin_path);

  // 5. 初始化插件 (传递参数)
  auto params = planner_config["params"];
  planner_->initialize(params);

  return true;
}

// 插件路径解析逻辑
std::string AlgorithmManager::resolvePluginPath(const std::string& plugin_spec) {
  // 1. 如果包含 '/' 或以 '.so' 结尾,视为完整路径
  if (plugin_spec.find('/') != std::string::npos ||
      plugin_spec.ends_with(".so")) {
    return plugin_spec;
  }

  // 2. 否则视为短名称,在默认目录中查找
  std::vector<std::string> search_paths = {
    "plugins/planning/lib" + plugin_spec + ".so",
    "plugins/perception/lib" + plugin_spec + ".so",
    std::string(getenv("HOME")) + "/.navsim/plugins/lib" + plugin_spec + ".so",
    "external_plugins/" + plugin_spec + "/build/lib" + plugin_spec + ".so"
  };

  // 3. 添加环境变量指定的路径
  if (const char* env_path = getenv("NAVSIM_PLUGIN_PATH")) {
    search_paths.push_back(std::string(env_path) + "/lib" + plugin_spec + ".so");
  }

  // 4. 按顺序查找
  for (const auto& path : search_paths) {
    if (std::filesystem::exists(path)) {
      return path;
    }
  }

  // 5. 未找到,抛出异常
  throw std::runtime_error("Plugin not found: " + plugin_spec);
}
```

**navsim_local_debug.cpp** - 主程序:
```cpp
int main(int argc, char** argv) {
  // 解析命令行参数
  std::string scenario_file = argv[1];  // --scenario
  std::string config_file = argv[2];    // --config
  std::string plugin_override = "";     // --plugin (可选)

  // 1. 加载插件
  AlgorithmManager algo_mgr;
  if (!plugin_override.empty()) {
    // 命令行指定插件,覆盖配置文件
    algo_mgr.loadPlugin("planner", plugin_override);
  } else {
    // 使用配置文件
    algo_mgr.loadPlugins(config_file);
  }

  // 2. 加载场景
  ScenarioLoader loader;
  loader.loadFromJson(scenario_file);

  // 3. 构造规划上下文
  auto context = loader.toPlanningContext();

  // 4. 调用规划器
  planning::Trajectory trajectory;
  algo_mgr.plan(context, trajectory);

  // 5. 可视化结果
  Visualizer viz;
  viz.drawScenario(loader.getScenario());
  viz.drawTrajectory(trajectory);
  viz.show();  // 显示窗口,用户可以查看结果

  return 0;
}
```

**优势**:
- ✅ **极简设计** - 只做一件事:测试插件在静态场景下的规划能力
- ✅ **完全独立** - 无需 navsim-online,无需网络连接
- ✅ **格式兼容** - 可直接使用在线系统导出的场景
- ✅ **快速验证** - 秒级加载和规划,快速迭代
- ✅ **易于调试** - 静态场景,结果可重复
- ✅ **CI 友好** - 可用于自动化测试 (--no-gui 模式)

---

### 方案 2: 插件开发工具链 (三层解耦架构)

**目标**: 简化插件创建流程,5 分钟创建插件项目,**算法与平台完全解耦**

#### 2.1 三层解耦架构 🆕

**核心思想**: 用户只需编写纯算法代码,无需了解平台细节

```
┌─────────────────────────────────────────────────────────┐
│  用户算法层 (algorithm/)                                 │
│  - 纯算法逻辑,使用标准数据结构                           │
│  - 完全独立于平台,可复用到其他项目                       │
│  - 可独立进行单元测试                                    │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│  Adapter 层 (adapter/)  🤖 自动生成                      │
│  - 平台数据结构 ↔ 算法数据结构转换                       │
│  - 实现平台接口,调用用户算法                             │
│  - 由脚手架工具自动生成,用户无需修改                     │
└────────────────────┬────────────────────────────────────┘
                     │
┌────────────────────┴────────────────────────────────────┐
│  平台接口层 (platform/interface/)                        │
│  - PlannerPluginInterface                               │
│  - PlanningContext, Trajectory 等平台数据结构            │
│  - 由平台维护,保持稳定                                   │
└─────────────────────────────────────────────────────────┘
```

#### 2.2 脚手架工具

```bash
# 创建插件项目
python3 tools/create_plugin.py MyPlanner planner

# 自动生成三层结构:
MyPlanner/
├── CMakeLists.txt                    # 配置好的构建文件
├── algorithm/                        # 🎯 用户算法层 (用户编辑)
│   ├── include/
│   │   └── my_planner.hpp           # 纯算法接口
│   └── src/
│       └── my_planner.cpp           # 纯算法实现
├── adapter/                          # 🤖 Adapter 层 (自动生成)
│   ├── include/
│   │   └── my_planner_adapter.hpp   # 平台适配器
│   └── src/
│       └── my_planner_adapter.cpp   # 数据转换逻辑
├── config/
│   └── default.json                 # 默认配置
├── tests/                            # 🧪 单元测试
│   └── test_my_planner.cpp          # 纯算法测试
├── README.md                         # 使用说明
└── .gitignore
```

#### 2.3 用户算法层 (用户编写)

**用户只需关注纯算法逻辑**,使用标准数据结构:

```cpp
// algorithm/include/my_planner.hpp
#pragma once
#include <vector>
#include <Eigen/Dense>

namespace my_planner {

// 纯算法的输入数据 (标准数据结构)
struct PlannerInput {
  Eigen::Vector3d start;              // 起点 (x, y, theta)
  Eigen::Vector3d goal;               // 终点 (x, y, theta)
  std::vector<Eigen::Vector2d> obstacles;  // 障碍物中心点
  std::vector<double> obstacle_radii;      // 障碍物半径
  double map_resolution;              // 地图分辨率
};

// 纯算法的输出数据 (标准数据结构)
struct PlannerOutput {
  std::vector<Eigen::Vector3d> path;  // 路径点 (x, y, theta)
  bool success;                       // 是否成功
  std::string message;                // 状态信息
};

// 纯算法类 - 完全独立于平台
class MyPlanner {
public:
  MyPlanner() = default;
  ~MyPlanner() = default;

  // 配置算法参数 (使用简单的 map)
  void configure(const std::map<std::string, double>& params);

  // 纯算法规划接口
  PlannerOutput plan(const PlannerInput& input);

private:
  // 算法参数
  double step_size_ = 0.1;
  double max_iterations_ = 1000;

  // 算法内部方法
  bool isCollisionFree(const Eigen::Vector2d& point);
  // ... 其他算法逻辑
};

} // namespace my_planner
```

**优势**:
- ✅ **零平台依赖** - 只依赖标准库和 Eigen
- ✅ **易于测试** - 可以直接编写单元测试
- ✅ **易于理解** - 清晰的输入输出,无需学习平台 API
- ✅ **可复用** - 算法可以用于其他项目

#### 2.4 Adapter 层 (自动生成) 🤖

**由脚手架工具自动生成**,用户无需修改:

```cpp
// adapter/include/my_planner_adapter.hpp
#pragma once
#include "plugin/framework/planner_plugin_interface.hpp"
#include "algorithm/include/my_planner.hpp"

namespace my_planner {

// Adapter 类 - 连接平台与算法
class MyPlannerAdapter : public navsim::plugin::PlannerPluginInterface {
public:
  MyPlannerAdapter() = default;
  ~MyPlannerAdapter() override = default;

  // 平台接口实现
  PluginMetadata getMetadata() const override;
  bool initialize(const nlohmann::json& config) override;
  bool plan(const planning::PlanningContext& context,
            planning::Trajectory& trajectory) override;
  bool isAvailable() const override { return true; }

private:
  MyPlanner algorithm_;  // 用户算法实例

  // 数据转换方法 (自动生成)
  PlannerInput convertInput(const planning::PlanningContext& context);
  void convertOutput(const PlannerOutput& output,
                     planning::Trajectory& trajectory);
};

} // namespace my_planner

// 注册插件
REGISTER_PLANNER_PLUGIN(my_planner::MyPlannerAdapter);
```

**Adapter 实现示例**:

```cpp
// adapter/src/my_planner_adapter.cpp
#include "adapter/include/my_planner_adapter.hpp"

namespace my_planner {

bool MyPlannerAdapter::initialize(const nlohmann::json& config) {
  // 将 JSON 配置转换为算法参数
  std::map<std::string, double> params;
  if (config.contains("step_size")) {
    params["step_size"] = config["step_size"];
  }
  if (config.contains("max_iterations")) {
    params["max_iterations"] = config["max_iterations"];
  }

  algorithm_.configure(params);
  return true;
}

bool MyPlannerAdapter::plan(const planning::PlanningContext& context,
                             planning::Trajectory& trajectory) {
  // 1. 平台数据 → 算法数据
  PlannerInput input = convertInput(context);

  // 2. 调用纯算法
  PlannerOutput output = algorithm_.plan(input);

  // 3. 算法数据 → 平台数据
  if (output.success) {
    convertOutput(output, trajectory);
    return true;
  }
  return false;
}

PlannerInput MyPlannerAdapter::convertInput(
    const planning::PlanningContext& context) {
  PlannerInput input;

  // 转换起点和终点
  input.start = Eigen::Vector3d(
    context.ego_state.pose.x,
    context.ego_state.pose.y,
    context.ego_state.pose.theta
  );
  input.goal = Eigen::Vector3d(
    context.goal.pose.x,
    context.goal.pose.y,
    context.goal.pose.theta
  );

  // 转换障碍物 (从栅格地图提取)
  if (context.occupancy_grid) {
    // ... 从栅格地图提取障碍物
  }

  input.map_resolution = context.map_resolution;
  return input;
}

void MyPlannerAdapter::convertOutput(const PlannerOutput& output,
                                      planning::Trajectory& trajectory) {
  trajectory.points.clear();
  for (const auto& point : output.path) {
    planning::TrajectoryPoint tp;
    tp.pose.x = point.x();
    tp.pose.y = point.y();
    tp.pose.theta = point.z();
    trajectory.points.push_back(tp);
  }
}

} // namespace my_planner
```

#### 2.5 CMake 构建配置

**插件项目的 `CMakeLists.txt`** (由脚手架自动生成):

```cmake
cmake_minimum_required(VERSION 3.16)
project(MyPlanner)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖 (标准库)
find_package(Eigen3 REQUIRED)

# 设置 NavSim 平台路径 (假设插件在 external_plugins/ 下)
set(NAVSIM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(NAVSIM_INCLUDE_DIR "${NAVSIM_ROOT}/include")

# 引入 NavSim CMake 辅助函数 (可选)
include("${NAVSIM_ROOT}/cmake/NavSimPluginHelpers.cmake")

# 1. 用户算法库 (独立编译,可单独测试)
add_library(my_planner_algorithm
  algorithm/src/my_planner.cpp
)
target_include_directories(my_planner_algorithm PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}  # 插件自己的头文件
)
target_link_libraries(my_planner_algorithm PUBLIC
  Eigen3::Eigen  # 只依赖标准库,无平台依赖
)

# 2. Adapter 插件 (连接平台与算法)
add_library(my_planner_plugin SHARED
  adapter/src/my_planner_adapter.cpp
)
target_include_directories(my_planner_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}      # 插件头文件
  ${NAVSIM_INCLUDE_DIR}            # 平台头文件 (直接引用)
)
target_link_libraries(my_planner_plugin PRIVATE
  my_planner_algorithm  # 链接用户算法库
  Eigen3::Eigen
  # 注意: 不需要链接平台库,插件是动态加载的
)

# 或者使用辅助函数 (简化版本)
# navsim_add_planner_plugin(
#     NAME my_planner_plugin
#     ALGORITHM_SOURCES algorithm/src/my_planner.cpp
#     ADAPTER_SOURCES adapter/src/my_planner_adapter.cpp
# )

# 3. 安装插件到标准位置
install(TARGETS my_planner_plugin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/navsim/plugins
)

# 4. 单元测试 (可选)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  add_executable(test_my_planner tests/test_my_planner.cpp)
  target_link_libraries(test_my_planner
    my_planner_algorithm  # 只测试算法,无需平台
    GTest::gtest_main
  )
  add_test(NAME test_my_planner COMMAND test_my_planner)
endif()
```

**关键点**:
- ✅ **直接引用平台头文件** - 通过 `${NAVSIM_INCLUDE_DIR}`,不使用 `find_package()`
- ✅ **算法库零平台依赖** - `my_planner_algorithm` 只依赖 Eigen
- ✅ **Adapter 动态加载** - 插件是 `.so` 文件,运行时动态加载
- ✅ **可选的辅助函数** - `navsim_add_planner_plugin()` 进一步简化配置

#### 2.6 用户工作流

**完整的开发流程**:

```bash
# 0. 克隆 navsim-local 仓库 (如果还没有)
git clone https://github.com/your-org/navsim-local.git
cd navsim-local

# 1. 创建插件项目 (在 external_plugins/ 下)
python3 tools/create_plugin.py MyPlanner planner

# 脚手架自动生成:
# external_plugins/MyPlanner/
# ├── algorithm/          # 用户算法层
# ├── adapter/            # Adapter 层 (自动生成)
# ├── tests/              # 单元测试
# └── CMakeLists.txt      # 构建配置

# 2. 实现算法逻辑 (只编辑 algorithm/ 目录)
cd external_plugins/MyPlanner/algorithm/src
vim my_planner.cpp  # 实现纯算法

# 用户可以随时查看平台代码:
# - 查看接口定义: ../../include/plugin/framework/planner_plugin_interface.hpp
# - 查看数据结构: ../../include/core/planning_context.hpp
# - 查看其他插件: ../../plugins/planning/

# 3. 编译和测试
cd external_plugins/MyPlanner
mkdir build && cd build
cmake .. && make

# 3a. 本地调试 (推荐) - 使用静态场景测试完整功能
cd ../../../build  # 回到 navsim-local/build
./navsim_local_debug \
  --scenario ../scenarios/simple_corridor.json \
  --plugin ../external_plugins/MyPlanner/build/libmy_planner_plugin.so

# 3b. 单元测试 (可选) - 测试纯算法逻辑
cd ../external_plugins/MyPlanner/build
./test_my_planner

# 4. 如果需要修改平台代码 (完全允许!)
cd ../../src/core
vim algorithm_manager.cpp  # 修改平台逻辑
cd ../../build && make     # 重新编译平台
```

**关键点**:
- ✅ **用户可以查看平台代码** - 所有源码都在仓库中
- ✅ **用户可以修改平台代码** - 开源项目,完全自由
- ✅ **脚手架降低门槛** - 但不强制使用,用户可以手动创建
- ✅ **灵活性** - 可以选择只开发算法,也可以深度定制平台

#### 2.7 插件测试方式

**推荐: 使用本地调试模式测试** ✅

本地调试模式是**最实用的测试方式**:

```bash
# 加载静态场景,测试插件
./navsim_local_debug \
  --scenario scenarios/simple_corridor.json \
  --plugin external_plugins/MyPlanner/build/libmy_planner_plugin.so

# 可视化查看规划结果,快速验证算法效果
```

**优势**:
- ✅ **直观** - 可视化查看规划结果,一目了然
- ✅ **真实** - 使用完整的平台数据结构,测试真实场景
- ✅ **快速** - 秒级加载和规划,快速迭代
- ✅ **易用** - 无需编写测试代码,只需准备场景 JSON

---

**可选: 纯算法单元测试** (高级用户)

如果需要更细粒度的测试,可以编写纯算法单元测试:

```cpp
// tests/test_my_planner.cpp (可选)
#include <gtest/gtest.h>
#include "algorithm/include/my_planner.hpp"

TEST(MyPlannerTest, SimpleStraightLine) {
  MyPlanner planner;

  PlannerInput input;
  input.start = Eigen::Vector3d(0, 0, 0);
  input.goal = Eigen::Vector3d(10, 0, 0);

  PlannerOutput output = planner.plan(input);

  EXPECT_TRUE(output.success);
  EXPECT_GT(output.path.size(), 0);
}
```

**注意**:
- ⚠️ **开发成本较高** - 需要手动构造输入数据,编写测试用例
- ⚠️ **维护成本高** - 数据结构变化时需要更新测试代码
- ✅ **适合特定场景** - 如 CI/CD 自动化测试、算法边界条件测试

**建议**:
- 大多数用户使用**本地调试模式**即可满足需求
- 只有在需要自动化测试或精细测试时才编写单元测试

#### 2.8 脚手架工具实现要点

`tools/create_plugin.py` 需要实现:

1. **读取模板文件** - 从 `templates/planner_plugin/` 或 `templates/perception_plugin/` 读取模板
2. **生成三层目录结构** - `algorithm/`, `adapter/`, `tests/`
3. **替换模板变量** - 将 `{{plugin_name}}`, `{{author}}` 等替换为实际值
4. **生成算法模板** - 使用 Eigen + STL 的纯算法类
5. **自动生成 Adapter 代码** - 包含完整的数据转换逻辑
6. **生成 CMakeLists.txt** - 直接引用平台头文件,不使用 `find_package()`
7. **生成单元测试模板** - 如上所示的测试示例
8. **生成 README** - 说明如何编译、测试、使用

**工作流程**:

```
用户运行:
  python3 tools/create_plugin.py MyPlanner planner

脚手架工具:
  1. 读取 templates/planner_plugin/ 目录
  2. 复制所有文件到 external_plugins/MyPlanner/
  3. 替换模板变量:
     - {{plugin_name}} → MyPlanner
     - {{plugin_type}} → planner
     - {{author}} → 用户名
  4. 生成完整的插件项目

结果:
  external_plugins/MyPlanner/
  ├── algorithm/
  │   ├── include/my_planner.hpp
  │   └── src/my_planner.cpp
  ├── adapter/
  │   ├── include/my_planner_adapter.hpp
  │   └── src/my_planner_adapter.cpp
  ├── tests/test_my_planner.cpp
  ├── CMakeLists.txt
  ├── plugin.json
  └── README.md
```

**工具使用示例**:

```bash
# 在 navsim-local 根目录下运行
cd /path/to/navsim-local

# 创建规划器插件 (默认在 external_plugins/ 下)
python3 tools/create_plugin.py MyPlanner planner \
  --author "Your Name" \
  --description "My custom planner"

# 创建感知插件
python3 tools/create_plugin.py MyPerception perception \
  --author "Your Name" \
  --description "My custom perception"

# 指定输出目录
python3 tools/create_plugin.py MyPlanner planner \
  --output /path/to/my/plugins
```

**关键点**:
- ✅ 脚手架工具从 `templates/` 目录读取模板文件
- ✅ 用户无需手动创建目录结构和文件
- ✅ 模板变量自动替换,生成可直接编译的代码
- ✅ 默认输出到 `external_plugins/`,也可指定其他目录

**`NavSimPluginHelpers.cmake` 辅助函数** (可选):

```cmake
# cmake/NavSimPluginHelpers.cmake

# 简化插件创建的辅助函数
function(navsim_add_planner_plugin)
  set(options "")
  set(oneValueArgs NAME)
  set(multiValueArgs ALGORITHM_SOURCES ADAPTER_SOURCES)
  cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  # 创建算法库
  add_library(${ARG_NAME}_algorithm ${ARG_ALGORITHM_SOURCES})
  target_include_directories(${ARG_NAME}_algorithm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(${ARG_NAME}_algorithm PUBLIC Eigen3::Eigen)

  # 创建插件
  add_library(${ARG_NAME} SHARED ${ARG_ADAPTER_SOURCES})
  target_include_directories(${ARG_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${NAVSIM_INCLUDE_DIR}
  )
  target_link_libraries(${ARG_NAME} PRIVATE ${ARG_NAME}_algorithm Eigen3::Eigen)
endfunction()
```

**优势总结**:
- ✅ **低耦合** - 算法与平台完全解耦
- ✅ **易测试** - 算法可独立测试,无需平台
- ✅ **易理解** - 用户只需理解标准数据结构 (Eigen + STL)
- ✅ **可复用** - 算法可用于其他项目
- ✅ **5 分钟创建** - 脚手架自动生成所有模板
- ✅ **标准化** - 统一的项目结构
- ✅ **高质量** - 自带单元测试框架
- ✅ **透明** - 用户可以查看和修改所有代码,包括平台代码

---

### 方案 3: 插件打包与分发

**目标**: 标准化的插件打包和安装流程

#### 3.1 打包工具

```bash
# 打包插件
python3 sdk/tools/package_plugin.py MyPlanner -o dist

# 生成:
# - MyPlanner-1.0.0.tar.gz (插件包)
# - MyPlanner-1.0.0.json (元信息)
```

#### 3.2 插件元信息

```json
{
  "name": "MyPlanner",
  "version": "1.0.0",
  "type": "planner",
  "author": "Your Name",
  "description": "My custom planner plugin",
  "navsim_version": ">=2.0.0",
  "dependencies": [],
  "files": {
    "library": "lib/libmy_planner_plugin.so",
    "config": "config/default.json"
  }
}
```

#### 3.3 安装插件

```bash
# 安装插件
python3 sdk/tools/install_plugin.py MyPlanner-1.0.0.tar.gz

# 自动:
# 1. 解压到 ~/.navsim/plugins/MyPlanner/
# 2. 验证兼容性
# 3. 注册到插件列表
```

**优势**:
- ✅ 标准化的分发格式
- ✅ 版本管理和依赖检查
- ✅ 便于社区共享

---

## 📂 目录结构调整

### 当前结构

```
navsim-local/
├── include/          # 所有头文件混在一起
├── src/              # 所有源文件混在一起
├── plugins/          # 内置插件
└── external_plugins/ # 外部插件
```

### 建议结构 (可选)

```
navsim-local/
├── platform/                    # 平台核心 (稳定)
│   ├── include/
│   ├── src/
│   └── proto/
├── cmake/                       # CMake 辅助函数
│   └── NavSimPluginHelpers.cmake
├── tools/                       # 🔧 开发工具 (脚手架)
│   ├── create_plugin.py         # 创建插件项目
│   ├── package_plugin.py        # 打包插件
│   └── install_plugin.py        # 安装插件
├── templates/                   # 📋 插件模板 (供脚手架使用)
│   ├── planner_plugin/          # 规划器插件模板
│   │   ├── algorithm/
│   │   │   ├── include/
│   │   │   │   └── {{plugin_name}}.hpp
│   │   │   └── src/
│   │   │       └── {{plugin_name}}.cpp
│   │   ├── adapter/
│   │   │   ├── include/
│   │   │   │   └── {{plugin_name}}_adapter.hpp
│   │   │   └── src/
│   │   │       └── {{plugin_name}}_adapter.cpp
│   │   ├── tests/
│   │   │   └── test_{{plugin_name}}.cpp
│   │   ├── CMakeLists.txt
│   │   └── plugin.json
│   └── perception_plugin/       # 感知插件模板
│       └── ...
├── config/                      # 配置文件
│   ├── default.json             # 在线模式配置 (现有)
│   └── plugins.json             # 🆕 本地调试模式配置 (新增)
├── plugins/                     # 🏠 内置插件 (平台自带)
│   ├── perception/
│   │   └── default_perception/  # 平台默认感知插件
│   └── planning/
│       └── default_planner/     # 平台默认规划器
├── scenarios/                   # 🆕 测试场景库
│   ├── simple_corridor.json
│   ├── urban_intersection.json
│   └── ...
├── apps/                        # 应用程序
│   ├── navsim_algo.cpp         # 在线模式 (现有)
│   └── navsim_local_debug.cpp  # 🆕 本地调试模式 (新增)
└── external_plugins/            # 🌍 外部插件 (用户开发)
    ├── MyPlanner/               # 用户插件示例 1
    ├── AdvancedPerception/      # 用户插件示例 2
    └── README.md                # 外部插件开发指南
```

### 目录说明

#### 1. `config/` - 配置文件 ⚙️

**用途**: 存放不同运行模式的配置文件

**文件说明**:

**`config/default.json`** - 在线模式配置 (现有):
- 用于 `navsim_algo` 程序 (与 navsim-online 联合仿真)
- 配置感知插件、规划器、算法参数
- **统一的插件加载方式** ✅
- 改进后的示例:
  ```json
  {
    "perception": {
      "plugins": [
        {
          "plugin": "GridMapBuilder",  // 短名称，自动查找
          "params": {...}
        },
        {
          "plugin": "/home/user/MyPerception/build/libmy_perception.so",  // 完整路径
          "params": {...}
        }
      ]
    },
    "planning": {
      "primary_planner": {
        "plugin": "/home/user/MyPlanner/build/libmy_planner.so",  // 用户插件
        "params": {...}
      }
    }
  }
  ```

**`config/plugins.json`** - 本地调试模式配置 (新增):
- 用于 `navsim_local_debug` 程序 (独立运行)
- **统一的插件加载方式** ✅
- 示例:
  ```json
  {
    "plugins": {
      "planner": {
        "plugin": "AStarPlanner",  // 短名称或完整路径
        "params": {...}
      }
    }
  }
  ```

**关键点**:
- ✅ **统一的插件加载方式** - 所有插件都是 `.so` 文件
- ✅ **支持短名称** - 平台插件无需写完整路径
- ✅ **自动查找** - 系统在默认目录中查找插件
- ✅ **两种模式都支持所有插件** - 平台插件和用户插件

#### 2. `templates/` - 插件模板 📋

**用途**: 供脚手架工具 `tools/create_plugin.py` 使用的模板文件

**工作流程**:
```bash
# 用户运行脚手架工具
python3 tools/create_plugin.py MyPlanner planner

# 脚手架工具会:
# 1. 读取 templates/planner_plugin/ 目录
# 2. 复制模板文件到 external_plugins/MyPlanner/
# 3. 替换模板变量 (如 {{plugin_name}} → MyPlanner)
# 4. 生成完整的插件项目结构
```

**模板内容**:
- `algorithm/` - 用户算法层的模板代码
- `adapter/` - Adapter 层的模板代码 (自动生成的转换逻辑)
- `tests/` - 单元测试模板
- `CMakeLists.txt` - 构建配置模板
- `plugin.json` - 插件元信息模板

**模板变量**:
- `{{plugin_name}}` - 插件名称 (如 MyPlanner)
- `{{plugin_type}}` - 插件类型 (planner/perception)
- `{{author}}` - 作者名称
- `{{description}}` - 插件描述

**示例**: `templates/planner_plugin/algorithm/include/{{plugin_name}}.hpp`
```cpp
// 这是模板文件,{{plugin_name}} 会被替换为实际的插件名
#pragma once

namespace {{plugin_name}} {

class {{plugin_name}} {
public:
  PlannerOutput plan(const PlannerInput& input);
};

}  // namespace {{plugin_name}}
```

**用户无需直接修改 `templates/` 目录**,这些文件只供脚手架工具使用。

#### 3. `plugins/` vs `external_plugins/` - 内置插件 vs 外部插件

**区别**:

| 维度 | `plugins/` (内置插件) | `external_plugins/` (外部插件) |
|------|----------------------|-------------------------------|
| **用途** | 平台自带的默认插件 | 用户开发的第三方插件 |
| **维护者** | 平台开发团队 | 插件开发者 |
| **编译方式** | 与平台一起编译 | 独立编译 (可选) |
| **版本管理** | 跟随平台版本 | 独立版本 |
| **分发方式** | 随平台分发 | 独立分发 |
| **示例** | 默认规划器、默认感知 | 用户自定义算法 |

**`plugins/` (内置插件)** 🏠:
- **作用**: 提供平台的默认功能,保证平台开箱即用
- **示例**:
  - `plugins/planning/default_planner/` - 简单的 A* 规划器
  - `plugins/perception/default_perception/` - 基础的障碍物检测
- **特点**:
  - 与平台代码一起维护
  - 作为参考实现,展示如何编写插件
  - 用户可以直接使用,也可以替换为自己的插件
- **编译**: 在平台的 CMakeLists.txt 中编译

**`external_plugins/` (外部插件)** 🌍:
- **作用**: 用户开发的第三方插件,扩展平台功能
- **示例**:
  - `external_plugins/MyPlanner/` - 用户的自定义规划器
  - `external_plugins/AdvancedPerception/` - 用户的高级感知算法
- **特点**:
  - 独立于平台代码
  - 可以独立编译和分发
  - 用户完全控制代码和版本
- **编译**: 独立的 CMakeLists.txt,可选择性编译

**是否可以只保留一个目录?**

**选项 A**: 保留两个目录 (推荐) ✅
- **优势**:
  - 清晰区分平台代码和用户代码
  - 内置插件作为参考实现,降低学习成本
  - 用户可以专注于 `external_plugins/`,不会被平台代码干扰
- **劣势**:
  - 目录结构稍复杂

**选项 B**: 只保留 `plugins/`,不区分内置和外部
- **优势**:
  - 目录结构更简单
- **劣势**:
  - 平台插件和用户插件混在一起,不易管理
  - 用户可能误修改平台插件
  - 不利于版本控制 (用户插件不应提交到平台仓库)

**建议**: 保留两个目录,清晰区分平台代码和用户代码

**注意**: 目录重组是**可选的**,可以保持现有结构,只添加新组件。

---

## 🎯 实施计划

### 实施策略 ✅

**已决策**: 分阶段实施,逐步验证

**选择理由**:
- ✅ 快速验证价值,风险可控
- ✅ 可以根据反馈调整方案
- ✅ 每个阶段都有可交付成果
- ✅ 避免一次性投入过大

**总时间**: 4-6 周 (分阶段,可根据反馈调整)

---

### 阶段 0: 目录结构重组 (1-2 天)

**已决策**: 重组为 `platform/` + `tools/` + `apps/`

**任务清单**:
- [ ] 使用 `git mv` 移动文件 (保留历史)
  - [ ] `include/` + `src/` → `platform/`
  - [ ] 创建 `apps/` 目录
  - [ ] 创建 `tools/`, `templates/`, `config/`, `scenarios/` 目录
- [ ] 更新所有 `#include` 路径
- [ ] 更新 CMakeLists.txt 文件
- [ ] 确保编译通过
- [ ] 更新文档中的路径引用

**目标结构**:
```
navsim-local/
├── platform/         # 平台核心
│   ├── include/
│   ├── src/
│   └── proto/
├── cmake/            # CMake 辅助函数
├── tools/            # 脚手架工具
├── templates/        # 插件模板
├── config/           # 配置文件
│   ├── default.json      # 在线模式配置
│   └── plugins.json      # 本地调试模式配置
├── scenarios/        # 测试场景 (JSON 格式)
├── apps/             # 应用程序
│   ├── navsim_algo.cpp         # 在线模式
│   └── navsim_local_debug.cpp  # 本地调试模式
├── plugins/          # 内置插件
└── external_plugins/ # 外部插件示例
```

**注意事项**:
- 使用 `git mv` 保留文件历史
- 分步骤进行,每步都确保编译通过
- 更新 CI/CD 配置 (如果有)

---

### 阶段 1: 本地调试模式 (2-3 周) - **核心价值** 🎯

**必须完成**,这是核心价值:

**任务清单**:
- [ ] **统一的插件加载机制**
  - [ ] 实现 `PluginLoader::resolvePluginPath()` (短名称 + 完整路径)
  - [ ] 实现插件查找逻辑 (5 个默认路径)
  - [ ] 实现插件版本兼容性检查 (API 版本 + 平台版本)
  - [ ] 支持 `$NAVSIM_PLUGIN_PATH` 环境变量
- [ ] **ScenarioLoader**
  - [ ] JSON 解析 (与 navsim-online 格式一致)
  - [ ] 转换为 PlanningContext
  - [ ] 错误处理和验证
- [ ] **navsim_local_debug 程序**
  - [ ] 命令行参数解析 (`--scenario`, `--config`, `--plugin`)
  - [ ] 插件加载
  - [ ] 场景加载
  - [ ] 规划执行
  - [ ] ImGui 可视化 (场景 + 轨迹)
- [ ] **示例场景**
  - [ ] 创建 3-5 个 JSON 场景文件
  - [ ] 简单走廊、十字路口、停车场等
- [ ] **文档**
  - [ ] 本地调试模式使用指南
  - [ ] 插件配置说明
  - [ ] 场景格式说明

**交付物**: 静态场景测试工具,可独立运行

**简化设计**:
- ✅ 不需要动态仿真循环
- ✅ 不需要时间步更新
- ✅ 不需要播放/暂停/单步控制
- ✅ 只需要: 加载场景 → 规划一次 → 可视化结果

**成功标准**:
- ✅ 可以独立运行,无需 navsim-online
- ✅ 可以使用短名称加载平台插件 (如 `"JpsPlanner"`)
- ✅ 可以使用完整路径加载用户插件
- ✅ 可以可视化规划结果
- ✅ 有完整的使用文档

---

### 验证和反馈 (1 周)

**目标**: 实际使用并收集反馈

**活动**:
- [ ] 实际使用本地调试模式测试现有插件
- [ ] 收集用户反馈
- [ ] 评估是否需要调整方案
- [ ] 决定是否继续后续阶段

---

### 阶段 2: 插件开发工具 (1-2 周) - **提升体验** ✨

**重要但不紧急**,可以后续完善:

**任务清单**:
- [ ] **脚手架工具** (`tools/create_plugin.py`)
  - [ ] 命令行参数解析
  - [ ] 从 `templates/` 复制模板
  - [ ] 替换模板变量 (插件名称等)
  - [ ] 生成完整的项目结构
- [ ] **插件模板** (`templates/`)
  - [ ] `planner_plugin/` 模板
  - [ ] `perception_plugin/` 模板
  - [ ] 包含 algorithm/, adapter/, tests/, CMakeLists.txt
- [ ] **CMake 辅助函数** (`cmake/NavSimPluginHelpers.cmake`)
  - [ ] `navsim_add_plugin()` 函数
  - [ ] 自动链接依赖
- [ ] **文档**
  - [ ] 插件开发快速入门
  - [ ] 三层架构说明
  - [ ] 示例插件教程

**交付物**: 5 分钟创建插件的工具链

---

### 阶段 3: 打包与分发 (1 周) - **锦上添花** 🎁

**锦上添花**,可以最后实现:

**任务清单**:
- [ ] **打包工具** (`tools/package_plugin.py`)
  - [ ] 打包插件为 `.tar.gz`
  - [ ] 生成插件元信息 JSON
- [ ] **安装工具** (`tools/install_plugin.py`)
  - [ ] 从 `.tar.gz` 安装插件
  - [ ] 复制到 `~/.navsim/plugins/`
- [ ] **插件元信息格式**
  - [ ] 定义 `plugin.json` 格式
  - [ ] 包含版本、依赖、API 版本等
- [ ] **文档**
  - [ ] 插件分发指南
  - [ ] 插件安装指南

**交付物**: 标准化的插件分发机制

---

## ✅ 已确认的关键决策

### 1. 三层解耦架构 ✅

**决策**: 采用三层架构 (algorithm + adapter + platform)

**理由**:
- ✅ 算法与平台完全解耦,可独立测试和复用
- ✅ 降低用户学习成本,只需理解标准数据结构
- ✅ Adapter 层由脚手架自动生成,用户无需关心
- ✅ 性能损失可忽略,长期收益远大于短期成本

### 2. Adapter 层完全自动生成 ✅

**决策**: 脚手架工具自动生成完整的 Adapter 代码

**理由**:
- ✅ 用户无需手动编写或修改 Adapter
- ✅ 降低开发门槛,5 分钟创建插件
- ✅ 保证代码质量和一致性
- ✅ 生成完整的默认实现,覆盖常见场景 (起点、终点、障碍物等)
- ✅ 用户 90% 的情况下无需修改,特殊需求可手动调整

### 3. 算法层数据结构标准 ✅

**决策**: 使用 Eigen + STL (std::vector, std::map 等)

**理由**:
- ✅ 通用,易学,性能好
- ✅ 不定义自定义数据结构,降低学习成本
- ✅ 与主流算法库兼容

### 4. 完全开源,无独立 SDK 包 ✅

**决策**: 用户可以直接访问和修改平台代码

**理由**:
- ✅ navsim-local 是开源项目,所有源码可见
- ✅ 提供开发工具 (`tools/`, `cmake/`, `templates/`),不是独立 SDK 包
- ✅ 用户可以根据需要修改平台代码
- ✅ 不使用 `find_package()`,直接引用平台头文件

### 5. 目录结构重组 ✅

**决策**: 重组为 `platform/` + `tools/` + `apps/`

**理由**:
- ✅ 职责清晰,便于长期维护
- ✅ 更符合"平台化"的定位
- ✅ 为未来扩展预留空间
- ✅ 清晰区分平台核心代码和应用程序

### 6. 统一的插件加载方式 ✅

**决策**: 所有插件都是 `.so` 文件,支持短名称和完整路径

**理由**:
- ✅ 简洁性 - 只需一个 `plugin` 字段,无需 `type` 字段
- ✅ 统一性 - 所有插件加载方式一致
- ✅ 灵活性 - 支持短名称 (平台插件) 和完整路径 (用户插件)
- ✅ 易用性 - 平台插件无需写完整路径,自动查找

**插件查找路径**:
1. `plugins/planning/lib{name}.so`
2. `plugins/perception/lib{name}.so`
3. `~/.navsim/plugins/lib{name}.so`
4. `external_plugins/{name}/build/lib{name}.so`
5. `$NAVSIM_PLUGIN_PATH/lib{name}.so`

### 7. 场景定义格式 ✅

**决策**: JSON 格式,与 navsim-online 保存格式一致

**理由**:
- ✅ 易读易写,工具支持好
- ✅ 可以用 JSON Schema 验证
- ✅ 易于版本控制
- ✅ 与 navsim-online 保存格式一致,可直接导出/导入
- ✅ 无需额外依赖 (如 Python 解释器)

### 8. 实施节奏 ✅

**决策**: 分阶段实施,逐步验证

**理由**:
- ✅ 快速验证价值,风险可控
- ✅ 可以根据反馈调整方案
- ✅ 每个阶段都有可交付成果
- ✅ 避免一次性投入过大

**实施顺序**:
1. 阶段 0: 目录结构重组 (1-2 天)
2. 阶段 1: 本地调试模式 (2-3 周) - 核心价值
3. 验证和反馈 (1 周)
4. 阶段 2: 插件开发工具 (1-2 周) - 提升体验
5. 阶段 3: 打包与分发 (1 周) - 锦上添花

### 9. 版本管理策略 ✅

**决策**: 语义化版本 (Semantic Versioning) + API 版本号

**理由**:
- ✅ 清晰的版本语义 (MAJOR.MINOR.PATCH)
- ✅ API 稳定性承诺
- ✅ 插件版本兼容性检查
- ✅ 便于长期维护和演进

**版本规则**:
- **MAJOR**: 不兼容的 API 变更
- **MINOR**: 向后兼容的新功能
- **PATCH**: 向后兼容的 bug 修复

**API 版本号**:
- `NAVSIM_API_VERSION = 2` (整数)
- 插件声明所需的 API 版本
- 加载时检查兼容性

**版本示例**:
- `v2.0.0` - 平台化重构 (本次重构)
- `v2.1.0` - 新增插件热重载功能
- `v2.1.1` - 修复插件加载 bug
- `v3.0.0` - 重大 API 变更

---

## 📊 预期收益

| 指标 | 当前 | 重构后 | 改进 |
|------|------|--------|------|
| **插件开发时间** | 2-4 小时 | < 30 分钟 | **80% ↓** |
| **调试依赖** | 需要 navsim-online | 完全独立 | **100% 独立** |
| **首次编译成功率** | ~50% | > 90% | **80% ↑** |
| **测试迭代周期** | 5-10 分钟 | < 10 秒 | **95% ↓** |
| **算法复用性** | 无法复用 | 可用于其他项目 | **100% ↑** |
| **学习成本** | 需学习平台 API | 只需 Eigen + STL | **70% ↓** |

---

## 🚀 下一步

### ✅ 已完成的决策

以下关键决策已经确认:

1. ✅ **三层解耦架构** - algorithm + adapter + platform
2. ✅ **Adapter 完全自动生成** - 脚手架工具生成完整代码
3. ✅ **算法层数据结构** - Eigen + STL
4. ✅ **完全开源** - 用户可直接访问和修改平台代码
5. ✅ **目录结构重组** - platform/ + tools/ + apps/
6. ✅ **统一的插件加载方式** - 所有插件都是 `.so` 文件
7. ✅ **场景定义格式** - JSON 格式
8. ✅ **实施节奏** - 分阶段实施
9. ✅ **版本管理策略** - 语义化版本 + API 版本号

### 📋 立即开始: 阶段 0 + 阶段 1

**阶段 0: 目录结构重组** (1-2 天)
- [ ] 使用 `git mv` 移动文件
- [ ] 更新 `#include` 路径和 CMakeLists.txt
- [ ] 确保编译通过

**阶段 1: 本地调试模式** (2-3 周)
- [ ] 实现统一的插件加载机制
- [ ] 实现 ScenarioLoader
- [ ] 实现 navsim_local_debug 程序
- [ ] 创建示例场景文件
- [ ] 编写使用文档

**成功标准**:
- ✅ 可以独立运行,无需 navsim-online
- ✅ 可以使用短名称加载平台插件
- ✅ 可以使用完整路径加载用户插件
- ✅ 可以可视化规划结果
- ✅ 有完整的使用文档

### 🔍 后续优化 (可选)

以下功能可以在后续版本中考虑:

1. **插件高级功能**:
   - 支持多个规划器同时加载 (用于对比测试)
   - 支持插件热重载 (运行时切换插件)
   - 支持插件链 (多个插件串联)

2. **场景生成**:
   - 支持 Python 脚本生成复杂场景
   - Python 脚本最终输出 JSON 格式

3. **技术细节**:
   - 插件加载失败的错误处理策略
   - 插件参数验证机制
   - 插件性能监控和日志

---

## 📝 附录

### A. 核心接口示例

**ScenarioLoader 接口** (简化设计):

```cpp
class ScenarioLoader {
public:
  // 从 JSON 加载场景 (格式与 navsim-online 一致)
  bool loadFromJson(const std::string& json_file);

  // 获取场景数据
  const ScenarioData& getScenario() const;

  // 转换为 PlanningContext
  planning::PlanningContext toPlanningContext() const;
};
```

**navsim_local_debug 主程序** (简化设计):

```cpp
int main(int argc, char** argv) {
  // 1. 加载场景
  ScenarioLoader loader;
  loader.loadFromJson(scenario_file);

  // 2. 加载插件
  AlgorithmManager algo_mgr;
  algo_mgr.loadPlugins(config_file);

  // 3. 构造规划上下文
  auto context = loader.toPlanningContext();

  // 4. 调用规划器
  planning::Trajectory trajectory;
  algo_mgr.plan(context, trajectory);

  // 5. 可视化结果
  Visualizer viz;
  viz.drawScenario(loader.getScenario());
  viz.drawTrajectory(trajectory);
  viz.show();  // 显示窗口,用户可以查看结果

  return 0;
}
```

**关键点**:
- ✅ **极简设计** - 只做静态场景测试,不做动态仿真
- ✅ **格式兼容** - 场景 JSON 与 navsim-online 保存格式一致
- ✅ **一次规划** - 加载场景 → 规划一次 → 可视化结果

### B. 参考资料

- 现有文档: `PLUGIN_SYSTEM_README.md`
- 现有文档: `external_plugins/README.md`
- 现有代码: `src/plugin/framework/`

---

**请审阅本方案,并提供您的反馈和建议!** 🙏


# JPS 规划器适配快速指南

## 🎯 适配目标

将基于 ROS 的 JPS 规划器适配到 NavSim-Local 插件系统。

**重要说明**：这不是简单的"包装"，而是完整的移植！

需要移植的内容：
1. ✅ **GraphSearch** - JPS 核心算法
2. ✅ **JPSPlanner 核心逻辑** - 路径优化、轨迹生成、时间规划
3. ✅ **SDFmap 接口** - 通过 JPSGridAdapter 适配
4. ✅ **插件接口** - 对接 NavSim 系统

---

## 📋 适配检查清单

### ✅ 第一步：理解现有代码

- [x] 阅读 `JPS_PLANNER_ANALYSIS.md` 了解算法原理
- [x] 阅读 `JPS_COMPLETE_ADAPTATION_PLAN.md` 了解完整架构
- [ ] 理解 `GraphSearch` 核心算法
- [ ] 理解 `JPSPlanner` 的所有功能（不仅仅是搜索！）
- [ ] 识别 ROS 依赖项
- [ ] 识别 SDFmap 依赖

### ✅ 第二步：创建插件框架

**完整文件结构**：
```
plugins/planning/jps_planner/
├── CMakeLists.txt
├── include/
│   ├── jps_planner_plugin.hpp          # 插件接口实现（对接 NavSim）
│   ├── jps_planner_core.hpp            # 规划器核心逻辑（移植 JPSPlanner）
│   ├── jps_graph_search.hpp            # JPS 算法（移植 GraphSearch）
│   ├── jps_grid_adapter.hpp            # 栅格地图适配器（替代 SDFmap）
│   ├── jps_neighbor.hpp                # JPS 邻居查找（移植 JPS2DNeib）
│   ├── jps_state.hpp                   # 搜索节点定义（移植 State）
│   └── jps_trajectory.hpp              # 轨迹数据结构（移植 FlatTrajData）
└── src/
    ├── jps_planner_plugin.cpp          # 插件实现
    ├── jps_planner_core.cpp            # 核心逻辑实现
    ├── jps_graph_search.cpp            # 算法实现
    ├── jps_grid_adapter.cpp            # 适配器实现
    ├── jps_neighbor.cpp                # 邻居查找实现
    └── register.cpp                    # 插件注册
```

**关键说明**：
- `jps_planner_core` 是移植 `JPSPlanner` 的核心，包含路径优化、轨迹生成等
- `jps_graph_search` 是移植 `GraphSearch` 的核心算法
- `jps_grid_adapter` 是适配 `OccupancyGrid` 到 `SDFmap` 接口

### ✅ 第三步：移植核心算法

#### 3.1 创建栅格地图适配器

**`jps_grid_adapter.hpp`**:
```cpp
#pragma once
#include "core/planning_context.hpp"
#include <Eigen/Dense>

namespace navsim {
namespace plugins {
namespace planning {

class JPSGridAdapter {
public:
  JPSGridAdapter(const navsim::planning::OccupancyGrid* grid, double safe_dis);
  
  // 地图尺寸
  int getWidth() const { return width_; }
  int getHeight() const { return height_; }
  
  // 坐标转换
  int coordToId(int x, int y) const;
  Eigen::Vector2i worldToGrid(const Eigen::Vector2d& world) const;
  Eigen::Vector2d gridToWorld(const Eigen::Vector2i& grid) const;
  
  // 碰撞检测
  bool isFree(int x, int y) const;
  bool isOccupied(int x, int y) const;
  bool isInBounds(int x, int y) const;
  
  // 安全距离检测
  bool isFreeWithSafeDis(int x, int y) const;
  
private:
  const navsim::planning::OccupancyGrid* grid_;
  int width_;
  int height_;
  double resolution_;
  Eigen::Vector2d origin_;
  double safe_dis_;
  int safe_dis_cells_;  // 安全距离对应的栅格数
};

} // namespace planning
} // namespace plugins
} // namespace navsim
```

**`jps_grid_adapter.cpp`**:
```cpp
#include "jps_grid_adapter.hpp"
#include <cmath>

namespace navsim {
namespace plugins {
namespace planning {

JPSGridAdapter::JPSGridAdapter(const navsim::planning::OccupancyGrid* grid, 
                               double safe_dis)
    : grid_(grid), safe_dis_(safe_dis) {
  width_ = grid_->width;
  height_ = grid_->height;
  resolution_ = grid_->resolution;
  origin_ = grid_->origin;
  safe_dis_cells_ = static_cast<int>(std::ceil(safe_dis_ / resolution_));
}

int JPSGridAdapter::coordToId(int x, int y) const {
  return y * width_ + x;
}

Eigen::Vector2i JPSGridAdapter::worldToGrid(const Eigen::Vector2d& world) const {
  Eigen::Vector2i grid;
  grid.x() = static_cast<int>((world.x() - origin_.x()) / resolution_);
  grid.y() = static_cast<int>((world.y() - origin_.y()) / resolution_);
  return grid;
}

Eigen::Vector2d JPSGridAdapter::gridToWorld(const Eigen::Vector2i& grid) const {
  Eigen::Vector2d world;
  world.x() = origin_.x() + (grid.x() + 0.5) * resolution_;
  world.y() = origin_.y() + (grid.y() + 0.5) * resolution_;
  return world;
}

bool JPSGridAdapter::isInBounds(int x, int y) const {
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

bool JPSGridAdapter::isOccupied(int x, int y) const {
  if (!isInBounds(x, y)) return true;
  int idx = coordToId(x, y);
  return grid_->data[idx] > 50;  // 阈值可配置
}

bool JPSGridAdapter::isFree(int x, int y) const {
  return !isOccupied(x, y);
}

bool JPSGridAdapter::isFreeWithSafeDis(int x, int y) const {
  if (!isInBounds(x, y)) return false;
  
  // 检查周围 safe_dis_cells_ 范围内是否有障碍物
  for (int dx = -safe_dis_cells_; dx <= safe_dis_cells_; ++dx) {
    for (int dy = -safe_dis_cells_; dy <= safe_dis_cells_; ++dy) {
      if (dx * dx + dy * dy <= safe_dis_cells_ * safe_dis_cells_) {
        if (isOccupied(x + dx, y + dy)) {
          return false;
        }
      }
    }
  }
  return true;
}

} // namespace planning
} // namespace plugins
} // namespace navsim
```

#### 3.2 移植 GraphSearch

**关键修改**：

1. **移除 ROS 依赖**：
```cpp
// 删除
#include <plan_env/sdf_map.h>

// 添加
#include "jps_grid_adapter.hpp"
```

2. **替换地图接口**：
```cpp
// 原始代码
std::shared_ptr<SDFmap> map_;

// 修改为
std::shared_ptr<JPSGridAdapter> grid_adapter_;
```

3. **修改构造函数**：
```cpp
// 原始
GraphSearch(std::shared_ptr<SDFmap> Map, const double &safe_dis);

// 修改为
GraphSearch(std::shared_ptr<JPSGridAdapter> grid_adapter);
```

4. **修改碰撞检测**：
```cpp
// 原始
inline bool isFree(int x, int y) const {
  if(x < 0 || x >= xDim_ || y < 0 || y >= yDim_)
    return false;
  return !map_->isOccWithSafeDis(x, y, safe_dis_);
}

// 修改为
inline bool isFree(int x, int y) const {
  return grid_adapter_->isFreeWithSafeDis(x, y);
}
```

### ✅ 第四步：实现插件接口

**`jps_planner_plugin.hpp`**:
```cpp
#pragma once

#include "plugin/framework/planner_plugin_interface.hpp"
#include "jps_search_engine.hpp"
#include <memory>

namespace navsim {
namespace plugins {
namespace planning {

class JPSPlannerPlugin : public plugin::PlannerPluginInterface {
public:
  struct Config {
    double safe_dis = 0.3;           // 安全距离 (m)
    int max_iterations = 100000;     // 最大迭代次数
    bool path_optimization = true;   // 是否优化路径
    double time_step = 0.1;          // 时间步长 (s)
    double default_velocity = 2.0;   // 默认速度 (m/s)
    
    static Config fromJson(const nlohmann::json& json);
  };
  
  JPSPlannerPlugin();
  explicit JPSPlannerPlugin(const Config& config);
  
  // 必须实现的接口
  plugin::PlannerPluginMetadata getMetadata() const override;
  bool initialize(const nlohmann::json& config) override;
  bool plan(const navsim::planning::PlanningContext& context,
           std::chrono::milliseconds deadline,
           plugin::PlanningResult& result) override;
  std::pair<bool, std::string> isAvailable(
      const navsim::planning::PlanningContext& context) const override;
  
  // 可选接口
  void reset() override;
  nlohmann::json getStatistics() const override;

private:
  // 辅助方法
  std::vector<plugin::TrajectoryPoint> gridPathToTrajectory(
      const std::vector<Eigen::Vector2i>& grid_path,
      const navsim::planning::OccupancyGrid* grid) const;
  
  Config config_;
  std::unique_ptr<JPSSearchEngine> search_engine_;
  
  struct Statistics {
    size_t total_plans = 0;
    size_t successful_plans = 0;
    size_t failed_plans = 0;
    double total_time_ms = 0.0;
    double avg_path_length = 0.0;
  };
  Statistics stats_;
};

} // namespace planning
} // namespace plugins
} // namespace navsim
```

**`jps_planner_plugin.cpp`** (关键部分):
```cpp
#include "jps_planner_plugin.hpp"
#include <iostream>

namespace navsim {
namespace plugins {
namespace planning {

JPSPlannerPlugin::Config JPSPlannerPlugin::Config::fromJson(const nlohmann::json& json) {
  Config config;
  config.safe_dis = json.value("safe_dis", 0.3);
  config.max_iterations = json.value("max_iterations", 100000);
  config.path_optimization = json.value("path_optimization", true);
  config.time_step = json.value("time_step", 0.1);
  config.default_velocity = json.value("default_velocity", 2.0);
  return config;
}

plugin::PlannerPluginMetadata JPSPlannerPlugin::getMetadata() const {
  return {
    .name = "JPSPlanner",
    .version = "1.0.0",
    .description = "Jump Point Search path planner (adapted from ROS)",
    .type = "search",
    .author = "Adapted to NavSim",
    .can_be_fallback = false,
    .required_perception = {"occupancy_grid"}
  };
}

bool JPSPlannerPlugin::initialize(const nlohmann::json& config) {
  config_ = Config::fromJson(config);
  search_engine_ = std::make_unique<JPSSearchEngine>(config_.safe_dis);
  
  std::cout << "[JPSPlanner] Initialized with:" << std::endl;
  std::cout << "  - safe_dis: " << config_.safe_dis << " m" << std::endl;
  std::cout << "  - max_iterations: " << config_.max_iterations << std::endl;
  std::cout << "  - path_optimization: " << config_.path_optimization << std::endl;
  
  return true;
}

bool JPSPlannerPlugin::plan(const navsim::planning::PlanningContext& context,
                            std::chrono::milliseconds deadline,
                            plugin::PlanningResult& result) {
  auto start_time = std::chrono::steady_clock::now();
  stats_.total_plans++;
  
  // 1. 检查栅格地图
  if (!context.occupancy_grid) {
    result.success = false;
    result.failure_reason = "Missing occupancy grid";
    stats_.failed_plans++;
    return false;
  }
  
  // 2. 创建栅格适配器
  auto grid_adapter = std::make_shared<JPSGridAdapter>(
      context.occupancy_grid.get(), config_.safe_dis);
  
  // 3. 转换坐标
  Eigen::Vector2d start_world(context.ego.pose.x, context.ego.pose.y);
  Eigen::Vector2d goal_world(context.task.goal_pose.x, context.task.goal_pose.y);
  
  Eigen::Vector2i start_grid = grid_adapter->worldToGrid(start_world);
  Eigen::Vector2i goal_grid = grid_adapter->worldToGrid(goal_world);
  
  // 4. 执行 JPS 搜索
  std::vector<Eigen::Vector2i> grid_path;
  bool success = search_engine_->search(
      start_grid, goal_grid, grid_adapter,
      config_.max_iterations, grid_path);
  
  if (!success || grid_path.empty()) {
    result.success = false;
    result.failure_reason = "JPS search failed to find path";
    stats_.failed_plans++;
    return false;
  }
  
  // 5. 路径优化
  if (config_.path_optimization) {
    grid_path = search_engine_->optimizePath(grid_path, grid_adapter);
  }
  
  // 6. 转换为轨迹
  result.trajectory = gridPathToTrajectory(grid_path, context.occupancy_grid.get());
  result.success = true;
  result.planner_name = "JPSPlanner";
  
  auto end_time = std::chrono::steady_clock::now();
  result.computation_time_ms = 
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  stats_.successful_plans++;
  stats_.total_time_ms += result.computation_time_ms;
  
  return true;
}

std::pair<bool, std::string> JPSPlannerPlugin::isAvailable(
    const navsim::planning::PlanningContext& context) const {
  if (!context.occupancy_grid) {
    return {false, "Missing occupancy grid"};
  }
  return {true, ""};
}

std::vector<plugin::TrajectoryPoint> JPSPlannerPlugin::gridPathToTrajectory(
    const std::vector<Eigen::Vector2i>& grid_path,
    const navsim::planning::OccupancyGrid* grid) const {
  
  std::vector<plugin::TrajectoryPoint> trajectory;
  
  double resolution = grid->resolution;
  Eigen::Vector2d origin = grid->origin;
  
  double t = 0.0;
  for (size_t i = 0; i < grid_path.size(); ++i) {
    plugin::TrajectoryPoint pt;
    
    // 转换为世界坐标
    pt.x = origin.x() + (grid_path[i].x() + 0.5) * resolution;
    pt.y = origin.y() + (grid_path[i].y() + 0.5) * resolution;
    
    // 计算航向
    if (i < grid_path.size() - 1) {
      double dx = grid_path[i+1].x() - grid_path[i].x();
      double dy = grid_path[i+1].y() - grid_path[i].y();
      pt.theta = std::atan2(dy, dx);
    } else if (i > 0) {
      pt.theta = trajectory.back().theta;
    }
    
    // 设置速度和时间
    pt.v = config_.default_velocity;
    pt.t = t;
    
    if (i > 0) {
      double dist = std::hypot(pt.x - trajectory.back().x, 
                               pt.y - trajectory.back().y);
      t += dist / config_.default_velocity;
    }
    
    trajectory.push_back(pt);
  }
  
  return trajectory;
}

} // namespace planning
} // namespace plugins
} // namespace navsim
```

### ✅ 第五步：注册插件

**`register.cpp`**:
```cpp
#include "jps_planner_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"

namespace navsim {
namespace plugins {
namespace planning {

void registerJPSPlannerPlugin() {
  static bool registered = false;
  if (!registered) {
    plugin::PlannerPluginRegistry::getInstance().registerPlugin(
        "JPSPlanner",
        []() -> std::shared_ptr<plugin::PlannerPluginInterface> {
          return std::make_shared<JPSPlannerPlugin>();
        });
    registered = true;
  }
}

} // namespace planning
} // namespace plugins
} // namespace navsim

extern "C" {
  void registerJPSPlannerPlugin() {
    navsim::plugins::planning::registerJPSPlannerPlugin();
  }
}

namespace {
struct JPSPlannerPluginInitializer {
  JPSPlannerPluginInitializer() {
    navsim::plugins::planning::registerJPSPlannerPlugin();
  }
};
static JPSPlannerPluginInitializer g_jps_planner_initializer;
}
```

### ✅ 第六步：配置 CMake

**`CMakeLists.txt`**:
```cmake
# JPS Planner Plugin
add_library(jps_planner_plugin SHARED
    src/jps_planner_plugin.cpp
    src/jps_search_engine.cpp
    src/jps_graph_search.cpp
    src/jps_grid_adapter.cpp
    src/register.cpp)

set_target_properties(jps_planner_plugin PROPERTIES
    OUTPUT_NAME "jps_planner_plugin"
    VERSION 1.0.0
    SOVERSION 1)

target_include_directories(jps_planner_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(jps_planner_plugin
    PUBLIC
        navsim_plugin_framework
    PRIVATE
        Eigen3::Eigen)

target_compile_features(jps_planner_plugin PUBLIC cxx_std_17)

message(STATUS "    JPS planner plugin configured")
```

### ✅ 第七步：添加配置

**`config/default.json`**:
```json
{
  "planning": {
    "primary_planner": "JPSPlanner",
    "fallback_planner": "StraightLinePlanner",
    "enable_fallback": true,
    "planners": {
      "JPSPlanner": {
        "safe_dis": 0.3,
        "max_iterations": 100000,
        "path_optimization": true,
        "time_step": 0.1,
        "default_velocity": 2.0
      }
    }
  }
}
```

---

## 🔍 关键注意事项

1. **坐标系转换**：确保世界坐标和栅格坐标转换正确
2. **安全距离**：JPS 使用栅格膨胀实现安全距离
3. **路径优化**：保留原有的 `removeCornerPts` 逻辑
4. **性能优化**：JPS 比 A* 快，但仍需注意超时
5. **边界检查**：所有栅格访问都要检查边界

---

## 📊 测试计划

1. **单元测试**：测试栅格适配器
2. **简单场景**：无障碍物直线路径
3. **复杂场景**：多障碍物环境
4. **性能测试**：与 A* 对比
5. **边界测试**：起点/终点在边界

---

## 🎯 预期效果

- ✅ 比 A* 快 10-100 倍
- ✅ 路径质量与 A* 相同
- ✅ 支持路径优化
- ✅ 完全集成到 NavSim 插件系统


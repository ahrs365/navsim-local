# NavSim-Local 配置快速参考

## 配置文件位置

❌ **当前没有独立的配置文件**（如 `.json`、`.yaml`）

✅ **所有配置都在源代码中**

## 主要配置位置

### 1. 主程序配置

**文件**：`src/main.cpp`  
**行号**：65-69

```cpp
navsim::AlgorithmManager::Config algo_config;
algo_config.primary_planner = "StraightLinePlanner";
algo_config.enable_occupancy_grid = true;
algo_config.enable_bev_obstacles = true;
algo_config.verbose_logging = false;
```

### 2. 感知模块配置

**文件**：`src/algorithm_manager.cpp`

#### 栅格地图（第147-154行）
```cpp
grid_config.resolution = 0.1;        // 栅格分辨率 0.1m
grid_config.map_width = 100.0;       // 地图宽度 100m
grid_config.map_height = 100.0;      // 地图高度 100m
grid_config.inflation_radius = 0.3;  // 膨胀半径 0.3m
```

#### BEV障碍物（第157-164行）
```cpp
bev_config.detection_range = 30.0;       // 检测范围 30m
bev_config.confidence_threshold = 0.5;   // 置信度阈值 0.5
```

#### 动态预测（第166-173行）
```cpp
pred_config.prediction_horizon = 3.0;           // 预测时域 3秒
pred_config.prediction_model = "constant_velocity";  // 恒速模型
```

## 如何修改配置

### 方法1：修改源代码（当前唯一方式）

1. 编辑配置文件（`src/main.cpp` 或 `src/algorithm_manager.cpp`）
2. 重新编译：
   ```bash
   cd navsim-local/build
   cmake ..
   make
   ```

### 方法2：添加配置文件支持（建议）

创建 `config.json`：
```json
{
  "algorithm": {
    "primary_planner": "StraightLinePlanner",
    "enable_occupancy_grid": true,
    "enable_bev_obstacles": true,
    "verbose_logging": false
  },
  "perception": {
    "detection_range": 30.0,
    "inflation_radius": 0.3
  }
}
```

## 常用配置项

| 配置项 | 位置 | 默认值 | 说明 |
|--------|------|--------|------|
| `primary_planner` | main.cpp:66 | "StraightLinePlanner" | 规划器类型 |
| `verbose_logging` | main.cpp:69 | false | 详细日志 |
| `detection_range` | algorithm_manager.cpp:159 | 30.0 | 检测范围(m) |
| `inflation_radius` | algorithm_manager.cpp:151 | 0.3 | 膨胀半径(m) |
| `prediction_horizon` | algorithm_manager.cpp:168 | 3.0 | 预测时域(s) |

## 详细文档

查看 `CONFIG_GUIDE.md` 获取完整的配置说明。


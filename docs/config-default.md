# default.json 配置说明

该文档说明 `config/default.json` 中各字段的含义与典型取值，帮助你在不同场景下调整感知与规划行为。

- **文件路径**：`config/default.json`
- **加载方式**：通过命令行参数 `--config=config/default.json` 传入 `navsim_algo`，或在自定义配置中复制这些条目。

---

## 顶层结构

```jsonc
{
  "perception": { ... },
  "planning":   { ... },
  "algorithm":  { ... }
}
```

### 1. `perception`

#### 1.1 `preprocessing`

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `bev_extraction.enabled` | `true` | 启用鸟瞰图提取管线 |
| `bev_extraction.static_obstacle_inflation` | `0.2` (m) | 对静态障碍物膨胀（安全冗余） |
| `bev_extraction.dynamic_obstacle_inflation` | `0.3` (m) | 对动态障碍物膨胀 |
| `dynamic_prediction.enabled` | `true` | 启用动态障碍预测 |
| `dynamic_prediction.prediction_horizon` | `5.0` (s) | 预测时间范围 |
| `dynamic_prediction.time_step` | `0.1` (s) | 预测步长 |

#### 1.2 `plugins`

感知插件按数组顺序加载，`priority` 数值越小越先执行。常用字段：

| 字段 | 说明 |
|------|------|
| `name` | 插件注册名（需与插件实现一致） |
| `enabled` | 是否启用 |
| `priority` | 执行优先级 |
| `params` | 插件特有参数 |

默认提供两个插件：

- **GridMapBuilder**（默认禁用）：构建栅格地图，核心参数为 `resolution`、地图宽高和 `inflation_radius`（地图内再次膨胀安全距离）。
- **EsdfBuilder**：基于栅格生成 ESDF。`max_distance` 控制可用距离上限，`include_dynamic` 指定是否将动态障碍纳入 ESDF 计算。

### 2. `planning`

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `primary_planner` | `"JpsPlanner"` | 主规划器名称 |
| `fallback_planner` | `"StraightLine"` | 备选规划器，当主规划失败时使用 |
| `enable_fallback` | `true` | 是否启用备选机制 |

#### 2.1 `planners` 子结构

以规划器注册名为键，配置特定参数。

##### StraightLine

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `default_velocity` | `1.5` (m/s) | 规划使用的恒定速度 |
| `time_step` | `0.1` (s) | 采样时间间隔 |
| `planning_horizon` | `5.0` (s) | 前视规划时域长度 |
| `use_trapezoidal_profile` | `true` | 是否应用梯形速度曲线 |
| `max_acceleration` | `1.0` (m/s²) | 速度曲线的最大加速度 |

##### AStarPlanner

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `time_step` | `0.1` (s) | 拓展时的时间分辨率 |
| `heuristic_weight` | `1.2` | A\* 启发式权重（>1 时偏向贪心） |
| `step_size` | `0.5` (m) | 网格步长 |
| `max_iterations` | `10000` | 搜索上限 |
| `goal_tolerance` | `0.5` (m) | 终点位置容差 |
| `default_velocity` | `1.5` (m/s) | 生成轨迹的目标速度 |

##### JpsPlanner

常规参数：

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `safe_dis` | `0.3` (m) | JPS 搜索安全距离 |
| `max_jps_dis` | `10.0` (m) | 最大跳点间距 |
| `distance_weight`, `yaw_weight` | `1.0` | 代价权重 |
| `traj_cut_length` | `600.0` | 轨迹裁剪长度（点数） |
| `max_vel` / `max_acc` / `max_omega` / `max_domega` | `1.5 / 1.0 / 1.0 / 1.0` | 速度、加速度和角速度限制 |
| `sample_time` | `0.1` (s) | 轨迹采样间隔 |
| `min_traj_num` | `3` | 最小候选轨迹数量 |
| `jps_truncation_time` | `5.0` (s) | 跳点规划截断时域 |

`optimizer` 段控制 LBFGS 优化与约束，可按类别理解：

- **运动约束**：`max_domega`, `max_centripetal_acc`, `if_directly_constrain_v_omega`
- **安全区域**：`safeDis`, `finalMinSafeDis`, `finalSafeDisCheckNum`, `safeReplanMaxTime`
- **代价权重**：`time_weight`, `acc_weight`, `domega_weight`, `collision_weight`, `moment_weight`, `mean_time_weight`, `path_*`
- **罚函数参数**：`EqualLambda`, `EqualRho`, `CutEqualLambda`, `CutEqualRho` 等
- **LBFGS 设置**：`path_lbfgs_*` / `lbfgs_*` / `timeResolution` / `sparseResolution`
- **可视化与其它**：`if_visual_optimization`, `hrz_limited`, `hrz_laser_range_dgr`

> 注：部分键以 `_comment_` 开头，仅作说明，不会被程序解析。

### 3. `algorithm`

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `max_computation_time_ms` | `25.0` (ms) | 单帧算法预算上限，由 `AlgorithmManager` 转换为截止时间 |
| `verbose_logging` | `false` | 是否输出详细日志（启用后会打印各阶段耗时和轨迹信息） |
| `goal_hold_distance_` | `2.0` (m) | 自车距离目标小于该值时复用缓存“Hold Trajectory”，防止终点抖动 |

---

## 常见修改建议

1. **切换主/备规划器**：调整 `planning.primary_planner` / `fallback_planner`；确保对应键在 `planners` 中已定义参数。
2. **启用栅格地图**：将 `GridMapBuilder.enabled` 改为 `true`，并根据场景尺寸调整 `map_width` / `map_height`。
3. **缩短预测时域**：降低 `dynamic_prediction.prediction_horizon` 以提高实时性，或提高 `time_step` 减少采样点。
4. **加严安全距离**：增加 `perception.preprocessing.bev_extraction.static_obstacle_inflation` 或 JPS `safeDis`，提升避障冗余。
5. **设置硬实时预算**：根据目标硬件能力调整 `algorithm.max_computation_time_ms`，同时校准规划器内部迭代上限。

如需自定义独立配置文件，可复制 `config/default.json` 并通过 `--config=<your_config>` 加载； `AlgorithmManager::Config` 会按上述字段解析。

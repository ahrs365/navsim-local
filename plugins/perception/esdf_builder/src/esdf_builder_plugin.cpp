#include "esdf_builder_plugin.hpp"
#include "plugin/framework/plugin_registry.hpp"
#include <iostream>
#include <chrono>

namespace navsim {
namespace plugins {
namespace perception {

// ========== 元数据 ==========

plugin::PerceptionPluginMetadata ESDFBuilderPlugin::getMetadata() const {
  plugin::PerceptionPluginMetadata metadata;
  metadata.name = "EsdfBuilder";
  metadata.version = "1.0.0";
  metadata.description = "ESDF (Euclidean Signed Distance Field) map builder";
  metadata.author = "NavSim Team";
  return metadata;
}

// ========== 初始化 ==========

bool ESDFBuilderPlugin::initialize(const nlohmann::json& config) {
  try {
    // 读取配置参数
    if (config.contains("resolution")) {
      resolution_ = config["resolution"].get<double>();
    }
    if (config.contains("map_width")) {
      map_width_ = config["map_width"].get<double>();
    }
    if (config.contains("map_height")) {
      map_height_ = config["map_height"].get<double>();
    }
    if (config.contains("max_distance")) {
      max_distance_ = config["max_distance"].get<double>();
    }
    if (config.contains("include_dynamic")) {
      include_dynamic_ = config["include_dynamic"].get<bool>();
    }

    // 计算栅格尺寸
    grid_width_ = static_cast<int>(std::ceil(map_width_ / resolution_));
    grid_height_ = static_cast<int>(std::ceil(map_height_ / resolution_));
    int grid_size = grid_width_ * grid_height_;

    // 分配内存
    occupancy_grid_.resize(grid_size, 0);

    // 创建并初始化 ESDFMap 对象
    esdf_map_ = std::make_shared<navsim::perception::ESDFMap>();
    navsim::perception::ESDFMap::Config esdf_config;
    esdf_config.resolution = resolution_;
    esdf_config.map_width = map_width_;
    esdf_config.map_height = map_height_;
    esdf_config.max_distance = max_distance_;
    esdf_map_->initialize(esdf_config);

    std::cout << "[ESDFBuilder] Initialized with parameters:" << std::endl;
    std::cout << "  - resolution: " << resolution_ << " m/cell" << std::endl;
    std::cout << "  - map_width: " << map_width_ << " m" << std::endl;
    std::cout << "  - map_height: " << map_height_ << " m" << std::endl;
    std::cout << "  - grid_size: " << grid_width_ << " x " << grid_height_ << " cells" << std::endl;
    std::cout << "  - max_distance: " << max_distance_ << " m" << std::endl;
    std::cout << "  - include_dynamic: " << (include_dynamic_ ? "true" : "false") << std::endl;

    return true;
  } catch (const std::exception& e) {
    std::cerr << "[ESDFBuilder] Initialization failed: " << e.what() << std::endl;
    return false;
  }
}

// ========== 主处理函数 ==========

bool ESDFBuilderPlugin::process(const plugin::PerceptionInput& input, planning::PlanningContext& context) {
  auto start_time = std::chrono::high_resolution_clock::now();

  // 计算地图原点（以自车为中心）
  planning::Point2d origin;
  origin.x = input.ego.pose.x - map_width_ / 2.0;
  origin.y = input.ego.pose.y - map_height_ / 2.0;

  // 1. 构建占据栅格
  auto grid_start = std::chrono::high_resolution_clock::now();
  buildOccupancyGrid(input, origin);
  auto grid_end = std::chrono::high_resolution_clock::now();
  double grid_time_ms = std::chrono::duration<double, std::milli>(grid_end - grid_start).count();

  // 2. 更新 ESDFMap
  Eigen::Vector2d origin_eigen(origin.x, origin.y);
  auto build_start = std::chrono::high_resolution_clock::now();
  esdf_map_->buildFromOccupancyGrid(occupancy_grid_, origin_eigen);
  auto build_end = std::chrono::high_resolution_clock::now();
  double build_time_ms = std::chrono::duration<double, std::milli>(build_end - build_start).count();

  // 3. 计算 ESDF
  auto esdf_start = std::chrono::high_resolution_clock::now();
  esdf_map_->computeESDF();
  auto esdf_end = std::chrono::high_resolution_clock::now();
  double esdf_time_ms = std::chrono::duration<double, std::milli>(esdf_end - esdf_start).count();

  // 4. 创建 NavSim 格式的 ESDF 地图（用于规划器和可视化）
  auto esdf_map_navsim = std::make_unique<planning::ESDFMap>();
  esdf_map_navsim->config.origin = origin;
  esdf_map_navsim->config.resolution = resolution_;
  esdf_map_navsim->config.width = grid_width_;
  esdf_map_navsim->config.height = grid_height_;
  esdf_map_navsim->config.max_distance = max_distance_;
  esdf_map_navsim->data.resize(grid_width_ * grid_height_, max_distance_);

  // 复制距离场数据（保留原始值，包括负值）
  // ⚠️ 重要：planning::ESDFMap 应该保留负值（障碍物内部）
  //    - 正值：自由空间，表示到最近障碍物的距离
  //    - 负值：障碍物内部，表示到最近自由空间的距离
  //    - 规划器需要负值来识别障碍物内部
  // 注意：getDistance() 返回的已经是米单位（computeESDF 中已经乘以 grid_interval_）
  int occupied_count = 0;
  int count_0_05 = 0, count_05_1 = 0, count_1_2 = 0, count_2_3 = 0, count_3_plus = 0;
  double min_dist = std::numeric_limits<double>::max();
  double max_dist = std::numeric_limits<double>::lowest();

  for (int i = 0; i < grid_width_ * grid_height_; ++i) {
    double dist_meter = esdf_map_->getDistance(esdf_map_->vectornum2gridIndex(i));
    // ✅ 保留原始值（包括负值），不取绝对值
    esdf_map_navsim->data[i] = dist_meter;

    // 统计时使用绝对值
    double abs_dist = std::abs(dist_meter);
    if (abs_dist < 0.01) {  // 障碍物
      occupied_count++;
    } else {
      // 统计距离分布
      if (abs_dist < 0.5) count_0_05++;
      else if (abs_dist < 1.0) count_05_1++;
      else if (abs_dist < 2.0) count_1_2++;
      else if (abs_dist < 3.0) count_2_3++;
      else count_3_plus++;
    }

    min_dist = std::min(min_dist, abs_dist);
    max_dist = std::max(max_dist, abs_dist);
  }

  // 每 60 帧打印一次 ESDF 统计信息和计时
  static int esdf_frame_count = 0;
  if (++esdf_frame_count % 60 == 0) {
    std::cout << "[ESDFBuilder] ⏱️  Timing breakdown:" << std::endl;
    std::cout << "  - Build occupancy grid: " << grid_time_ms << " ms" << std::endl;
    std::cout << "  - Build ESDF map: " << build_time_ms << " ms" << std::endl;
    std::cout << "  - Compute ESDF: " << esdf_time_ms << " ms" << std::endl;
    std::cout << "  - Total: " << (grid_time_ms + build_time_ms + esdf_time_ms) << " ms" << std::endl;
    std::cout << "[ESDFBuilder] ESDF stats:\n"
              << "  Occupied cells: " << occupied_count << "\n"
              << "  Distance < 0.5m:  " << count_0_05 << " cells\n"
              << "  Distance 0.5-1m:  " << count_05_1 << " cells\n"
              << "  Distance 1-2m:    " << count_1_2 << " cells\n"
              << "  Distance 2-3m:    " << count_2_3 << " cells\n"
              << "  Distance > 3m:    " << count_3_plus << " cells\n"
              << "  Min distance:     " << min_dist << "m\n"
              << "  Max distance:     " << max_dist << "m" << std::endl;
  }

  // 5. 存储到规划上下文
  context.esdf_map = std::move(esdf_map_navsim);

  // 6. 同时将 perception::ESDFMap 存储到 custom_data 中供 JPS 等规划器使用
  context.setCustomData<navsim::perception::ESDFMap>("perception_esdf_map", esdf_map_);

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  // 每 60 帧打印一次统计信息
  static int frame_count = 0;
  if (++frame_count % 60 == 0) {
    std::cout << "[ESDFBuilder] Processing time: " << duration.count() / 1000.0 << " ms" << std::endl;
  }

  return true;
}

// ========== 构建占据栅格 ==========

void ESDFBuilderPlugin::buildOccupancyGrid(const plugin::PerceptionInput& input, const planning::Point2d& origin) {
  // 清空栅格
  std::fill(occupancy_grid_.begin(), occupancy_grid_.end(), 0);

  // 处理静态障碍物 - 圆形
  for (const auto& circle : input.bev_obstacles.circles) {
    // 计算圆形障碍物覆盖的栅格范围
    int min_x = std::max(0, static_cast<int>((circle.center.x - circle.radius - origin.x) / resolution_));
    int max_x = std::min(grid_width_ - 1, static_cast<int>((circle.center.x + circle.radius - origin.x) / resolution_));
    int min_y = std::max(0, static_cast<int>((circle.center.y - circle.radius - origin.y) / resolution_));
    int max_y = std::min(grid_height_ - 1, static_cast<int>((circle.center.y + circle.radius - origin.y) / resolution_));

    for (int x = min_x; x <= max_x; ++x) {
      for (int y = min_y; y <= max_y; ++y) {
        // 计算栅格中心坐标
        double cell_x = origin.x + (x + 0.5) * resolution_;
        double cell_y = origin.y + (y + 0.5) * resolution_;

        if (isInCircle(cell_x, cell_y, circle)) {
          occupancy_grid_[y * grid_width_ + x] = 100;  // 占据
        }
      }
    }
  }

  // 处理静态障碍物 - 矩形
  for (const auto& rect : input.bev_obstacles.rectangles) {
    // 计算矩形障碍物的包络范围（考虑旋转）
    double half_diag = std::sqrt(rect.width * rect.width + rect.height * rect.height) / 2.0;
    int min_x = std::max(0, static_cast<int>((rect.pose.x - half_diag - origin.x) / resolution_));
    int max_x = std::min(grid_width_ - 1, static_cast<int>((rect.pose.x + half_diag - origin.x) / resolution_));
    int min_y = std::max(0, static_cast<int>((rect.pose.y - half_diag - origin.y) / resolution_));
    int max_y = std::min(grid_height_ - 1, static_cast<int>((rect.pose.y + half_diag - origin.y) / resolution_));

    for (int x = min_x; x <= max_x; ++x) {
      for (int y = min_y; y <= max_y; ++y) {
        double cell_x = origin.x + (x + 0.5) * resolution_;
        double cell_y = origin.y + (y + 0.5) * resolution_;

        if (isInRectangle(cell_x, cell_y, rect)) {
          occupancy_grid_[y * grid_width_ + x] = 100;
        }
      }
    }
  }

  // 处理静态障碍物 - 多边形
  for (const auto& polygon : input.bev_obstacles.polygons) {
    // 计算多边形的包络盒
    if (polygon.vertices.empty()) continue;

    double min_px = polygon.vertices[0].x, max_px = polygon.vertices[0].x;
    double min_py = polygon.vertices[0].y, max_py = polygon.vertices[0].y;
    for (const auto& v : polygon.vertices) {
      min_px = std::min(min_px, v.x);
      max_px = std::max(max_px, v.x);
      min_py = std::min(min_py, v.y);
      max_py = std::max(max_py, v.y);
    }

    int min_x = std::max(0, static_cast<int>((min_px - origin.x) / resolution_));
    int max_x = std::min(grid_width_ - 1, static_cast<int>((max_px - origin.x) / resolution_));
    int min_y = std::max(0, static_cast<int>((min_py - origin.y) / resolution_));
    int max_y = std::min(grid_height_ - 1, static_cast<int>((max_py - origin.y) / resolution_));

    for (int x = min_x; x <= max_x; ++x) {
      for (int y = min_y; y <= max_y; ++y) {
        double cell_x = origin.x + (x + 0.5) * resolution_;
        double cell_y = origin.y + (y + 0.5) * resolution_;

        if (isInPolygon(cell_x, cell_y, polygon)) {
          occupancy_grid_[y * grid_width_ + x] = 100;
        }
      }
    }
  }

  // 处理动态障碍物
  if (include_dynamic_) {
    for (const auto& dyn_obs : input.dynamic_obstacles) {
      // 使用当前位置
      double half_diag = std::sqrt(dyn_obs.width * dyn_obs.width + dyn_obs.length * dyn_obs.length) / 2.0;
      int min_x = std::max(0, static_cast<int>((dyn_obs.current_pose.x - half_diag - origin.x) / resolution_));
      int max_x = std::min(grid_width_ - 1, static_cast<int>((dyn_obs.current_pose.x + half_diag - origin.x) / resolution_));
      int min_y = std::max(0, static_cast<int>((dyn_obs.current_pose.y - half_diag - origin.y) / resolution_));
      int max_y = std::min(grid_height_ - 1, static_cast<int>((dyn_obs.current_pose.y + half_diag - origin.y) / resolution_));

      for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
          double cell_x = origin.x + (x + 0.5) * resolution_;
          double cell_y = origin.y + (y + 0.5) * resolution_;

          if (isInDynamicObstacle(cell_x, cell_y, dyn_obs)) {
            occupancy_grid_[y * grid_width_ + x] = 100;
          }
        }
      }
    }
  }
}

// ========== 辅助函数 ==========

bool ESDFBuilderPlugin::isInCircle(double x, double y, const planning::BEVObstacles::Circle& circle) const {
  double dx = x - circle.center.x;
  double dy = y - circle.center.y;
  double dist_sq = dx * dx + dy * dy;
  return dist_sq <= circle.radius * circle.radius;
}

bool ESDFBuilderPlugin::isInRectangle(double x, double y, const planning::BEVObstacles::Rectangle& rect) const {
  // 将点转换到矩形局部坐标系
  double dx = x - rect.pose.x;
  double dy = y - rect.pose.y;

  double cos_theta = std::cos(-rect.pose.yaw);
  double sin_theta = std::sin(-rect.pose.yaw);

  double local_x = dx * cos_theta - dy * sin_theta;
  double local_y = dx * sin_theta + dy * cos_theta;

  // 检查是否在矩形内
  return std::abs(local_x) <= rect.width / 2.0 && std::abs(local_y) <= rect.height / 2.0;
}

bool ESDFBuilderPlugin::isInPolygon(double x, double y, const planning::BEVObstacles::Polygon& polygon) const {
  // 射线法判断点是否在多边形内
  if (polygon.vertices.size() < 3) return false;

  int count = 0;
  int n = polygon.vertices.size();

  for (int i = 0; i < n; ++i) {
    const auto& v1 = polygon.vertices[i];
    const auto& v2 = polygon.vertices[(i + 1) % n];

    // 检查射线是否与边相交
    if ((v1.y > y) != (v2.y > y)) {
      double x_intersect = (v2.x - v1.x) * (y - v1.y) / (v2.y - v1.y) + v1.x;
      if (x < x_intersect) {
        count++;
      }
    }
  }

  return (count % 2) == 1;
}

bool ESDFBuilderPlugin::isInDynamicObstacle(double x, double y, const planning::DynamicObstacle& obs) const {
  // 将动态障碍物视为矩形
  double dx = x - obs.current_pose.x;
  double dy = y - obs.current_pose.y;

  double cos_theta = std::cos(-obs.current_pose.yaw);
  double sin_theta = std::sin(-obs.current_pose.yaw);

  double local_x = dx * cos_theta - dy * sin_theta;
  double local_y = dx * sin_theta + dy * cos_theta;

  return std::abs(local_x) <= obs.width / 2.0 && std::abs(local_y) <= obs.length / 2.0;
}

void ESDFBuilderPlugin::reset() {
  // 清空占据栅格
  if (!occupancy_grid_.empty()) {
    std::fill(occupancy_grid_.begin(), occupancy_grid_.end(), 0);
  }

  // 重新初始化 ESDF 地图（清空所有缓存）
  if (esdf_map_) {
    navsim::perception::ESDFMap::Config esdf_config;
    esdf_config.resolution = resolution_;
    esdf_config.map_width = map_width_;
    esdf_config.map_height = map_height_;
    esdf_config.max_distance = max_distance_;
    esdf_map_->initialize(esdf_config);
  }

  std::cout << "[ESDFBuilder] Plugin reset complete" << std::endl;
}

} // namespace perception
} // namespace plugins
} // namespace navsim

// 注册插件
namespace {
static navsim::plugin::PerceptionPluginRegistrar<navsim::plugins::perception::ESDFBuilderPlugin>
    esdf_builder_registrar("ESDFBuilder");
}


#include "perception_processor.hpp"
#include <cmath>
#include <algorithm>

namespace navsim {
namespace perception {

// ========== OccupancyGridBuilder ==========

OccupancyGridBuilder::OccupancyGridBuilder() : config_(Config{}) {}

OccupancyGridBuilder::OccupancyGridBuilder(const Config& config)
    : config_(config) {}

bool OccupancyGridBuilder::process(const proto::WorldTick& world_tick,
                                  planning::PlanningContext& context) {
  // 创建栅格地图
  auto grid = std::make_unique<planning::OccupancyGrid>();

  // 配置地图参数
  grid->config.resolution = config_.resolution;
  grid->config.width = static_cast<int>(config_.map_width / config_.resolution);
  grid->config.height = static_cast<int>(config_.map_height / config_.resolution);

  // 以自车为中心的地图
  const auto& ego_pose = world_tick.ego().pose();
  grid->config.origin.x = ego_pose.x() - config_.map_width / 2.0;
  grid->config.origin.y = ego_pose.y() - config_.map_height / 2.0;

  // 初始化栅格数据
  grid->data.resize(grid->config.width * grid->config.height, 0);

  // 添加静态障碍物
  addStaticObstacles(world_tick, *grid);

  // 添加动态障碍物
  addDynamicObstacles(world_tick, *grid);

  // 膨胀处理
  inflateObstacles(*grid);

  // 保存到上下文
  context.occupancy_grid = std::move(grid);

  return true;
}

void OccupancyGridBuilder::addStaticObstacles(const proto::WorldTick& world_tick,
                                             planning::OccupancyGrid& grid) {
  // TODO: 处理静态地图数据
  // 当前版本中，WorldTick 的 map 字段主要包含版本信息
  // 实际的静态障碍物需要根据具体的地图格式来解析
}

void OccupancyGridBuilder::addDynamicObstacles(const proto::WorldTick& world_tick,
                                              planning::OccupancyGrid& grid) {
  // 当前protobuf定义中没有dynamic字段，暂时跳过动态障碍物处理
  // TODO: 更新protobuf定义以包含dynamic obstacles，或直接解析JSON数据

  // 预留实现：
  // for (const auto& obstacle : world_tick.dynamic()) {
  //   const auto& state = obstacle.state();
  //   // 处理动态障碍物...
  // }
}

void OccupancyGridBuilder::inflateObstacles(planning::OccupancyGrid& grid) {
  if (config_.inflation_radius <= 0.0) return;

  int inflation_cells = static_cast<int>(std::ceil(config_.inflation_radius / grid.config.resolution));
  std::vector<uint8_t> inflated_data = grid.data;

  for (int y = 0; y < grid.config.height; ++y) {
    for (int x = 0; x < grid.config.width; ++x) {
      int index = y * grid.config.width + x;
      if (grid.data[index] >= config_.obstacle_cost) {
        // 膨胀障碍物
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            if (dx * dx + dy * dy <= inflation_cells * inflation_cells) {
              int nx = x + dx;
              int ny = y + dy;
              if (nx >= 0 && nx < grid.config.width && ny >= 0 && ny < grid.config.height) {
                int nindex = ny * grid.config.width + nx;
                inflated_data[nindex] = std::max(inflated_data[nindex],
                                                static_cast<uint8_t>(config_.obstacle_cost / 2));
              }
            }
          }
        }
      }
    }
  }

  grid.data = std::move(inflated_data);
}

// ========== BEVObstacleExtractor ==========

BEVObstacleExtractor::BEVObstacleExtractor() : config_(Config{}) {}

BEVObstacleExtractor::BEVObstacleExtractor(const Config& config)
    : config_(config) {}

bool BEVObstacleExtractor::process(const proto::WorldTick& world_tick,
                                  planning::PlanningContext& context) {
  auto obstacles = std::make_unique<planning::BEVObstacles>();

  // 提取静态障碍物
  extractStaticObstacles(world_tick, *obstacles);

  // 提取动态障碍物
  extractDynamicObstacles(world_tick, *obstacles);

  // 保存到上下文
  context.bev_obstacles = std::move(obstacles);

  return true;
}

void BEVObstacleExtractor::extractStaticObstacles(const proto::WorldTick& world_tick,
                                                 planning::BEVObstacles& obstacles) {
  // 更新静态地图缓存（如果这个tick包含静态地图）
  if (world_tick.has_static_map()) {
    cached_static_map_ = world_tick.static_map();
    has_cached_static_map_ = true;
  }

  // 如果没有缓存的静态地图，则跳过
  if (!has_cached_static_map_) {
    return;
  }

  const auto& static_map = cached_static_map_;
  const auto& ego_pose = world_tick.ego().pose();

  // 处理静态圆形障碍物
  for (const auto& circle : static_map.circles()) {
    // 检查距离是否在检测范围内
    double dx = circle.x() - ego_pose.x();
    double dy = circle.y() - ego_pose.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= config_.detection_range) {
      planning::BEVObstacles::Circle circle_obs;
      circle_obs.center.x = circle.x();
      circle_obs.center.y = circle.y();
      circle_obs.radius = circle.r();
      circle_obs.confidence = 1.0;  // 静态障碍物置信度为1.0

      obstacles.circles.push_back(circle_obs);
    }
  }

  // 处理静态多边形障碍物（转换为多边形）
  for (const auto& polygon : static_map.polygons()) {
    if (polygon.points().empty()) continue;

    // 检查多边形中心点是否在检测范围内
    double center_x = 0.0, center_y = 0.0;
    for (const auto& point : polygon.points()) {
      center_x += point.x();
      center_y += point.y();
    }
    center_x /= polygon.points().size();
    center_y /= polygon.points().size();

    double dx = center_x - ego_pose.x();
    double dy = center_y - ego_pose.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= config_.detection_range) {
      planning::BEVObstacles::Polygon poly_obs;
      poly_obs.confidence = 1.0;  // 静态障碍物置信度为1.0

      for (const auto& point : polygon.points()) {
        planning::Point2d vertex;
        vertex.x = point.x();
        vertex.y = point.y();
        poly_obs.vertices.push_back(vertex);
      }

      obstacles.polygons.push_back(poly_obs);
    }
  }
}

void BEVObstacleExtractor::extractDynamicObstacles(const proto::WorldTick& world_tick,
                                                  planning::BEVObstacles& obstacles) {
  const auto& ego_pose = world_tick.ego().pose();

  // 处理动态障碍物
  for (const auto& dyn_obs : world_tick.dynamic_obstacles()) {
    // 检查距离是否在检测范围内
    double dx = dyn_obs.pose().x() - ego_pose.x();
    double dy = dyn_obs.pose().y() - ego_pose.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= config_.detection_range) {
      // 根据形状类型处理
      if (dyn_obs.shape().has_circle()) {
        const auto& circle = dyn_obs.shape().circle();
        planning::BEVObstacles::Circle circle_obs;
        circle_obs.center.x = dyn_obs.pose().x();
        circle_obs.center.y = dyn_obs.pose().y();
        circle_obs.radius = circle.r();
        circle_obs.confidence = 0.9;  // 动态障碍物置信度稍低

        obstacles.circles.push_back(circle_obs);
      } else if (dyn_obs.shape().has_rectangle()) {
        const auto& rect = dyn_obs.shape().rectangle();
        planning::BEVObstacles::Rectangle rect_obs;
        rect_obs.pose.x = dyn_obs.pose().x();
        rect_obs.pose.y = dyn_obs.pose().y();
        rect_obs.pose.yaw = dyn_obs.pose().yaw();
        rect_obs.width = rect.w();
        rect_obs.height = rect.h();
        rect_obs.confidence = 0.9;  // 动态障碍物置信度稍低

        obstacles.rectangles.push_back(rect_obs);
      }
    }
  }
}

// ========== DynamicObstaclePredictor ==========

DynamicObstaclePredictor::DynamicObstaclePredictor() : config_(Config{}) {}

DynamicObstaclePredictor::DynamicObstaclePredictor(const Config& config)
    : config_(config) {}

bool DynamicObstaclePredictor::process(const proto::WorldTick& world_tick,
                                      planning::PlanningContext& context) {
  std::vector<planning::DynamicObstacle> predicted_obstacles;

  if (config_.prediction_model == "constant_velocity") {
    predictConstantVelocity(world_tick, predicted_obstacles);
  } else if (config_.prediction_model == "constant_acceleration") {
    predictConstantAcceleration(world_tick, predicted_obstacles);
  }

  // 保存到上下文
  context.dynamic_obstacles = std::move(predicted_obstacles);

  return true;
}

void DynamicObstaclePredictor::predictConstantVelocity(const proto::WorldTick& world_tick,
                                                      std::vector<planning::DynamicObstacle>& obstacles) {
  const auto& ego_pose = world_tick.ego().pose();
  const double detection_range = 50.0;  // 固定检测范围50m

  for (const auto& dyn_obs : world_tick.dynamic_obstacles()) {
    // 检查距离是否在检测范围内
    double dx = dyn_obs.pose().x() - ego_pose.x();
    double dy = dyn_obs.pose().y() - ego_pose.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= detection_range) {
      planning::DynamicObstacle pred_obs;

      // 将string id转换为int hash
      std::hash<std::string> hasher;
      pred_obs.id = static_cast<int>(hasher(dyn_obs.id()) % 10000);
      pred_obs.type = dyn_obs.model();

      // 当前位置和速度
      pred_obs.current_pose.x = dyn_obs.pose().x();
      pred_obs.current_pose.y = dyn_obs.pose().y();
      pred_obs.current_pose.yaw = dyn_obs.pose().yaw();

      pred_obs.current_twist.vx = dyn_obs.twist().vx();
      pred_obs.current_twist.vy = dyn_obs.twist().vy();
      pred_obs.current_twist.omega = dyn_obs.twist().omega();

      // 生成恒定速度预测轨迹
      planning::DynamicObstacle::Trajectory trajectory;
      trajectory.probability = 1.0;  // 单一轨迹概率为1.0

      double dt = config_.time_step;
      int num_steps = static_cast<int>(config_.prediction_horizon / dt);

      for (int i = 0; i <= num_steps; ++i) {
        double t = i * dt;

        planning::Pose2d future_pose;
        future_pose.x = pred_obs.current_pose.x + pred_obs.current_twist.vx * t;
        future_pose.y = pred_obs.current_pose.y + pred_obs.current_twist.vy * t;
        future_pose.yaw = pred_obs.current_pose.yaw + pred_obs.current_twist.omega * t;

        trajectory.poses.push_back(future_pose);
        trajectory.timestamps.push_back(t);
      }

      pred_obs.predicted_trajectories.push_back(trajectory);
      obstacles.push_back(pred_obs);
    }
  }
}

void DynamicObstaclePredictor::predictConstantAcceleration(const proto::WorldTick& world_tick,
                                                          std::vector<planning::DynamicObstacle>& obstacles) {
  // TODO: 实现恒定加速度预测模型
  // 当前简化为恒定速度
  predictConstantVelocity(world_tick, obstacles);
}

// ========== PerceptionPipeline ==========

void PerceptionPipeline::addProcessor(std::unique_ptr<PerceptionProcessor> processor, bool enabled) {
  processors_.push_back({std::move(processor), enabled});
}

void PerceptionPipeline::enableProcessor(const std::string& name, bool enabled) {
  for (auto& entry : processors_) {
    if (entry.processor->getName() == name) {
      entry.enabled = enabled;
      break;
    }
  }
}

bool PerceptionPipeline::process(const proto::WorldTick& world_tick,
                                planning::PlanningContext& context) {
  // 首先转换基础数据
  BasicDataConverter::convertBasicContext(world_tick, context);

  // 依次执行各个处理器
  for (const auto& entry : processors_) {
    if (!entry.enabled) continue;

    if (!entry.processor->process(world_tick, context)) {
      std::cerr << "[PerceptionPipeline] Processor " << entry.processor->getName()
                << " failed" << std::endl;
      return false;
    }
  }

  return true;
}

std::vector<std::pair<std::string, bool>> PerceptionPipeline::getProcessorStatus() const {
  std::vector<std::pair<std::string, bool>> status;
  for (const auto& entry : processors_) {
    status.emplace_back(entry.processor->getName(), entry.enabled);
  }
  return status;
}

// ========== BasicDataConverter ==========

planning::EgoVehicle BasicDataConverter::convertEgo(const proto::WorldTick& world_tick) {
  planning::EgoVehicle ego;

  // 转换位姿
  const auto& pose = world_tick.ego().pose();
  ego.pose = {pose.x(), pose.y(), pose.yaw()};

  // 转换速度
  const auto& twist = world_tick.ego().twist();
  ego.twist = {twist.vx(), twist.vy(), twist.omega()};

  // 时间戳
  ego.timestamp = world_tick.stamp();

  // 车辆参数（从 world_tick 中获取底盘配置）
  if (world_tick.has_chassis()) {
    const auto& chassis = world_tick.chassis();
    ego.kinematics.wheelbase = chassis.wheelbase();

    // 运动限制
    if (chassis.has_limits()) {
      const auto& limits = chassis.limits();
      ego.limits.max_velocity = limits.v_max();
      ego.limits.max_acceleration = limits.a_max();
      ego.limits.max_steer_angle = limits.steer_max();
    } else {
      // 默认限制
      ego.limits.max_velocity = 2.0;
      ego.limits.max_acceleration = 2.0;
      ego.limits.max_steer_angle = 0.0;
    }

    // 打印底盘配置（仅在配置变化时）
    static std::string last_chassis_model;
    if (chassis.model() != last_chassis_model) {
      std::cout << "[Perception] 底盘配置: " << chassis.model()
                << ", wheelbase=" << chassis.wheelbase() << "m"
                << ", v_max=" << ego.limits.max_velocity << "m/s" << std::endl;
      last_chassis_model = chassis.model();
    }
  } else {
    // 使用默认车辆参数（差速底盘）
    ego.kinematics.wheelbase = 0.5;
    ego.limits.max_velocity = 2.0;
    ego.limits.max_acceleration = 2.0;
    ego.limits.max_steer_angle = 0.0;
  }

  return ego;
}

planning::PlanningTask BasicDataConverter::convertTask(const proto::WorldTick& world_tick) {
  planning::PlanningTask task;

  // 转换目标位姿
  const auto& goal = world_tick.goal().pose();
  task.goal_pose = {goal.x(), goal.y(), goal.yaw()};

  // 转换容差
  if (world_tick.goal().has_tol()) {
    const auto& tol = world_tick.goal().tol();
    task.tolerance.position = tol.pos();
    task.tolerance.yaw = tol.yaw();
  }

  // 任务类型 (目前默认为点到点导航)
  task.type = planning::PlanningTask::Type::GOTO_GOAL;

  return task;
}

void BasicDataConverter::convertBasicContext(const proto::WorldTick& world_tick,
                                           planning::PlanningContext& context) {
  // 转换基础数据
  context.ego = convertEgo(world_tick);
  context.task = convertTask(world_tick);
  context.timestamp = world_tick.stamp();

  // 设置规划时域 (当前protobuf定义中没有qos字段)
  // TODO: 更新protobuf定义以包含qos信息，或使用默认值
  // if (world_tick.has_qos() && world_tick.qos().deadline_ms() > 0) {
  //   context.planning_horizon = world_tick.qos().deadline_ms() / 1000.0 * 50;
  // }

  // 使用默认规划时域
  context.planning_horizon = 5.0; // 默认5秒
}

} // namespace perception
} // namespace navsim
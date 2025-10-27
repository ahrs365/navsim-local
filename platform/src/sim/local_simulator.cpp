#include "sim/local_simulator.hpp"
#include "core/scenario_loader.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>

namespace navsim {
namespace sim {

// ========== LocalSimulator::Impl ==========

class LocalSimulator::Impl {
public:
  Impl() = default;
  ~Impl() = default;

  // 配置
  SimulatorConfig config_;
  bool initialized_ = false;

  // 仿真状态
  WorldState world_state_;
  bool is_running_ = false;
  double real_time_factor_ = 1.0;  // 实际运行的时间缩放

  // 初始状态（用于重置）
  WorldState initial_state_;

  // 回调函数
  SimulationStateCallback state_callback_;
  FrameUpdateCallback frame_callback_;
  CollisionCallback collision_callback_;

  // 统计信息
  std::chrono::steady_clock::time_point last_step_time_;
  double accumulated_time_ = 0.0;

  // ========== 内部方法 ==========

  /**
   * @brief 积分动态障碍物运动
   * @param dt 时间步长
   */
  void integrate_dynamic_obstacles(double dt);

  /**
   * @brief 检查并处理碰撞
   */
  void check_and_handle_collisions();

  /**
   * @brief 自车运动积分（如果有外部控制输入）
   * @param dt 时间步长
   */
  void integrate_ego_motion(double dt);

  /**
   * @brief 碰撞检测：点与圆
   */
  bool point_circle_collision(const planning::Point2d& point,
                              const planning::Point2d& center,
                              double radius) const;

  /**
   * @brief 碰撞检测：圆与圆
   */
  bool circle_circle_collision(const planning::Point2d& center1, double radius1,
                               const planning::Point2d& center2, double radius2) const;

  /**
   * @brief 自车几何模型（简化为圆形）
   */
  double get_ego_radius() const;

  /**
   * @brief 角度标准化到 [-π, π]
   */
  double normalize_angle(double angle) const;

  /**
   * @brief 从 PlanningContext 转换动态障碍物
   */
  std::vector<DynamicObstacle> convert_dynamic_obstacles(
      const std::vector<planning::DynamicObstacle>& obstacles) const;

  /**
   * @brief 从 PlanningContext 转换静态障碍物
   */
  std::vector<StaticObstacle> convert_static_obstacles(
      const planning::BEVObstacles& bev_obstacles) const;
};

// ========== LocalSimulator 实现 ==========

LocalSimulator::LocalSimulator() : impl_(std::make_unique<Impl>()) {
  // 设置默认的底盘配置
  auto& chassis = impl_->world_state_.chassis_config;
  chassis.set_model("differential");
  chassis.set_wheelbase(0.0);
  chassis.set_track_width(0.4);

  auto* limits = chassis.mutable_limits();
  limits->set_v_max(2.0);
  limits->set_a_max(2.0);
  limits->set_omega_max(2.0);
  limits->set_steer_max(0.0);

  auto* geometry = chassis.mutable_geometry();
  geometry->set_body_length(0.5);
  geometry->set_body_width(0.5);
  geometry->set_body_height(0.3);
  geometry->set_wheel_radius(0.08);
  geometry->set_wheel_width(0.05);
  geometry->set_front_overhang(0.3);
  geometry->set_rear_overhang(0.2);
  geometry->set_caster_count(2);
  geometry->set_track_width_ratio(0.0);
}

LocalSimulator::~LocalSimulator() = default;

bool LocalSimulator::initialize(const SimulatorConfig& config) {
  impl_->config_ = config;
  impl_->world_state_.timestamp = 0.0;
  impl_->world_state_.frame_id = 0;
  impl_->is_running_ = false;
  impl_->accumulated_time_ = 0.0;

  // 保存初始状态
  impl_->initial_state_ = impl_->world_state_;

  impl_->initialized_ = true;
  std::cout << "[LocalSimulator] Initialized with time_step="
            << config.time_step << "s, time_scale=" << config.time_scale << std::endl;

  return true;
}

bool LocalSimulator::load_scenario(const std::string& scenario_file) {
  if (!impl_->initialized_) {
    std::cerr << "[LocalSimulator] Not initialized" << std::endl;
    return false;
  }

  // 使用现有的场景加载器
  planning::PlanningContext context;
  if (!planning::ScenarioLoader::loadFromFile(scenario_file, context)) {
    std::cerr << "[LocalSimulator] Failed to load scenario: " << scenario_file << std::endl;
    return false;
  }

  // 转换为 WorldState
  impl_->world_state_.ego_pose = context.ego.pose;
  impl_->world_state_.ego_twist = context.ego.twist;
  impl_->world_state_.goal_pose = context.task.goal_pose;

  // 转换动态障碍物
  impl_->world_state_.dynamic_obstacles = impl_->convert_dynamic_obstacles(context.dynamic_obstacles);

  // 转换静态障碍物（从 BEV 数据）
  if (context.bev_obstacles) {
    impl_->world_state_.static_obstacles = impl_->convert_static_obstacles(*context.bev_obstacles);
  }

  // 更新地图版本
  impl_->world_state_.map_version++;

  // 保存为初始状态
  impl_->initial_state_ = impl_->world_state_;

  std::cout << "[LocalSimulator] Loaded scenario: " << scenario_file << std::endl;
  std::cout << "  Ego: (" << impl_->world_state_.ego_pose.x << ", "
            << impl_->world_state_.ego_pose.y << ", "
            << impl_->world_state_.ego_pose.yaw << ")" << std::endl;
  std::cout << "  Goal: (" << impl_->world_state_.goal_pose.x << ", "
            << impl_->world_state_.goal_pose.y << ", "
            << impl_->world_state_.goal_pose.yaw << ")" << std::endl;
  std::cout << "  Dynamic obstacles: " << impl_->world_state_.dynamic_obstacles.size() << std::endl;
  std::cout << "  Static obstacles: " << impl_->world_state_.static_obstacles.size() << std::endl;

  // 调试：打印前3个动态障碍物的速度
  for (size_t i = 0; i < std::min(size_t(3), impl_->world_state_.dynamic_obstacles.size()); ++i) {
    const auto& obs = impl_->world_state_.dynamic_obstacles[i];
    std::cout << "    Obstacle " << i << ": pos=(" << obs.pose.x << ", " << obs.pose.y
              << "), vel=(" << obs.twist.vx << ", " << obs.twist.vy << ")" << std::endl;
  }

  return true;
}

const SimulatorConfig& LocalSimulator::get_config() const {
  return impl_->config_;
}

void LocalSimulator::start() {
  if (!impl_->initialized_) {
    std::cerr << "[LocalSimulator] Not initialized" << std::endl;
    return;
  }

  bool was_running = impl_->is_running_;
  impl_->is_running_ = true;
  impl_->last_step_time_ = std::chrono::steady_clock::now();

  if (!was_running && impl_->state_callback_) {
    impl_->state_callback_(true);
  }

  std::cout << "[LocalSimulator] Started" << std::endl;
}

void LocalSimulator::pause() {
  bool was_running = impl_->is_running_;
  impl_->is_running_ = false;

  if (was_running && impl_->state_callback_) {
    impl_->state_callback_(false);
  }

  std::cout << "[LocalSimulator] Paused" << std::endl;
}

void LocalSimulator::reset() {
  impl_->is_running_ = false;
  impl_->world_state_ = impl_->initial_state_;
  impl_->world_state_.timestamp = 0.0;
  impl_->world_state_.frame_id = 0;
  impl_->accumulated_time_ = 0.0;

  if (impl_->state_callback_) {
    impl_->state_callback_(false);
  }

  std::cout << "[LocalSimulator] Reset to initial state" << std::endl;
  std::cout << "  Ego: (" << impl_->world_state_.ego_pose.x << ", "
            << impl_->world_state_.ego_pose.y << ", "
            << impl_->world_state_.ego_pose.yaw << ")" << std::endl;
  std::cout << "  Dynamic obstacles: " << impl_->world_state_.dynamic_obstacles.size() << std::endl;

  // 调试：打印前3个动态障碍物的速度
  for (size_t i = 0; i < std::min(size_t(3), impl_->world_state_.dynamic_obstacles.size()); ++i) {
    const auto& obs = impl_->world_state_.dynamic_obstacles[i];
    std::cout << "    Obstacle " << i << ": pos=(" << obs.pose.x << ", " << obs.pose.y
              << "), vel=(" << obs.twist.vx << ", " << obs.twist.vy << ")" << std::endl;
  }
}

bool LocalSimulator::step(double dt) {
  if (!impl_->initialized_) {
    std::cerr << "[LocalSimulator] Not initialized" << std::endl;
    return false;
  }

  // 使用默认步长如果未指定
  if (dt <= 0.0) {
    dt = impl_->config_.time_step;
  }

  // 应用时间缩放
  double scaled_dt = dt * impl_->config_.time_scale;

  // 限制最大步长
  if (scaled_dt > impl_->config_.max_time_step) {
    scaled_dt = impl_->config_.max_time_step;
  }

  // 只有运行状态才积分
  if (impl_->is_running_) {
    // 积分动态障碍物
    impl_->integrate_dynamic_obstacles(scaled_dt);

    // 自车运动积分（目前保持静止，等待外部控制）
    impl_->integrate_ego_motion(scaled_dt);

    // 检查碰撞
    impl_->check_and_handle_collisions();

    // 更新时间
    impl_->world_state_.timestamp += scaled_dt;
  }

  // 无论是否运行都更新帧ID
  impl_->world_state_.frame_id++;

  // 触发帧更新回调
  if (impl_->frame_callback_) {
    impl_->frame_callback_(impl_->world_state_);
  }

  return true;
}

bool LocalSimulator::is_running() const {
  return impl_->is_running_;
}

const WorldState& LocalSimulator::get_world_state() const {
  return impl_->world_state_;
}

double LocalSimulator::get_simulation_time() const {
  return impl_->world_state_.timestamp;
}

uint64_t LocalSimulator::get_frame_id() const {
  return impl_->world_state_.frame_id;
}

void LocalSimulator::set_ego_pose(const planning::Pose2d& pose) {
  impl_->world_state_.ego_pose = pose;
  std::cout << "[LocalSimulator] Set ego pose: (" << pose.x << ", " << pose.y << ", " << pose.yaw << ")" << std::endl;
}

void LocalSimulator::set_ego_twist(const planning::Twist2d& twist) {
  impl_->world_state_.ego_twist = twist;
}

void LocalSimulator::set_goal_pose(const planning::Pose2d& pose) {
  impl_->world_state_.goal_pose = pose;
  std::cout << "[LocalSimulator] Set goal pose: (" << pose.x << ", " << pose.y << ", " << pose.yaw << ")" << std::endl;
}

void LocalSimulator::set_goal_tolerance(double pos_tol, double yaw_tol) {
  impl_->world_state_.goal_tolerance_pos = pos_tol;
  impl_->world_state_.goal_tolerance_yaw = yaw_tol;
}

void LocalSimulator::add_static_obstacle(const StaticObstacle& obstacle) {
  impl_->world_state_.static_obstacles.push_back(obstacle);
  impl_->world_state_.map_version++;
}

void LocalSimulator::add_dynamic_obstacle(const DynamicObstacle& obstacle) {
  impl_->world_state_.dynamic_obstacles.push_back(obstacle);
}

void LocalSimulator::remove_dynamic_obstacle(const std::string& id) {
  auto& obstacles = impl_->world_state_.dynamic_obstacles;
  obstacles.erase(std::remove_if(obstacles.begin(), obstacles.end(),
                                 [&id](const DynamicObstacle& obs) { return obs.id == id; }),
                  obstacles.end());
}

void LocalSimulator::clear_obstacles() {
  clear_static_obstacles();
  clear_dynamic_obstacles();
}

void LocalSimulator::clear_static_obstacles() {
  impl_->world_state_.static_obstacles.clear();
  impl_->world_state_.map_version++;
}

void LocalSimulator::clear_dynamic_obstacles() {
  impl_->world_state_.dynamic_obstacles.clear();
}

void LocalSimulator::set_time_scale(double scale) {
  impl_->config_.time_scale = std::max(0.01, scale);  // 最小0.01倍速
  std::cout << "[LocalSimulator] Set time scale: " << impl_->config_.time_scale << "x" << std::endl;
}

double LocalSimulator::get_time_scale() const {
  return impl_->config_.time_scale;
}

void LocalSimulator::set_simulation_state_callback(const SimulationStateCallback& callback) {
  impl_->state_callback_ = callback;
}

void LocalSimulator::set_frame_update_callback(const FrameUpdateCallback& callback) {
  impl_->frame_callback_ = callback;
}

void LocalSimulator::set_collision_callback(const CollisionCallback& callback) {
  impl_->collision_callback_ = callback;
}

bool LocalSimulator::check_collision() const {
  return check_collision_at(impl_->world_state_.ego_pose);
}

bool LocalSimulator::check_collision_at(const planning::Pose2d& pose) const {
  double ego_radius = impl_->get_ego_radius();
  planning::Point2d ego_center{pose.x, pose.y};

  // 检查与静态障碍物的碰撞
  for (const auto& obs : impl_->world_state_.static_obstacles) {
    if (obs.type == StaticObstacle::Type::CIRCLE) {
      if (impl_->circle_circle_collision(ego_center, ego_radius,
                                         obs.circle.center, obs.circle.radius)) {
        return true;
      }
    }
    // TODO: 多边形碰撞检测
  }

  // 检查与动态障碍物的碰撞
  for (const auto& obs : impl_->world_state_.dynamic_obstacles) {
    planning::Point2d obs_center{obs.pose.x, obs.pose.y};
    if (obs.shape == DynamicObstacle::Shape::CIRCLE) {
      if (impl_->circle_circle_collision(ego_center, ego_radius,
                                         obs_center, obs.radius)) {
        return true;
      }
    }
    // TODO: 矩形碰撞检测
  }

  return false;
}

planning::PlanningContext LocalSimulator::to_planning_context() const {
  planning::PlanningContext context;

  // 基础信息
  context.timestamp = impl_->world_state_.timestamp;

  // 自车状态
  context.ego.pose = impl_->world_state_.ego_pose;
  context.ego.twist = impl_->world_state_.ego_twist;

  // 任务信息
  context.task.goal_pose = impl_->world_state_.goal_pose;

  // 动态障碍物（需要转换格式）
  for (const auto& obs : impl_->world_state_.dynamic_obstacles) {
    planning::DynamicObstacle dyn_obs;
    dyn_obs.id = std::stoi(obs.id);  // 转换字符串ID为整数
    dyn_obs.current_pose = obs.pose;
    dyn_obs.current_twist = obs.twist;
    dyn_obs.type = "vehicle";  // 默认类型
    dyn_obs.shape_type = (obs.shape == DynamicObstacle::Shape::CIRCLE) ? "circle" : "rectangle";
    // TODO: 完善形状和预测信息
    context.dynamic_obstacles.push_back(dyn_obs);
  }

  // TODO: 转换静态障碍物到 BEV 格式

  return context;
}

void LocalSimulator::from_world_tick(const proto::WorldTick& world_tick) {
  // 更新自车状态
  if (world_tick.has_ego()) {
    const auto& ego = world_tick.ego();
    if (ego.has_pose()) {
      impl_->world_state_.ego_pose.x = ego.pose().x();
      impl_->world_state_.ego_pose.y = ego.pose().y();
      impl_->world_state_.ego_pose.yaw = ego.pose().yaw();
    }
    if (ego.has_twist()) {
      impl_->world_state_.ego_twist.vx = ego.twist().vx();
      impl_->world_state_.ego_twist.vy = ego.twist().vy();
      impl_->world_state_.ego_twist.omega = ego.twist().omega();
    }
  }

  // 更新目标
  if (world_tick.has_goal()) {
    const auto& goal = world_tick.goal();
    if (goal.has_pose()) {
      impl_->world_state_.goal_pose.x = goal.pose().x();
      impl_->world_state_.goal_pose.y = goal.pose().y();
      impl_->world_state_.goal_pose.yaw = goal.pose().yaw();
    }
  }

  // 更新时间戳
  impl_->world_state_.timestamp = world_tick.stamp();
  impl_->world_state_.frame_id = world_tick.tick_id();

  // TODO: 更新障碍物信息
}

proto::WorldTick LocalSimulator::to_world_tick() const {
  proto::WorldTick world_tick;

  world_tick.set_tick_id(impl_->world_state_.frame_id);
  world_tick.set_stamp(impl_->world_state_.timestamp);

  // 自车状态
  auto* ego = world_tick.mutable_ego();
  auto* ego_pose = ego->mutable_pose();
  ego_pose->set_x(impl_->world_state_.ego_pose.x);
  ego_pose->set_y(impl_->world_state_.ego_pose.y);
  ego_pose->set_yaw(impl_->world_state_.ego_pose.yaw);

  auto* ego_twist = ego->mutable_twist();
  ego_twist->set_vx(impl_->world_state_.ego_twist.vx);
  ego_twist->set_vy(impl_->world_state_.ego_twist.vy);
  ego_twist->set_omega(impl_->world_state_.ego_twist.omega);

  // 目标
  auto* goal = world_tick.mutable_goal();
  auto* goal_pose = goal->mutable_pose();
  goal_pose->set_x(impl_->world_state_.goal_pose.x);
  goal_pose->set_y(impl_->world_state_.goal_pose.y);
  goal_pose->set_yaw(impl_->world_state_.goal_pose.yaw);

  // 底盘配置
  *world_tick.mutable_chassis() = impl_->world_state_.chassis_config;

  // 转换静态障碍物到 static_map
  if (!impl_->world_state_.static_obstacles.empty()) {
    auto* static_map = world_tick.mutable_static_map();

    for (const auto& obs : impl_->world_state_.static_obstacles) {
      if (obs.type == StaticObstacle::Type::CIRCLE) {
        auto* circle = static_map->add_circles();
        circle->set_x(obs.circle.center.x);
        circle->set_y(obs.circle.center.y);
        circle->set_r(obs.circle.radius);
      } else if (obs.type == StaticObstacle::Type::POLYGON) {
        auto* polygon = static_map->add_polygons();
        for (const auto& point : obs.polygon.points) {
          auto* vertex = polygon->add_points();
          vertex->set_x(point.x);
          vertex->set_y(point.y);
          vertex->set_yaw(0.0);  // 多边形顶点没有朝向
        }
      }
    }
  }

  // 转换动态障碍物
  for (const auto& obs : impl_->world_state_.dynamic_obstacles) {
    auto* dyn_obs = world_tick.add_dynamic_obstacles();

    // 设置ID
    dyn_obs->set_id(obs.id);

    // 设置位姿
    auto* pose = dyn_obs->mutable_pose();
    pose->set_x(obs.pose.x);
    pose->set_y(obs.pose.y);
    pose->set_yaw(obs.pose.yaw);

    // 设置速度
    auto* twist = dyn_obs->mutable_twist();
    twist->set_vx(obs.twist.vx);
    twist->set_vy(obs.twist.vy);
    twist->set_omega(obs.twist.omega);

    // 设置形状 (使用 oneof)
    auto* shape = dyn_obs->mutable_shape();
    if (obs.shape == DynamicObstacle::Shape::CIRCLE) {
      auto* circle = shape->mutable_circle();
      circle->set_x(0.0);  // 圆心相对于障碍物位姿的偏移
      circle->set_y(0.0);
      circle->set_r(obs.radius);
    } else if (obs.shape == DynamicObstacle::Shape::RECTANGLE) {
      auto* rect = shape->mutable_rectangle();
      rect->set_w(obs.width);
      rect->set_h(obs.height);
      rect->set_yaw(0.0);  // 矩形相对于障碍物位姿的朝向偏移
    }

    // 设置运动模型
    dyn_obs->set_model(obs.model);
  }

  return world_tick;
}

// ========== LocalSimulator::Impl 内部方法实现 ==========

void LocalSimulator::Impl::integrate_dynamic_obstacles(double dt) {
  for (auto& obs : world_state_.dynamic_obstacles) {
    if (obs.model == "cv") {
      // 恒速模型
      obs.pose.x += obs.twist.vx * dt;
      obs.pose.y += obs.twist.vy * dt;
      obs.pose.yaw = normalize_angle(obs.pose.yaw + obs.twist.omega * dt);
    } else if (obs.model == "ca") {
      // TODO: 恒加速模型
    }
    // "custom" 模型由外部控制，不在这里积分
  }
}

void LocalSimulator::Impl::integrate_ego_motion(double dt) {
  // 目前不积分自车运动，等待外部算法控制
  // 这个函数预留给未来的直接控制模式
  (void)dt;
}

void LocalSimulator::Impl::check_and_handle_collisions() {
  if (!config_.physics.collision_detection) {
    return;
  }

  double ego_radius = get_ego_radius();
  planning::Point2d ego_center{world_state_.ego_pose.x, world_state_.ego_pose.y};

  // 检查与动态障碍物的碰撞
  for (const auto& obs : world_state_.dynamic_obstacles) {
    if (obs.shape == DynamicObstacle::Shape::CIRCLE) {
      planning::Point2d obs_center{obs.pose.x, obs.pose.y};
      if (circle_circle_collision(ego_center, ego_radius, obs_center, obs.radius)) {
        if (collision_callback_) {
          collision_callback_(obs.id);
        }
      }
    }
  }
}

bool LocalSimulator::Impl::point_circle_collision(const planning::Point2d& point,
                                                   const planning::Point2d& center,
                                                   double radius) const {
  double dx = point.x - center.x;
  double dy = point.y - center.y;
  double dist_sq = dx * dx + dy * dy;
  return dist_sq <= radius * radius;
}

bool LocalSimulator::Impl::circle_circle_collision(const planning::Point2d& center1, double radius1,
                                                    const planning::Point2d& center2, double radius2) const {
  double dx = center1.x - center2.x;
  double dy = center1.y - center2.y;
  double dist_sq = dx * dx + dy * dy;
  double radius_sum = radius1 + radius2;
  return dist_sq <= radius_sum * radius_sum;
}

double LocalSimulator::Impl::get_ego_radius() const {
  // 简化为圆形，半径取车体对角线的一半
  const auto& geometry = world_state_.chassis_config.geometry();
  double length = geometry.body_length();
  double width = geometry.body_width();
  return std::sqrt(length * length + width * width) / 2.0;
}

double LocalSimulator::Impl::normalize_angle(double angle) const {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::vector<DynamicObstacle> LocalSimulator::Impl::convert_dynamic_obstacles(
    const std::vector<planning::DynamicObstacle>& obstacles) const {
  std::vector<DynamicObstacle> result;

  for (const auto& obs : obstacles) {
    DynamicObstacle dyn_obs;
    dyn_obs.id = std::to_string(obs.id);  // 转换整数ID为字符串
    dyn_obs.pose = obs.current_pose;
    dyn_obs.twist = obs.current_twist;

    // 设置默认参数
    dyn_obs.shape = DynamicObstacle::Shape::CIRCLE;
    dyn_obs.radius = 0.5;  // 默认半径
    dyn_obs.model = "cv";  // 恒速模型

    result.push_back(dyn_obs);
  }

  return result;
}

std::vector<StaticObstacle> LocalSimulator::Impl::convert_static_obstacles(
    const planning::BEVObstacles& bev_obstacles) const {
  std::vector<StaticObstacle> result;

  // 从 BEV 数据转换圆形障碍物
  for (const auto& circle : bev_obstacles.circles) {
    StaticObstacle obs;
    obs.type = StaticObstacle::Type::CIRCLE;
    obs.circle.center = planning::Point2d{circle.center.x, circle.center.y};
    obs.circle.radius = circle.radius;
    result.push_back(obs);
  }

  // 转换多边形障碍物
  for (const auto& polygon : bev_obstacles.polygons) {
    StaticObstacle obs;
    obs.type = StaticObstacle::Type::POLYGON;
    obs.polygon.points.clear();
    for (const auto& vertex : polygon.vertices) {
      obs.polygon.points.push_back(planning::Point2d{vertex.x, vertex.y});
    }
    result.push_back(obs);
  }

  return result;
}

} // namespace sim
} // namespace navsim
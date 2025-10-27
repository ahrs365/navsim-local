#include "plugin/preprocessing/preprocessing.hpp"
#include <iostream>

namespace navsim {
namespace perception {

planning::EgoVehicle BasicDataConverter::convertEgo(
    const proto::WorldTick& world_tick) {
  planning::EgoVehicle ego;

  // 转换位姿
  const auto& pose = world_tick.ego().pose();
  ego.pose = {pose.x(), pose.y(), pose.yaw()};

  // 转换速度
  const auto& twist = world_tick.ego().twist();
  ego.twist = {twist.vx(), twist.vy(), twist.omega()};

  // 🔍 调试：每秒打印一次规划器接收到的自车状态
  static uint64_t last_log_tick = 0;
  if (world_tick.tick_id() % 30 == 0 && world_tick.tick_id() != last_log_tick) {
    std::cout << "[BasicDataConverter::convertEgo] Planner received ego state:" << std::endl;
    std::cout << "  Pose: (" << ego.pose.x << ", " << ego.pose.y << ", " << ego.pose.yaw << ")" << std::endl;
    std::cout << "  Twist: vx=" << ego.twist.vx << ", vy=" << ego.twist.vy << ", omega=" << ego.twist.omega << std::endl;
    last_log_tick = world_tick.tick_id();
  }

  // 时间戳
  ego.timestamp = world_tick.stamp();

  // 🔧 车辆参数（从 world_tick 中获取底盘配置）
  if (world_tick.has_chassis()) {
    const auto& chassis = world_tick.chassis();

    // 底盘类型
    ego.chassis_model = chassis.model();

    // 基础参数
    ego.kinematics.wheelbase = chassis.wheelbase();
    ego.kinematics.track_width = chassis.track_width();

    // 🔧 几何参数（从 ChassisGeometry 中获取）
    if (chassis.has_geometry()) {
      const auto& geom = chassis.geometry();
      ego.kinematics.body_length = geom.body_length();
      ego.kinematics.body_width = geom.body_width();
      ego.kinematics.width = geom.body_width();  // 兼容旧字段
      ego.kinematics.height = geom.body_height();
      ego.kinematics.front_overhang = geom.front_overhang();
      ego.kinematics.rear_overhang = geom.rear_overhang();
      ego.kinematics.wheel_radius = geom.wheel_radius();
    } else {
      // 🔧 如果没有 geometry，根据底盘类型使用默认值
      if (ego.chassis_model == "differential") {
        // 差速底盘：小型机器人
        ego.kinematics.body_length = ego.kinematics.wheelbase * 1.5;
        ego.kinematics.body_width = ego.kinematics.track_width * 1.2;
        ego.kinematics.width = ego.kinematics.body_width;
        ego.kinematics.height = 0.3;
        ego.kinematics.front_overhang = ego.kinematics.wheelbase * 0.25;
        ego.kinematics.rear_overhang = ego.kinematics.wheelbase * 0.25;
        ego.kinematics.wheel_radius = 0.1;
      } else if (ego.chassis_model == "ackermann" || ego.chassis_model == "four_wheel") {
        // 阿克曼/四轮底盘：标准汽车
        ego.kinematics.body_length = ego.kinematics.wheelbase * 1.7;
        ego.kinematics.body_width = ego.kinematics.track_width * 1.1;
        ego.kinematics.width = ego.kinematics.body_width;
        ego.kinematics.height = 1.5;
        ego.kinematics.front_overhang = ego.kinematics.wheelbase * 0.35;
        ego.kinematics.rear_overhang = ego.kinematics.wheelbase * 0.35;
        ego.kinematics.wheel_radius = 0.3;
      } else if (ego.chassis_model == "tracked") {
        // 履带底盘
        ego.kinematics.body_length = ego.kinematics.wheelbase * 1.4;
        ego.kinematics.body_width = ego.kinematics.track_width;
        ego.kinematics.width = ego.kinematics.body_width;
        ego.kinematics.height = 0.5;
        ego.kinematics.front_overhang = ego.kinematics.wheelbase * 0.2;
        ego.kinematics.rear_overhang = ego.kinematics.wheelbase * 0.2;
        ego.kinematics.wheel_radius = 0.15;
      }
    }

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
  } else {
    // 使用默认车辆参数（差速底盘）
    ego.chassis_model = "differential";
    ego.kinematics.wheelbase = 0.5;
    ego.kinematics.track_width = 0.4;
    ego.kinematics.body_length = 0.75;
    ego.kinematics.body_width = 0.48;
    ego.kinematics.width = 0.48;
    ego.kinematics.height = 0.3;
    ego.kinematics.front_overhang = 0.125;
    ego.kinematics.rear_overhang = 0.125;
    ego.kinematics.wheel_radius = 0.1;
    ego.limits.max_velocity = 2.0;
    ego.limits.max_acceleration = 2.0;
    ego.limits.max_steer_angle = 0.0;
  }

  return ego;
}

planning::PlanningTask BasicDataConverter::convertTask(
    const proto::WorldTick& world_tick) {
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

void BasicDataConverter::convertBasicContext(
    const proto::WorldTick& world_tick,
    planning::PlanningContext& context) {
  // 转换基础数据
  context.ego = convertEgo(world_tick);
  context.task = convertTask(world_tick);
  context.timestamp = world_tick.stamp();

  // 使用默认规划时域
  context.planning_horizon = 5.0; // 默认5秒
}

} // namespace perception
} // namespace navsim


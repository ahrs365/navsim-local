#include "perception/preprocessing.hpp"
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
  } else {
    // 使用默认车辆参数（差速底盘）
    ego.kinematics.wheelbase = 0.5;
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


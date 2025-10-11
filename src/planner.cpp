#include "planner.hpp"

#include <cmath>
#include <vector>

namespace navsim {

namespace {
constexpr double kDefaultHorizon = 5.0;  // seconds
constexpr double kMinHorizon = 2.0;
constexpr double kTimeStep = 0.1;
}

bool Planner::solve(const proto::WorldTick& world,
                    const std::optional<proto::PlanUpdate>& /*last_plan*/,
                    std::chrono::milliseconds deadline,
                    proto::PlanUpdate* out_plan,
                    proto::EgoCmd* out_cmd) const {
  if (!out_plan || !out_cmd) {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto deadline_time = now + deadline;

  proto::PlanUpdate plan;
  plan.set_tick_id(world.tick_id());
  plan.set_stamp(world.stamp());

  const auto& ego_pose = world.ego().pose();
  const auto& goal_pose = world.goal().pose();

  const double dx = goal_pose.x() - ego_pose.x();
  const double dy = goal_pose.y() - ego_pose.y();
  const double distance = std::hypot(dx, dy);
  const double target_horizon = std::max(kMinHorizon, std::max(kDefaultHorizon, distance));
  const int steps = static_cast<int>(target_horizon / kTimeStep);

  // Calculate trajectory heading based on movement direction
  const double trajectory_yaw = std::atan2(dy, dx);

  for (int i = 0; i <= steps; ++i) {
    if (std::chrono::steady_clock::now() > deadline_time) {
      break;
    }
    const double t = i * kTimeStep;
    const double ratio = steps > 0 ? static_cast<double>(i) / steps : 1.0;
    auto* point = plan.add_trajectory();
    point->set_t(t);
    point->set_x(ego_pose.x() + ratio * dx);
    point->set_y(ego_pose.y() + ratio * dy);

    // ✅ 修复：轨迹朝向应该指向运动方向，而不是线性插值到目标朝向
    // 在轨迹中途保持指向目标的朝向，只在最后一个点使用目标朝向
    if (i == steps) {
      // 最后一个点使用目标朝向
      point->set_yaw(goal_pose.yaw());
    } else {
      // 中间点使用轨迹方向作为朝向
      point->set_yaw(trajectory_yaw);
    }
  }

  // Simple feed-forward command: aim towards goal with zero acceleration.
  proto::EgoCmd cmd;
  cmd.set_acceleration(0.0);
  cmd.set_steering(std::atan2(dy, dx));

  out_plan->Swap(&plan);
  out_cmd->Swap(&cmd);
  return true;
}

}  // namespace navsim

#include "perception/preprocessing.hpp"
#include <cmath>
#include <iostream>

namespace navsim {
namespace perception {

std::vector<planning::DynamicObstacle> DynamicObstaclePredictor::predict(
    const proto::WorldTick& world_tick) {
  std::vector<planning::DynamicObstacle> predicted_obstacles;

  if (config_.prediction_model == "constant_velocity") {
    predictConstantVelocity(world_tick, predicted_obstacles);
  }

  total_predictions_++;

  return predicted_obstacles;
}

void DynamicObstaclePredictor::predictConstantVelocity(
    const proto::WorldTick& world_tick,
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

void DynamicObstaclePredictor::reset() {
  total_predictions_ = 0;
}

} // namespace perception
} // namespace navsim


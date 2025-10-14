#include "plugin/preprocessing/preprocessing.hpp"
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

      // 🔧 修复：从 shape 中提取形状类型和尺寸
      if (dyn_obs.shape().has_circle()) {
        const auto& circle = dyn_obs.shape().circle();
        pred_obs.shape_type = "circle";
        pred_obs.length = circle.r() * 2.0;  // 直径
        pred_obs.width = circle.r() * 2.0;   // 直径
      } else if (dyn_obs.shape().has_rectangle()) {
        const auto& rect = dyn_obs.shape().rectangle();
        pred_obs.shape_type = "rectangle";

        // 🔍 调试日志：打印从 protobuf 读取的原始值
        static int debug_count = 0;
        if (debug_count++ % 60 == 0) {
          std::cout << "[DynamicPredictor] Rectangle from protobuf:" << std::endl;
          std::cout << "[DynamicPredictor]   rect.w() = " << rect.w() << std::endl;
          std::cout << "[DynamicPredictor]   rect.h() = " << rect.h() << std::endl;
          std::cout << "[DynamicPredictor]   rect.yaw() = " << rect.yaw() << std::endl;
          std::cout << "[DynamicPredictor]   dyn_obs.pose().yaw() = " << dyn_obs.pose().yaw() << std::endl;
          std::cout << "[DynamicPredictor]   dyn_obs.twist().vx() = " << dyn_obs.twist().vx() << std::endl;
          std::cout << "[DynamicPredictor]   dyn_obs.twist().vy() = " << dyn_obs.twist().vy() << std::endl;
        }

        // 🔧 重要：前端坐标系与后端坐标系的映射关系
        //
        // 前端（navsim-online）的定义：
        //   - rect.w: 矩形在世界坐标系中的 X 方向尺寸
        //   - rect.h: 矩形在世界坐标系中的 Y 方向尺寸
        //
        // 后端（navsim-local）的定义：
        //   - width: 车辆的横向宽度（垂直于运动方向）
        //   - length: 车辆的纵向长度（平行于运动方向）
        //
        // 经过测试发现，前端的 w/h 与后端的 width/length 映射关系需要对调：
        //   - 前端的 w → 后端的 length（车辆长度方向）
        //   - 前端的 h → 后端的 width（车辆宽度方向）
        //
        pred_obs.length = rect.w();  // 前端的 w → 车辆长度
        pred_obs.width = rect.h();   // 前端的 h → 车辆宽度

        if (debug_count % 60 == 0) {
          std::cout << "[DynamicPredictor]   Mapping: rect.w()=" << rect.w()
                    << " -> length=" << pred_obs.length << std::endl;
          std::cout << "[DynamicPredictor]   Mapping: rect.h()=" << rect.h()
                    << " -> width=" << pred_obs.width << std::endl;
        }
      } else {
        // 使用默认值（已在结构体中定义）
        pred_obs.shape_type = "rectangle";
        pred_obs.length = 4.5;
        pred_obs.width = 2.0;
      }

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


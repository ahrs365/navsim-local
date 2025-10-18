#include "straight_line.hpp"
#include <chrono>
#include <cmath>

namespace straight_line {
namespace algorithm {

StraightLine::StraightLine(const Config& config)
    : config_(config) {
}

void StraightLine::setConfig(const Config& config) {
  config_ = config;
}

StraightLine::Result StraightLine::plan(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal) {
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  Result result;
  
  // 计算距离
  double dx = goal.x() - start.x();
  double dy = goal.y() - start.y();
  double distance = std::sqrt(dx * dx + dy * dy);
  
  // 计算轨迹点数量
  int num_points = static_cast<int>(config_.planning_horizon / config_.time_step);
  if (num_points < 2) num_points = 2;
  
  // 生成直线轨迹
  result.path = generateStraightLine(start, goal, num_points);
  
  // 计算速度曲线
  if (config_.use_trapezoidal_profile) {
    computeTrapezoidalProfile(result.path, distance);
  } else {
    computeVelocityProfile(result.path, distance);
  }
  
  result.success = true;
  
  // 计算耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  result.computation_time_ms = duration.count() / 1000.0;
  
  return result;
}

void StraightLine::reset() {
  // 直线规划器无状态，无需重置
}

std::vector<StraightLine::Waypoint> StraightLine::generateStraightLine(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    int num_points) const {
  
  std::vector<Waypoint> trajectory;
  trajectory.reserve(num_points);
  
  double dx = goal.x() - start.x();
  double dy = goal.y() - start.y();
  double distance = std::sqrt(dx * dx + dy * dy);
  
  // 计算朝向
  double heading = std::atan2(dy, dx);
  
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    
    Waypoint wp;
    wp.position.x() = start.x() + dx * t;
    wp.position.y() = start.y() + dy * t;
    wp.position.z() = heading;
    
    wp.timestamp = i * config_.time_step;
    wp.path_length = distance * t;
    
    trajectory.push_back(wp);
  }
  
  return trajectory;
}

void StraightLine::computeVelocityProfile(
    std::vector<Waypoint>& trajectory,
    double total_distance) const {
  // 简单的匀速运动
  for (auto& wp : trajectory) {
    wp.velocity = config_.default_velocity;
    wp.acceleration = 0.0;
  }
}

void StraightLine::computeTrapezoidalProfile(
    std::vector<Waypoint>& trajectory,
    double total_distance) const {
  if (trajectory.empty()) return;
  
  double v_max = config_.default_velocity;
  double a_max = config_.max_acceleration;
  
  // 计算加速、匀速、减速的距离
  double t_accel = v_max / a_max;
  double d_accel = 0.5 * a_max * t_accel * t_accel;
  
  // 如果距离太短，无法达到最大速度
  if (2 * d_accel > total_distance) {
    // 三角形速度曲线
    for (auto& wp : trajectory) {
      double s = wp.path_length;
      
      if (s < total_distance / 2.0) {
        // 加速阶段
        wp.velocity = std::sqrt(2 * a_max * s);
        wp.acceleration = a_max;
      } else {
        // 减速阶段
        double s_remaining = total_distance - s;
        wp.velocity = std::sqrt(2 * a_max * s_remaining);
        wp.acceleration = -a_max;
      }
    }
  } else {
    // 梯形速度曲线
    double d_cruise = total_distance - 2 * d_accel;
    
    for (auto& wp : trajectory) {
      double s = wp.path_length;
      
      if (s < d_accel) {
        // 加速阶段
        wp.velocity = std::sqrt(2 * a_max * s);
        wp.acceleration = a_max;
      } else if (s < d_accel + d_cruise) {
        // 匀速阶段
        wp.velocity = v_max;
        wp.acceleration = 0.0;
      } else {
        // 减速阶段
        double s_remaining = total_distance - s;
        wp.velocity = std::sqrt(2 * a_max * s_remaining);
        wp.acceleration = -a_max;
      }
    }
  }
}

} // namespace algorithm
} // namespace straight_line

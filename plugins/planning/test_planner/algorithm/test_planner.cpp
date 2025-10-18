#include "test_planner.hpp"
#include <chrono>
#include <cmath>

namespace test_planner {
namespace algorithm {

// ========== TestPlanner ==========

TestPlanner::TestPlanner(const Config& config)
    : config_(config) {
}

void TestPlanner::setConfig(const Config& config) {
  config_ = config;
}

TestPlanner::Result TestPlanner::plan(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal) {
  
  auto start_time = std::chrono::high_resolution_clock::now();
  
  Result result;
  
  // TODO: 实现您的规划算法
  // 这里是一个简单的示例：生成直线路径
  
  // 计算距离
  double distance = (goal.head<2>() - start.head<2>()).norm();
  
  // 生成路径点
  int num_points = static_cast<int>(distance / config_.step_size) + 2;
  result.path.reserve(num_points);
  
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    
    Waypoint wp;
    wp.position = start + t * (goal - start);
    wp.velocity = config_.max_velocity;
    wp.timestamp = i * config_.step_size / config_.max_velocity;
    
    result.path.push_back(wp);
  }
  
  result.success = true;
  
  // 计算耗时
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  result.computation_time_ms = duration.count() / 1000.0;
  
  return result;
}

void TestPlanner::reset() {
  // TODO: 重置算法状态
}

} // namespace algorithm
} // namespace test_planner


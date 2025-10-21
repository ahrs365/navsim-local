/**
 * @file test_minco_extraction.cpp
 * @brief Test program to verify MINCO trajectory extraction with full dynamics
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

// Mock structures for testing
namespace navsim {
namespace planning {
struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Twist2d {
  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
};
} // namespace planning

namespace plugin {
struct TrajectoryPoint {
  planning::Pose2d pose;
  planning::Twist2d twist;
  double acceleration = 0.0;
  double steering_angle = 0.0;
  double curvature = 0.0;
  double time_from_start = 0.0;
  double path_length = 0.0;
};
} // namespace plugin
} // namespace navsim

void printTrajectoryStatistics(const std::vector<navsim::plugin::TrajectoryPoint>& trajectory) {
  if (trajectory.empty()) {
    std::cout << "Trajectory is empty!" << std::endl;
    return;
  }

  std::cout << "\n========== MINCO Trajectory Statistics ==========" << std::endl;
  std::cout << "Total points: " << trajectory.size() << std::endl;
  std::cout << "Total time: " << trajectory.back().time_from_start << " s" << std::endl;
  std::cout << "Total length: " << trajectory.back().path_length << " m" << std::endl;

  // Find max/min velocities
  double max_vel = -1e10, min_vel = 1e10;
  double max_omega = -1e10, min_omega = 1e10;
  double max_acc = -1e10, min_acc = 1e10;
  double max_curv = -1e10, min_curv = 1e10;

  for (const auto& pt : trajectory) {
    max_vel = std::max(max_vel, pt.twist.vx);
    min_vel = std::min(min_vel, pt.twist.vx);
    max_omega = std::max(max_omega, std::abs(pt.twist.omega));
    min_omega = std::min(min_omega, std::abs(pt.twist.omega));
    max_acc = std::max(max_acc, std::abs(pt.acceleration));
    min_acc = std::min(min_acc, std::abs(pt.acceleration));
    max_curv = std::max(max_curv, std::abs(pt.curvature));
    min_curv = std::min(min_curv, std::abs(pt.curvature));
  }

  std::cout << "\n--- Velocity Statistics ---" << std::endl;
  std::cout << "  Max linear velocity: " << max_vel << " m/s" << std::endl;
  std::cout << "  Min linear velocity: " << min_vel << " m/s" << std::endl;
  std::cout << "  Max angular velocity: " << max_omega << " rad/s" << std::endl;
  std::cout << "  Min angular velocity: " << min_omega << " rad/s" << std::endl;

  std::cout << "\n--- Acceleration Statistics ---" << std::endl;
  std::cout << "  Max acceleration: " << max_acc << " m/s²" << std::endl;
  std::cout << "  Min acceleration: " << min_acc << " m/s²" << std::endl;

  std::cout << "\n--- Curvature Statistics ---" << std::endl;
  std::cout << "  Max curvature: " << max_curv << " 1/m" << std::endl;
  std::cout << "  Min curvature: " << min_curv << " 1/m" << std::endl;

  // Print first 10 points
  std::cout << "\n--- First 10 Points ---" << std::endl;
  std::cout << std::setw(5) << "i" 
            << std::setw(10) << "x" 
            << std::setw(10) << "y" 
            << std::setw(10) << "yaw" 
            << std::setw(10) << "v" 
            << std::setw(10) << "ω" 
            << std::setw(10) << "a" 
            << std::setw(10) << "κ" 
            << std::setw(10) << "t" << std::endl;
  std::cout << std::string(95, '-') << std::endl;

  for (size_t i = 0; i < std::min(size_t(10), trajectory.size()); ++i) {
    const auto& pt = trajectory[i];
    std::cout << std::setw(5) << i
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.pose.x
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.pose.y
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.pose.yaw
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.twist.vx
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.twist.omega
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.acceleration
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.curvature
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.time_from_start
              << std::endl;
  }

  // Print last 10 points
  std::cout << "\n--- Last 10 Points ---" << std::endl;
  std::cout << std::setw(5) << "i" 
            << std::setw(10) << "x" 
            << std::setw(10) << "y" 
            << std::setw(10) << "yaw" 
            << std::setw(10) << "v" 
            << std::setw(10) << "ω" 
            << std::setw(10) << "a" 
            << std::setw(10) << "κ" 
            << std::setw(10) << "t" << std::endl;
  std::cout << std::string(95, '-') << std::endl;

  size_t start_idx = trajectory.size() >= 10 ? trajectory.size() - 10 : 0;
  for (size_t i = start_idx; i < trajectory.size(); ++i) {
    const auto& pt = trajectory[i];
    std::cout << std::setw(5) << i
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.pose.x
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.pose.y
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.pose.yaw
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.twist.vx
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.twist.omega
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.acceleration
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.curvature
              << std::setw(10) << std::fixed << std::setprecision(4) << pt.time_from_start
              << std::endl;
  }

  std::cout << "=================================================" << std::endl;
}

int main() {
  std::cout << "MINCO Trajectory Extraction Test" << std::endl;
  std::cout << "This is a placeholder test program." << std::endl;
  std::cout << "The actual extraction happens in JpsPlannerPlugin::extractMincoTrajectory()" << std::endl;
  std::cout << "\nTo see the real output, run:" << std::endl;
  std::cout << "  ./build/navsim_local_debug --scenario scenarios/map1.json --planner JpsPlanner --perception EsdfBuilder" << std::endl;
  
  return 0;
}


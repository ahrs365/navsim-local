#include "core/algorithm_manager.hpp"
#include "world_tick.pb.h"
#include "plan_update.pb.h"
#include "ego_cmd.pb.h"
#include <iostream>
#include <iomanip>
#include <chrono>

using namespace navsim;

// 创建一个简单的测试场景
proto::WorldTick createTestWorldTick() {
  proto::WorldTick world_tick;
  
  // 设置 tick_id 和时间戳
  world_tick.set_tick_id(1);
  world_tick.set_stamp(1.0);
  
  // 设置自车状态
  auto* ego = world_tick.mutable_ego();
  auto* pose = ego->mutable_pose();
  pose->set_x(0.0);
  pose->set_y(0.0);
  pose->set_yaw(0.0);
  
  auto* twist = ego->mutable_twist();
  twist->set_vx(0.0);
  twist->set_vy(0.0);
  twist->set_omega(0.0);
  
  // 设置目标点
  auto* goal = world_tick.mutable_goal();
  auto* goal_pose = goal->mutable_pose();
  goal_pose->set_x(10.0);
  goal_pose->set_y(10.0);
  goal_pose->set_yaw(0.785);  // 45度

  auto* goal_tol = goal->mutable_tol();
  goal_tol->set_pos(0.5);
  goal_tol->set_yaw(0.1);

  // 添加一些静态障碍物
  auto* static_map = world_tick.mutable_static_map();

  // 添加一个圆形障碍物
  auto* circle = static_map->add_circles();
  circle->set_x(5.0);
  circle->set_y(5.0);
  circle->set_r(1.0);  // 使用 r 而不是 radius

  // 添加一个多边形障碍物（矩形）
  auto* polygon = static_map->add_polygons();
  auto* p1 = polygon->add_points();
  p1->set_x(6.0);
  p1->set_y(2.5);
  auto* p2 = polygon->add_points();
  p2->set_x(8.0);
  p2->set_y(2.5);
  auto* p3 = polygon->add_points();
  p3->set_x(8.0);
  p3->set_y(3.5);
  auto* p4 = polygon->add_points();
  p4->set_x(6.0);
  p4->set_y(3.5);

  // 添加一些动态障碍物
  auto* dyn_obs = world_tick.add_dynamic_obstacles();
  dyn_obs->set_id("obstacle_1");  // id 是 string 类型

  auto* dyn_shape = dyn_obs->mutable_shape();
  auto* dyn_circle = dyn_shape->mutable_circle();
  dyn_circle->set_x(0.0);
  dyn_circle->set_y(0.0);
  dyn_circle->set_r(0.5);

  auto* dyn_pose = dyn_obs->mutable_pose();
  dyn_pose->set_x(3.0);
  dyn_pose->set_y(8.0);
  dyn_pose->set_yaw(1.57);  // 90度

  auto* dyn_twist = dyn_obs->mutable_twist();
  dyn_twist->set_vx(1.0);
  dyn_twist->set_vy(0.0);
  dyn_twist->set_omega(0.0);

  // 设置底盘配置
  auto* chassis = world_tick.mutable_chassis();
  chassis->set_model("differential");
  chassis->set_wheelbase(2.8);
  chassis->set_track_width(2.0);

  auto* limits = chassis->mutable_limits();
  limits->set_v_max(2.0);
  limits->set_a_max(1.0);
  limits->set_omega_max(1.0);

  auto* geometry = chassis->mutable_geometry();
  geometry->set_body_length(4.5);
  geometry->set_body_width(2.0);
  geometry->set_wheel_radius(0.3);
  
  return world_tick;
}

void printPlanUpdate(const proto::PlanUpdate& plan) {
  std::cout << "\n=== Plan Update ===" << std::endl;
  std::cout << "Tick ID: " << plan.tick_id() << std::endl;
  std::cout << "Stamp: " << std::fixed << std::setprecision(3) << plan.stamp() << std::endl;
  std::cout << "Trajectory points: " << plan.trajectory_size() << std::endl;
  
  if (plan.trajectory_size() > 0) {
    std::cout << "\nFirst 5 trajectory points:" << std::endl;
    std::cout << std::setw(5) << "Index"
              << std::setw(10) << "X"
              << std::setw(10) << "Y"
              << std::setw(10) << "Yaw"
              << std::setw(10) << "Time" << std::endl;
    
    for (int i = 0; i < std::min(5, plan.trajectory_size()); ++i) {
      const auto& point = plan.trajectory(i);
      std::cout << std::setw(5) << i
                << std::setw(10) << std::fixed << std::setprecision(2) << point.x()
                << std::setw(10) << std::fixed << std::setprecision(2) << point.y()
                << std::setw(10) << std::fixed << std::setprecision(2) << point.yaw()
                << std::setw(10) << std::fixed << std::setprecision(2) << point.t() << std::endl;
    }
    
    if (plan.trajectory_size() > 5) {
      std::cout << "... (" << (plan.trajectory_size() - 5) << " more points)" << std::endl;
    }
  }
  std::cout << "===================" << std::endl;
}

void printEgoCmd(const proto::EgoCmd& cmd) {
  std::cout << "\n=== Ego Command ===" << std::endl;
  std::cout << "Acceleration: " << std::fixed << std::setprecision(2) << cmd.acceleration() << " m/s^2" << std::endl;
  std::cout << "Steering: " << std::fixed << std::setprecision(2) << cmd.steering() << " rad" << std::endl;
  std::cout << "===================" << std::endl;
}

void testPluginSystem() {
  std::cout << "\n========================================" << std::endl;
  std::cout << "Testing PLUGIN System" << std::endl;
  std::cout << "========================================\n" << std::endl;
  
  // 配置算法管理器使用插件系统
  AlgorithmManager::Config config;
  config.primary_planner = "StraightLinePlanner";  // 使用直线规划器作为主规划器
  config.fallback_planner = "StraightLinePlanner";
  config.verbose_logging = true;
  
  AlgorithmManager manager(config);
  
  std::cout << "Initializing algorithm manager..." << std::endl;
  if (!manager.initialize()) {
    std::cerr << "Failed to initialize algorithm manager!" << std::endl;
    return;
  }
  std::cout << "Initialization successful!\n" << std::endl;
  
  // 创建测试场景
  std::cout << "Creating test scenario..." << std::endl;
  auto world_tick = createTestWorldTick();
  std::cout << "Test scenario created:" << std::endl;
  std::cout << "  - Ego at (0, 0)" << std::endl;
  std::cout << "  - Goal at (10, 10)" << std::endl;
  std::cout << "  - 1 circle obstacle at (5, 5)" << std::endl;
  std::cout << "  - 1 rectangle obstacle at (7, 3)" << std::endl;
  std::cout << "  - 1 dynamic obstacle at (3, 8)" << std::endl;
  std::cout << std::endl;
  
  // 执行规划
  std::cout << "Running planning..." << std::endl;
  proto::PlanUpdate plan;
  proto::EgoCmd cmd;
  auto deadline = std::chrono::milliseconds(100);
  
  auto start_time = std::chrono::steady_clock::now();
  bool success = manager.process(world_tick, deadline, plan, cmd);
  auto end_time = std::chrono::steady_clock::now();
  
  auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  std::cout << "\n=== Planning Result ===" << std::endl;
  std::cout << "Success: " << (success ? "YES" : "NO") << std::endl;
  std::cout << "Computation time: " << std::fixed << std::setprecision(2) << duration << " ms" << std::endl;
  std::cout << "=======================" << std::endl;
  
  if (success) {
    printPlanUpdate(plan);
    printEgoCmd(cmd);
  }
  
  // 打印统计信息
  auto stats = manager.getStatistics();
  std::cout << "\n=== Statistics ===" << std::endl;
  std::cout << "Total processed: " << stats.total_processed << std::endl;
  std::cout << "Successful: " << stats.successful_processed << std::endl;
  std::cout << "Perception failures: " << stats.perception_failures << std::endl;
  std::cout << "Planning failures: " << stats.planning_failures << std::endl;
  std::cout << "Avg computation time: " << std::fixed << std::setprecision(2) 
            << stats.avg_computation_time_ms << " ms" << std::endl;
  std::cout << "Avg perception time: " << std::fixed << std::setprecision(2) 
            << stats.avg_perception_time_ms << " ms" << std::endl;
  std::cout << "Avg planning time: " << std::fixed << std::setprecision(2) 
            << stats.avg_planning_time_ms << " ms" << std::endl;
  std::cout << "===================" << std::endl;
}



int main() {
  std::cout << "\n╔════════════════════════════════════════╗" << std::endl;
  std::cout << "║  NavSim Plugin System End-to-End Test ║" << std::endl;
  std::cout << "╚════════════════════════════════════════╝\n" << std::endl;
  
  // 测试插件系统
  testPluginSystem();

  std::cout << "\n\n╔════════════════════════════════════════╗" << std::endl;
  std::cout << "║         All Tests Completed!           ║" << std::endl;
  std::cout << "╚════════════════════════════════════════╝\n" << std::endl;
  
  return 0;
}


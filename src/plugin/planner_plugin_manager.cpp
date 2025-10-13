#include "plugin/planner_plugin_manager.hpp"
#include <iostream>

namespace navsim {
namespace plugin {

bool PlannerPluginManager::loadPlanners(
    const std::string& primary_planner_name,
    const std::string& fallback_planner_name,
    bool enable_fallback,
    const nlohmann::json& planner_configs) {
  // 保存配置
  primary_planner_name_ = primary_planner_name;
  fallback_planner_name_ = fallback_planner_name;
  enable_fallback_ = enable_fallback;
  
  // 获取注册表
  auto& registry = PlannerPluginRegistry::getInstance();
  
  // 加载主规划器
  primary_planner_ = registry.createPlugin(primary_planner_name);
  if (!primary_planner_) {
    std::cerr << "[PlannerPluginManager] Failed to create primary planner: " 
              << primary_planner_name << std::endl;
    return false;
  }
  std::cout << "[PlannerPluginManager] Loaded primary planner: " 
            << primary_planner_name << std::endl;
  
  // 加载降级规划器（如果启用）
  if (enable_fallback) {
    fallback_planner_ = registry.createPlugin(fallback_planner_name);
    if (!fallback_planner_) {
      std::cerr << "[PlannerPluginManager] Failed to create fallback planner: " 
                << fallback_planner_name << std::endl;
      return false;
    }
    std::cout << "[PlannerPluginManager] Loaded fallback planner: " 
              << fallback_planner_name << std::endl;
  }
  
  return true;
}

bool PlannerPluginManager::initialize() {
  if (initialized_) {
    std::cerr << "[PlannerPluginManager] Already initialized!" << std::endl;
    return false;
  }
  
  // 初始化主规划器
  std::cout << "[PlannerPluginManager] Initializing primary planner: " 
            << primary_planner_name_ << std::endl;
  
  // 这里需要从配置中获取主规划器的参数
  // 暂时使用空配置
  nlohmann::json primary_config;
  if (!primary_planner_->initialize(primary_config)) {
    std::cerr << "[PlannerPluginManager] Failed to initialize primary planner: " 
              << primary_planner_name_ << std::endl;
    return false;
  }
  
  std::cout << "[PlannerPluginManager] Primary planner '" << primary_planner_name_ 
            << "' initialized successfully" << std::endl;
  
  // 初始化降级规划器（如果启用）
  if (enable_fallback_ && fallback_planner_) {
    std::cout << "[PlannerPluginManager] Initializing fallback planner: " 
              << fallback_planner_name_ << std::endl;
    
    nlohmann::json fallback_config;
    if (!fallback_planner_->initialize(fallback_config)) {
      std::cerr << "[PlannerPluginManager] Failed to initialize fallback planner: " 
                << fallback_planner_name_ << std::endl;
      return false;
    }
    
    std::cout << "[PlannerPluginManager] Fallback planner '" << fallback_planner_name_ 
              << "' initialized successfully" << std::endl;
  }
  
  initialized_ = true;
  std::cout << "[PlannerPluginManager] All planners initialized" << std::endl;
  
  return true;
}

bool PlannerPluginManager::plan(
    const planning::PlanningContext& context,
    std::chrono::milliseconds deadline,
    PlanningResult& result) {
  if (!initialized_) {
    std::cerr << "[PlannerPluginManager] Not initialized!" << std::endl;
    return false;
  }
  
  stats_.total_calls++;
  
  // 尝试使用主规划器
  if (tryPlan(primary_planner_, primary_planner_name_, context, deadline, result)) {
    stats_.primary_success++;
    return true;
  }
  
  stats_.primary_failure++;
  
  // 如果主规划器失败且启用了降级机制，尝试使用降级规划器
  if (enable_fallback_ && fallback_planner_) {
    std::cout << "[PlannerPluginManager] Primary planner failed, trying fallback planner" 
              << std::endl;
    
    if (tryPlan(fallback_planner_, fallback_planner_name_, context, deadline, result)) {
      stats_.fallback_success++;
      return true;
    }
    
    stats_.fallback_failure++;
  }
  
  // 所有规划器都失败
  std::cerr << "[PlannerPluginManager] All planners failed!" << std::endl;
  return false;
}

void PlannerPluginManager::reset() {
  if (primary_planner_) {
    primary_planner_->reset();
  }
  if (fallback_planner_) {
    fallback_planner_->reset();
  }
  
  // 重置统计信息
  stats_ = Statistics();
  
  std::cout << "[PlannerPluginManager] All planners reset" << std::endl;
}

nlohmann::json PlannerPluginManager::getStatistics() const {
  nlohmann::json stats;
  stats["total_calls"] = stats_.total_calls;
  stats["primary_success"] = stats_.primary_success;
  stats["primary_failure"] = stats_.primary_failure;
  stats["fallback_success"] = stats_.fallback_success;
  stats["fallback_failure"] = stats_.fallback_failure;
  stats["total_time_ms"] = stats_.total_time_ms;
  
  if (stats_.total_calls > 0) {
    stats["primary_success_rate"] = 
        static_cast<double>(stats_.primary_success) / stats_.total_calls;
    stats["overall_success_rate"] = 
        static_cast<double>(stats_.primary_success + stats_.fallback_success) / stats_.total_calls;
    stats["average_time_ms"] = stats_.total_time_ms / stats_.total_calls;
  }
  
  stats["primary_planner"] = primary_planner_name_;
  stats["fallback_planner"] = fallback_planner_name_;
  stats["fallback_enabled"] = enable_fallback_;
  
  return stats;
}

bool PlannerPluginManager::tryPlan(
    PlannerPluginPtr planner,
    const std::string& planner_name,
    const planning::PlanningContext& context,
    std::chrono::milliseconds deadline,
    PlanningResult& result) {
  // 检查规划器是否可用
  auto [available, reason] = planner->isAvailable(context);
  if (!available) {
    std::cerr << "[PlannerPluginManager] Planner '" << planner_name 
              << "' is not available: " << reason << std::endl;
    result.success = false;
    result.failure_reason = reason;
    result.planner_name = planner_name;
    return false;
  }
  
  // 执行规划
  auto start_time = std::chrono::steady_clock::now();
  
  bool success = planner->plan(context, deadline, result);
  
  auto end_time = std::chrono::steady_clock::now();
  double elapsed_ms = 
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  stats_.total_time_ms += elapsed_ms;
  
  if (success) {
    std::cout << "[PlannerPluginManager] Planner '" << planner_name 
              << "' succeeded in " << elapsed_ms << " ms" << std::endl;
  } else {
    std::cerr << "[PlannerPluginManager] Planner '" << planner_name 
              << "' failed: " << result.failure_reason << std::endl;
  }
  
  return success;
}

} // namespace plugin
} // namespace navsim


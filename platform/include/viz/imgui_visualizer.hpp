#pragma once

#include "viz/visualizer_interface.hpp"
#include <SDL2/SDL.h>
#include <chrono>
#include <map>
#include <string>

// 前向声明 ImGui 类型
struct ImGuiContext;

namespace navsim {
namespace viz {

/**
 * @brief ImGui + SDL2 可视化器
 * 
 * 提供实时 2D 可视化窗口，显示：
 * - 自车状态
 * - 障碍物（静态 + 动态）
 * - 栅格地图
 * - 规划轨迹
 * - 调试信息面板
 */
class ImGuiVisualizer : public IVisualizer {
public:
  struct Config {
    int window_width = 1400;
    int window_height = 900;
    const char* window_title = "NavSim Local Visualizer";
    
    // 视图配置
    double view_range = 30.0;  // 视图范围（米）
    double pixels_per_meter = 20.0;  // 像素/米比例
    
    // 颜色配置
    bool dark_mode = true;
  };

  ImGuiVisualizer();
  explicit ImGuiVisualizer(const Config& config);
  ~ImGuiVisualizer() override;

  // IVisualizer 接口实现
  bool initialize() override;
  void beginFrame() override;
  void setSystemInfo(const SystemInfo& info) override;
  void updateConnectionStatus(const ConnectionStatus& status) override;
  void drawEgo(const planning::EgoVehicle& ego) override;
  void drawGoal(const planning::Pose2d& goal) override;
  void drawBEVObstacles(const planning::BEVObstacles& obstacles) override;
  void drawDynamicObstacles(const std::vector<planning::DynamicObstacle>& obstacles) override;
  void drawOccupancyGrid(const planning::OccupancyGrid& grid) override;
  void drawESDFMap(const planning::ESDFMap& esdf_map);
  void drawTrajectory(const std::vector<plugin::TrajectoryPoint>& trajectory,
                      const std::string& planner_name = "") override;
  void drawDebugPaths(const std::vector<std::vector<planning::Pose2d>>& paths,
                      const std::vector<std::string>& path_names,
                      const std::vector<std::string>& colors) override;
  void updatePlanningContext(const planning::PlanningContext& context) override;
  void updatePlanningResult(const plugin::PlanningResult& result) override;
  void showDebugInfo(const std::string& key, const std::string& value) override;
  void endFrame() override;
  bool shouldClose() const override;
  void shutdown() override;
  bool hasNewGoal(planning::Pose2d& new_goal) override;
  void setGoalSettingMode(bool enable) override;

private:
  Config config_;

  // SDL2 资源（使用 SDL_Renderer，不再使用 OpenGL）
  SDL_Window* window_ = nullptr;
  SDL_Renderer* sdl_renderer_ = nullptr;  // SDL_Renderer 替代 GL Context

  // ImGui 上下文
  ImGuiContext* imgui_context_ = nullptr;
  
  // 状态
  bool initialized_ = false;
  bool should_close_ = false;
  
  // 缓存的数据（用于显示）
  planning::EgoVehicle ego_;
  planning::Pose2d goal_;
  planning::BEVObstacles bev_obstacles_;
  std::vector<planning::DynamicObstacle> dynamic_obstacles_;
  std::unique_ptr<planning::OccupancyGrid> occupancy_grid_;
  std::unique_ptr<planning::ESDFMap> esdf_map_;
  std::vector<plugin::TrajectoryPoint> trajectory_;
  std::string planner_name_;
  std::map<std::string, std::string> debug_info_;

  // Debug paths for multi-stage visualization
  std::vector<std::vector<planning::Pose2d>> debug_paths_;
  std::vector<std::string> debug_path_names_;
  std::vector<std::string> debug_path_colors_;
  std::map<std::string, std::string> context_info_;
  std::map<std::string, std::string> result_info_;
  
  // 视图控制
  struct ViewState {
    double center_x = 0.0;  // 视图中心 X（世界坐标）
    double center_y = 0.0;  // 视图中心 Y（世界坐标）
    double zoom = 1.0;      // 缩放倍数
    bool follow_ego = true; // 是否跟随自车
  } view_state_;

  // 🎨 可视化选项（图例勾选项）
  struct VisualizationOptions {
    bool show_ego = true;              // 显示自车
    bool show_goal = true;             // 显示目标点
    bool show_trajectory = true;       // 显示规划轨迹
    bool show_bev_obstacles = true;    // 显示 BEV 静态障碍物
    bool show_dynamic_obstacles = true;// 显示动态障碍物
    bool show_occupancy_grid = true;   // 显示栅格地图
    bool show_esdf_map = true;         // 显示 ESDF 地图
    bool show_coordinate_axes = true;  // 显示坐标轴
    bool show_grid_lines = true;       // 显示网格线

    // Debug paths for JPS planner
    bool show_debug_paths = true;           // 总开关：显示JPS调试路径
    bool show_raw_jps_path = true;          // 显示原始JPS搜索路径（红色）
    bool show_optimized_path = true;        // 显示优化路径（绿色）
    bool show_sample_trajectory = true;     // 显示采样轨迹（蓝色）
    bool show_minco_trajectory = true;      // 显示MINCO最终轨迹（洋红色）
    bool show_minco_stage1_trajectory = true; // 显示MINCO预处理轨迹（橙色）
    bool show_minco_stage2_trajectory = true; // 显示MINCO主优化轨迹（青色）
  } viz_options_;

  // 系统信息
  SystemInfo system_info_;
  ConnectionStatus connection_status_;

  bool has_world_data_ = false;
  bool has_planning_result_ = false;
  std::chrono::steady_clock::time_point last_world_update_;
  std::string last_result_summary_;

  // 目标点设置相关状态
  bool goal_setting_mode_ = false;        // 是否处于目标点设置模式
  bool has_new_goal_ = false;             // 是否有新的目标点被设置
  planning::Pose2d new_goal_;             // 新设置的目标点
  
  // 内部辅助函数
  void handleEvents();
  void renderScene();
  void renderDebugPanel();
  void renderLegendPanel();  // 🎨 新增：渲染图例面板
  
  // 坐标转换
  struct Point2D { float x, y; };
  Point2D worldToScreen(double world_x, double world_y) const;
  Point2D worldToScreen(const planning::Point2d& point) const;

  void initializeStateDefaults();
  
  // 绘制辅助函数
  void drawCircle(const Point2D& center, float radius, uint32_t color, bool filled = true);
  void drawRectangle(const Point2D& center, float width, float height, 
                     float rotation, uint32_t color, bool filled = true);
  void drawPolygon(const std::vector<Point2D>& points, uint32_t color, bool filled = true);
  void drawLine(const Point2D& p1, const Point2D& p2, uint32_t color, float thickness = 1.0f);
  void drawArrow(const Point2D& start, const Point2D& end, uint32_t color, float thickness = 2.0f);
  
  // 颜色定义
  static constexpr uint32_t COLOR_EGO = 0xFF00FF00;        // 绿色
  static constexpr uint32_t COLOR_GOAL = 0xFFFF0000;       // 红色
  static constexpr uint32_t COLOR_OBSTACLE = 0xFF0000FF;   // 蓝色
  static constexpr uint32_t COLOR_DYNAMIC = 0xFFFF00FF;    // 紫色
  static constexpr uint32_t COLOR_TRAJECTORY = 0xFF00FFFF; // 青色
  static constexpr uint32_t COLOR_GRID_OCC = 0xFF404040;   // 深灰色
  static constexpr uint32_t COLOR_GRID_FREE = 0xFFF0F0F0;  // 浅灰色

  // 工具函数
  static std::string formatBool(bool value);
  static std::string formatDouble(double value, int precision = 2);
};

} // namespace viz
} // namespace navsim

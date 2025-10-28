#pragma once

#include "viz/visualizer_interface.hpp"
#include <SDL2/SDL.h>
#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <imgui.h>

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
    int window_width = 1800;   // 窗口宽度（合理尺寸）
    int window_height = 1400;  // 窗口高度（增加到 1400 以容纳完整的 Planning Result Plots）
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
  void drawTrajectoryTracking(const planning::Pose2d& actual_pose,
                              const planning::Pose2d& target_pose,
                              const plugin::TrajectoryPoint& current_target,
                              double position_error,
                              double heading_error) override;
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

  // 🔧 场景加载功能
  /**
   * @brief 设置场景加载回调
   * @param callback 场景加载回调函数，参数为场景文件路径
   */
  void setScenarioLoadCallback(std::function<void(const std::string&)> callback);

  /**
   * @brief 检查是否有新的场景加载请求
   * @param scenario_path 输出参数：场景文件路径
   * @return 是否有新的场景加载请求
   */
  bool hasScenarioLoadRequest(std::string& scenario_path);

  /**
   * @brief 添加日志到按钮日志显示区域
   * @param log 日志内容
   */
  void addLog(const std::string& log);

  // 🎮 仿真控制功能
  /**
   * @brief 设置仿真控制回调
   * @param start_callback 开始仿真回调
   * @param pause_callback 暂停仿真回调
   * @param reset_callback 重置仿真回调
   */
  void setSimulationControlCallbacks(
    std::function<void()> start_callback,
    std::function<void()> pause_callback,
    std::function<void()> reset_callback);

  /**
   * @brief 更新仿真状态（用于按钮显示）
   * @param is_paused 是否暂停
   */
  void updateSimulationStatus(bool is_paused);

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

  // 轨迹跟踪数据
  struct TrackingData {
    planning::Pose2d actual_pose;
    planning::Pose2d target_pose;
    plugin::TrajectoryPoint current_target;
    double position_error = 0.0;
    double heading_error = 0.0;
    bool has_tracking_data = false;
  } tracking_data_;
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

  struct PanState {
    bool active = false;
    float last_mouse_x = 0.0f;
    float last_mouse_y = 0.0f;
  } pan_state_;

  ImVec2 scene_canvas_pos_{0.0f, 0.0f};
  ImVec2 scene_canvas_size_{0.0f, 0.0f};

  // 鼠标滑轮缩放
  int wheel_delta_ = 0;  // 滑轮增量累积

  // 🎨 可视化选项（图例勾选项）
  struct VisualizationOptions {
    bool show_ego = true;              // 显示自车
    bool show_goal = true;             // 显示目标点
    bool show_trajectory = true;       // 显示主轨迹（青色粗线）
    bool show_bev_obstacles = true;    // 显示 BEV 静态障碍物
    bool show_dynamic_obstacles = true;// 显示动态障碍物
    bool show_occupancy_grid = true;   // 显示栅格地图
    bool show_esdf_map = true;         // 显示 ESDF 地图
    bool show_coordinate_axes = true;  // 显示坐标轴
    bool show_grid_lines = false;      // 显示网格线（默认关闭，避免视觉干扰）

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
  plugin::PlanningResult latest_planning_result_;  // 存储最新的规划结果用于绘图
  std::chrono::steady_clock::time_point last_world_update_;
  std::string last_result_summary_;

  // 📊 历史数据存储（用于 v-t 和 omega-t 图）
  std::vector<float> history_time_;      // 历史时间戳（仿真时间）
  std::vector<float> history_velocity_;  // 历史线速度
  std::vector<float> history_omega_;     // 历史角速度
  void clearHistoryData();               // 清空历史数据（Reset 时调用）

  // 轨迹跟踪可视化状态
  bool tracking_has_prev_yaw_ = false;
  double tracking_prev_yaw_ = 0.0;

  // 目标点设置相关状态
  bool goal_setting_mode_ = false;        // 是否处于目标点设置模式
  bool has_new_goal_ = false;             // 是否有新的目标点被设置
  planning::Pose2d new_goal_;             // 新设置的目标点

  // 🔧 场景加载相关状态
  std::function<void(const std::string&)> scenario_load_callback_;  // 场景加载回调
  bool has_scenario_load_request_ = false;  // 是否有场景加载请求
  std::string scenario_path_request_;       // 请求加载的场景路径
  char scenario_path_input_[256] = "";      // 场景路径输入框缓冲区

  // 🎮 仿真控制相关状态
  std::function<void()> sim_start_callback_;   // 开始仿真回调
  std::function<void()> sim_pause_callback_;   // 暂停仿真回调
  std::function<void()> sim_reset_callback_;   // 重置仿真回调
  bool simulation_is_paused_ = true;           // 仿真是否暂停（默认暂停）
  std::vector<std::string> button_logs_;       // 按钮点击日志（最多保留10条）
  void addButtonLog(const std::string& log);   // 添加按钮日志

  // 🎨 面板显示状态控制
  bool show_legend_panel_ = false;             // Legend & Visualization Options 面板默认隐藏
  bool show_plot_panel_ = true;                // 规划结果曲线图面板默认显示

  // 内部辅助函数
  void handleEvents();
  void renderScene();
  void renderDebugPanel();
  void renderLegendPanel();     // 🎨 渲染图例面板
  void renderLoadingScreen();   // 🎨 渲染加载画面
  void renderPlotPanel();       // 🎨 新增：渲染规划结果曲线图面板
  
  // 坐标转换
  struct Point2D { float x, y; };
  Point2D worldToScreen(double world_x, double world_y) const;
  Point2D worldToScreen(const planning::Point2d& point) const;
  Point2D screenToWorld(float screen_x, float screen_y) const;

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
  static double normalizeAngle(double angle);
};

} // namespace viz
} // namespace navsim

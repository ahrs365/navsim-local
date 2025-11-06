#include "viz/imgui_visualizer.hpp"
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>  // 使用 SDL_Renderer 后端
#include <implot.h>  // 添加 ImPlot 支持
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace navsim {
namespace viz {

ImGuiVisualizer::ImGuiVisualizer() : config_(Config{}) {
  initializeStateDefaults();
}

ImGuiVisualizer::ImGuiVisualizer(const Config& config) : config_(config) {
  initializeStateDefaults();
}

ImGuiVisualizer::~ImGuiVisualizer() {
  shutdown();
}

bool ImGuiVisualizer::initialize() {
  if (initialized_) {
    return true;
  }

  // 初始化 SDL2
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0) {
    std::cerr << "[ImGuiVisualizer] SDL_Init Error: " << SDL_GetError() << std::endl;
    return false;
  }

  // 创建窗口（不再使用 OpenGL 标志）
  window_ = SDL_CreateWindow(
    config_.window_title,
    SDL_WINDOWPOS_CENTERED,
    SDL_WINDOWPOS_CENTERED,
    config_.window_width,
    config_.window_height,
    SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI  // 移除 SDL_WINDOW_OPENGL
  );

  if (!window_) {
    std::cerr << "[ImGuiVisualizer] SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
    SDL_Quit();
    return false;
  }

  // 列出所有可用的渲染器
  int num_drivers = SDL_GetNumRenderDrivers();
  for (int i = 0; i < num_drivers; ++i) {
    SDL_RendererInfo info;
    SDL_GetRenderDriverInfo(i, &info);
  }

  // 创建 SDL_Renderer（优先使用软件渲染器）
  sdl_renderer_ = SDL_CreateRenderer(
    window_,
    -1,
    SDL_RENDERER_SOFTWARE  // 强制使用软件渲染器
  );

  if (!sdl_renderer_) {
    std::cerr << "[ImGuiVisualizer] SDL_CreateRenderer (SOFTWARE) Error: " << SDL_GetError() << std::endl;
    std::cerr << "[ImGuiVisualizer] Trying ACCELERATED renderer..." << std::endl;

    // 如果软件渲染器失败，尝试硬件加速
    sdl_renderer_ = SDL_CreateRenderer(
      window_,
      -1,
      SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );

    if (!sdl_renderer_) {
      std::cerr << "[ImGuiVisualizer] SDL_CreateRenderer (ACCELERATED) Error: " << SDL_GetError() << std::endl;
      SDL_DestroyWindow(window_);
      SDL_Quit();
      return false;
    }
  }

  // 初始化 ImGui
  IMGUI_CHECKVERSION();
  imgui_context_ = ImGui::CreateContext();
  ImGui::SetCurrentContext(imgui_context_);

  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  // 设置样式
  if (config_.dark_mode) {
    ImGui::StyleColorsDark();
  } else {
    ImGui::StyleColorsLight();
  }

  // 初始化 ImGui 后端（使用 SDL_Renderer）
  ImGui_ImplSDL2_InitForSDLRenderer(window_, sdl_renderer_);
  ImGui_ImplSDLRenderer2_Init(sdl_renderer_);

  // 初始化 ImPlot
  ImPlot::CreateContext();

  initialized_ = true;

  // 获取渲染器信息
  SDL_RendererInfo renderer_info;
  SDL_GetRendererInfo(sdl_renderer_, &renderer_info);

  return true;
}

void ImGuiVisualizer::initializeStateDefaults() {
  system_info_.general["Visualizer"] = "ImGui SDL2 + SDL_Renderer";
  connection_status_.connected = false;
  connection_status_.message = "Waiting for bridge";

  context_info_.clear();
  context_info_["Status"] = "Waiting for PlanningContext";

  result_info_.clear();
  result_info_["Status"] = "Waiting for PlanningResult";

  debug_info_.clear();
  debug_info_["Status"] = "Idle";

  has_world_data_ = false;
  has_planning_result_ = false;
  last_result_summary_.clear();
}

void ImGuiVisualizer::setSystemInfo(const SystemInfo& info) {
  system_info_ = info;
}

void ImGuiVisualizer::updateConnectionStatus(const ConnectionStatus& status) {
  connection_status_ = status;
  debug_info_["Connection"] = status.connected ? "Connected" : "Disconnected";
  if (!status.message.empty()) {
    debug_info_["Connection Detail"] = status.message;
  } else {
    debug_info_.erase("Connection Detail");
  }
}

void ImGuiVisualizer::beginFrame() {
  if (!initialized_) return;

  // 处理事件
  handleEvents();

  // 开始新的 ImGui 帧 - SDL_Renderer 顺序
  ImGui_ImplSDLRenderer2_NewFrame();  // 1. 先 SDL_Renderer 后端
  ImGui_ImplSDL2_NewFrame();          // 2. 再 SDL2 后端
  ImGui::NewFrame();                   // 3. 最后 ImGui 核心

}

void ImGuiVisualizer::handleEvents() {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    // 让 ImGui 先处理事件
    ImGui_ImplSDL2_ProcessEvent(&event);

    if (event.type == SDL_QUIT) {
      should_close_ = true;
    }

    if (event.type == SDL_WINDOWEVENT &&
        event.window.event == SDL_WINDOWEVENT_CLOSE &&
        event.window.windowID == SDL_GetWindowID(window_)) {
      should_close_ = true;
    }

    // 不再处理键盘快捷键，完全交给 ImGui 处理
    // 这样可以确保输入框能正常工作

    // 🔍 鼠标滑轮缩放功能（在Scene View窗口内时生效）
    if (event.type == SDL_MOUSEWHEEL) {
      // 获取鼠标位置
      int mouse_x, mouse_y;
      SDL_GetMouseState(&mouse_x, &mouse_y);

      // 这里先记录滑轮事件，具体的缩放逻辑在renderScene中处理
      // 因为需要判断鼠标是否在Scene View窗口内
      wheel_delta_ += event.wheel.y;  // 累积滑轮增量
    }

    // 鼠标点击事件将在renderScene中处理，以便获取正确的画布坐标
  }
}

void ImGuiVisualizer::drawEgo(const planning::EgoVehicle& ego) {
  // static int call_count = 0;
  // if (call_count++ % 60 == 0) {
  //   std::cout << "[Viz] drawEgo called: pos=(" << ego.pose.x << ", " << ego.pose.y
  //             << "), yaw=" << ego.pose.yaw << std::endl;
  // }
  ego_ = ego;
  has_world_data_ = true;
  last_world_update_ = std::chrono::steady_clock::now();
  debug_info_["Ego Pose"] = "x=" + formatDouble(ego.pose.x) +
                            ", y=" + formatDouble(ego.pose.y) +
                            ", yaw=" + formatDouble(ego.pose.yaw, 3);
  const double forward_speed = ego.twist.vx;
  const double lateral_speed = ego.twist.vy;
  const double speed_mag = std::hypot(forward_speed, lateral_speed);
  debug_info_["Ego Speed"] = formatDouble(forward_speed) + " m/s";
  debug_info_["|Speed|"] = formatDouble(speed_mag) + " m/s";

  // 📊 更新历史数据（用于 v-t 和 omega-t 图）
  // 从 debug_info_ 中获取仿真时间
  auto sim_time_it = debug_info_.find("Simulation Time");
  if (sim_time_it != debug_info_.end()) {
    try {
      float sim_time = std::stof(sim_time_it->second);
      float velocity = static_cast<float>(forward_speed);
      float omega = ego.twist.omega;

      // 追加到历史数据（限制最大长度为 10000 个点，避免内存溢出）
      if (history_time_.size() < 10000) {
        history_time_.push_back(sim_time);
        history_velocity_.push_back(velocity);
        history_omega_.push_back(omega);
      }
    } catch (...) {
      // 忽略解析错误
    }
  }

  // 更新视图中心（如果跟随自车）
  if (view_state_.follow_ego) {
    view_state_.center_x = ego.pose.x;
    view_state_.center_y = ego.pose.y;
  }
}

void ImGuiVisualizer::clearHistoryData() {
  history_time_.clear();
  history_velocity_.clear();
  history_omega_.clear();
}

void ImGuiVisualizer::clearAllVisualizationData() {
  // 清空历史数据
  clearHistoryData();

  // 清空轨迹
  trajectory_.clear();

  // 清空调试路径
  debug_paths_.clear();
  debug_path_names_.clear();
  debug_path_colors_.clear();

  // 清空近似圆
  approximation_circles_.clear();

  // 清空障碍物
  bev_obstacles_.circles.clear();
  bev_obstacles_.rectangles.clear();
  bev_obstacles_.polygons.clear();
  dynamic_obstacles_.clear();

  // 清空规划结果信息
  has_planning_result_ = false;
  last_result_summary_.clear();
  result_info_.clear();
  result_info_["Status"] = "Waiting for PlanningResult";

  // 清空上下文信息
  context_info_.clear();
  context_info_["Status"] = "Waiting for PlanningContext";
}

void ImGuiVisualizer::drawGoal(const planning::Pose2d& goal) {
  goal_ = goal;
  debug_info_["Goal"] = "x=" + formatDouble(goal.x) +
                        ", y=" + formatDouble(goal.y);
}

void ImGuiVisualizer::drawBEVObstacles(const planning::BEVObstacles& obstacles) {
  // 🔍 调试日志：检查传入的障碍物数据
  // std::cout << "[ImGuiVisualizer] drawBEVObstacles called:" << std::endl;
  // std::cout << "[ImGuiVisualizer]   Input circles: " << obstacles.circles.size() << std::endl;
  // std::cout << "[ImGuiVisualizer]   Input rectangles: " << obstacles.rectangles.size() << std::endl;
  // std::cout << "[ImGuiVisualizer]   Input polygons: " << obstacles.polygons.size() << std::endl;

  // 缓存障碍物数据，在 renderScene() 中绘制
  bev_obstacles_ = obstacles;

  // 🔍 调试日志：检查缓存后的数据
  // std::cout << "[ImGuiVisualizer]   Cached circles: " << bev_obstacles_.circles.size() << std::endl;
  // std::cout << "[ImGuiVisualizer]   Cached rectangles: " << bev_obstacles_.rectangles.size() << std::endl;
  // std::cout << "[ImGuiVisualizer]   Cached polygons: " << bev_obstacles_.polygons.size() << std::endl;

  debug_info_["BEV Circles"] = std::to_string(obstacles.circles.size());
  debug_info_["BEV Rectangles"] = std::to_string(obstacles.rectangles.size());
  debug_info_["BEV Polygons"] = std::to_string(obstacles.polygons.size());
}

void ImGuiVisualizer::drawDynamicObstacles(const std::vector<planning::DynamicObstacle>& obstacles) {
  dynamic_obstacles_ = obstacles;
  debug_info_["Dynamic Obstacles"] = std::to_string(obstacles.size());
}

void ImGuiVisualizer::drawOccupancyGrid(const planning::OccupancyGrid& grid) {
  occupancy_grid_ = std::make_unique<planning::OccupancyGrid>(grid);
  debug_info_["Grid Size"] = std::to_string(grid.config.width) + "x" + std::to_string(grid.config.height);
  debug_info_["Grid Resolution"] = std::to_string(grid.config.resolution) + "m";
}

void ImGuiVisualizer::drawESDFMap(const planning::ESDFMap& esdf_map) {
  esdf_map_ = std::make_unique<planning::ESDFMap>(esdf_map);
  debug_info_["ESDF Size"] = std::to_string(esdf_map.config.width) + "x" + std::to_string(esdf_map.config.height);
  debug_info_["ESDF Resolution"] = std::to_string(esdf_map.config.resolution) + "m";
  debug_info_["ESDF Max Distance"] = std::to_string(esdf_map.config.max_distance) + "m";
}

void ImGuiVisualizer::drawTrajectory(const std::vector<plugin::TrajectoryPoint>& trajectory,
                                      const std::string& planner_name) {
  // static int call_count = 0;
  // if (call_count++ % 60 == 0) {
  //   std::cout << "[Viz] drawTrajectory called: " << trajectory.size() << " points, planner=" << planner_name << std::endl;
  //   if (!trajectory.empty()) {
  //     std::cout << "[Viz]   First point: (" << trajectory[0].pose.x << ", " << trajectory[0].pose.y << ")" << std::endl;
  //   }
  // }
  trajectory_ = trajectory;
  planner_name_ = planner_name;
  has_planning_result_ = true;
  debug_info_["Trajectory Points"] = std::to_string(trajectory.size());
  debug_info_["Planner"] = planner_name;
}

void ImGuiVisualizer::drawTrajectoryTracking(const planning::Pose2d& actual_pose,
                                              const planning::Pose2d& target_pose,
                                              const plugin::TrajectoryPoint& current_target,
                                              double position_error,
                                              double heading_error) {
  // 存储轨迹跟踪状态用于渲染
  tracking_data_.actual_pose = actual_pose;
  tracking_data_.target_pose = target_pose;
  tracking_data_.current_target = current_target;
  tracking_data_.position_error = position_error;
  tracking_data_.heading_error = heading_error;
  tracking_data_.has_tracking_data = true;

  // 更新调试信息
  debug_info_["🎯 Actual Pos"] = "(" + formatDouble(actual_pose.x, 2) + ", " + formatDouble(actual_pose.y, 2) + ")";
  debug_info_["🔻 Target Pos"] = "(" + formatDouble(target_pose.x, 2) + ", " + formatDouble(target_pose.y, 2) + ")";
  debug_info_["📏 Position Error"] = formatDouble(position_error * 1000, 1) + " mm";
  debug_info_["🧭 Heading Error"] = formatDouble(heading_error * 180.0 / M_PI, 1) + " deg";
  debug_info_["⚡ Target Speed"] = formatDouble(current_target.twist.vx, 2) + " m/s";
}

void ImGuiVisualizer::drawDebugPaths(const std::vector<std::vector<planning::Pose2d>>& paths,
                                      const std::vector<std::string>& path_names,
                                      const std::vector<std::string>& colors) {
  debug_paths_ = paths;
  debug_path_names_ = path_names;
  debug_path_colors_ = colors;

  // static int call_count = 0;
  // if (call_count++ % 60 == 0) {
  //   std::cout << "[Viz] drawDebugPaths called with " << paths.size() << " paths" << std::endl;
  //   for (size_t i = 0; i < paths.size(); ++i) {
  //     std::cout << "[Viz]   Path " << i << " (" << (i < path_names.size() ? path_names[i] : "Unknown")
  //               << "): " << paths[i].size() << " points" << std::endl;
  //   }
  // }
}

void ImGuiVisualizer::drawApproximationCircles(const std::vector<std::tuple<double, double, double>>& circles) {
  approximation_circles_ = circles;
}

void ImGuiVisualizer::updatePlanningContext(const planning::PlanningContext& context) {
  context_info_.clear();
  context_info_["Timestamp"] = formatDouble(context.timestamp, 3) + " s";
  context_info_["Planning Horizon"] = formatDouble(context.planning_horizon) + " s";
  context_info_["Goal Pose"] = "x=" + formatDouble(context.task.goal_pose.x) +
                               ", y=" + formatDouble(context.task.goal_pose.y) +
                               ", yaw=" + formatDouble(context.task.goal_pose.yaw, 3);
  context_info_["Dynamic Obstacles"] = std::to_string(context.dynamic_obstacles.size());

  auto taskTypeToString = [](planning::PlanningTask::Type type) {
    switch (type) {
      case planning::PlanningTask::Type::GOTO_GOAL: return "Go To Goal";
      case planning::PlanningTask::Type::LANE_FOLLOWING: return "Lane Following";
      case planning::PlanningTask::Type::LANE_CHANGE: return "Lane Change";
      case planning::PlanningTask::Type::PARKING: return "Parking";
      case planning::PlanningTask::Type::EMERGENCY_STOP: return "Emergency Stop";
      default: return "Unknown";
    }
  };
  context_info_["Task Type"] = taskTypeToString(context.task.type);

  if (context.occupancy_grid) {
    const auto& grid = *context.occupancy_grid;
    context_info_["Occupancy Grid"] = std::to_string(grid.config.width) + "x" +
                                      std::to_string(grid.config.height) +
                                      " @" + formatDouble(grid.config.resolution, 2) + "m";
  } else {
    context_info_["Occupancy Grid"] = "None";
  }

  if (context.esdf_map) {
    const auto& esdf = *context.esdf_map;
    context_info_["ESDF Map"] = std::to_string(esdf.config.width) + "x" +
                                std::to_string(esdf.config.height) +
                                " @" + formatDouble(esdf.config.resolution, 2) + "m" +
                                " (max=" + formatDouble(esdf.config.max_distance, 1) + "m)";
    drawESDFMap(esdf);
  } else {
    context_info_["ESDF Map"] = "None";
  }

  if (context.bev_obstacles) {
    const auto& bev = *context.bev_obstacles;
    context_info_["BEV Obstacles"] = "circles=" + std::to_string(bev.circles.size()) +
                                     ", rectangles=" + std::to_string(bev.rectangles.size()) +
                                     ", polygons=" + std::to_string(bev.polygons.size());
  } else {
    context_info_["BEV Obstacles"] = "None";
  }

  context_info_["Follow Ego"] = formatBool(view_state_.follow_ego);
}

void ImGuiVisualizer::updatePlanningResult(const plugin::PlanningResult& result) {
  result_info_.clear();
  has_planning_result_ = true;
  latest_planning_result_ = result;  // 存储完整的规划结果用于绘图

  result_info_["Planner"] = result.planner_name.empty() ? "Unknown" : result.planner_name;
  result_info_["Status"] = result.success ? "Success" : "Failure";
  if (!result.success && !result.failure_reason.empty()) {
    result_info_["Failure Reason"] = result.failure_reason;
  }

  result_info_["Trajectory Points"] = std::to_string(result.trajectory.size());
  result_info_["Total Time"] = formatDouble(result.getTotalTime()) + " s";
  result_info_["Total Length"] = formatDouble(result.getTotalLength()) + " m";
  result_info_["Computation Time"] = formatDouble(result.computation_time_ms) + " ms";
  result_info_["Iterations"] = std::to_string(result.iterations);

  result_info_["Constraints"] = result.constraints_satisfied ? "Satisfied" : "Violated";
  if (!result.constraint_violations.empty()) {
    std::ostringstream oss;
    for (auto it = result.constraint_violations.begin(); it != result.constraint_violations.end(); ++it) {
      if (it != result.constraint_violations.begin()) {
        oss << ", ";
      }
      oss << it->first << "=" << formatDouble(it->second, 3);
    }
    result_info_["Violations"] = oss.str();
  }

  // Check for optimization status in metadata
  if (!result.metadata.empty()) {
    auto opt_success_it = result.metadata.find("optimization_success");
    if (opt_success_it != result.metadata.end()) {
      bool opt_success = (opt_success_it->second > 0.5);
      result_info_["Optimization"] = opt_success ? "✓ Success" : "✗ Failed";

      // If optimization failed but planning succeeded, it means we're using JPS path only
      if (!opt_success && result.success) {
        result_info_["Note"] = "Using JPS path (optimization failed)";
      }
    }

    // Display other metadata
    std::ostringstream oss;
    for (auto it = result.metadata.begin(); it != result.metadata.end(); ++it) {
      // Skip optimization_success as we've already displayed it
      if (it->first == "optimization_success") continue;

      if (oss.tellp() > 0) {
        oss << ", ";
      }
      oss << it->first << "=" << formatDouble(it->second, 3);
    }
    if (oss.tellp() > 0) {
      result_info_["Metadata"] = oss.str();
    }
  }

  last_result_summary_ = result_info_["Status"];
}

void ImGuiVisualizer::showDebugInfo(const std::string& key, const std::string& value) {
  debug_info_[key] = value;
}

void ImGuiVisualizer::endFrame() {
  if (!initialized_) return;

  // 渲染场景
  renderScene();

  // 渲染调试面板
  renderDebugPanel();

  // 🎨 渲染图例面板
  renderLegendPanel();

  // 🎨 渲染规划结果曲线图面板
  renderPlotPanel();

  // 渲染 ImGui - SDL_Renderer 流程
  // ✅ 正确的渲染顺序：渲染 ImGui -> 呈现（不清屏，让 ImGui 自己管理背景）

  // 1. 渲染 ImGui 绘制数据
  ImGui::Render();
  ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), sdl_renderer_);

  // 2. 呈现到屏幕
  SDL_RenderPresent(sdl_renderer_);
}

void ImGuiVisualizer::renderScene() {
  static int render_count = 0;
  static auto first_render_time = std::chrono::steady_clock::now();
  render_count++;

  // 创建主场景窗口 - 左侧区域
  // 位置：(0, 0)，尺寸：(1190, 850) - 与右侧 Debug Info 面板高度一致，留出 10px 间距
  ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(1190, 850), ImGuiCond_Always);

  // 🕐 在窗口标题显示仿真时间
  std::string window_title = "Scene View";
  auto it = debug_info_.find("Simulation Time");
  if (it != debug_info_.end()) {
    window_title = "Scene View - Sim Time: " + it->second;
  }

  ImGui::Begin(window_title.c_str(), nullptr, ImGuiWindowFlags_NoCollapse);

  // 获取画布位置和大小，并注册一个不可见按钮以捕获鼠标交互
  ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
  ImVec2 canvas_size = ImGui::GetContentRegionAvail();
  scene_canvas_pos_ = canvas_pos;
  scene_canvas_size_ = canvas_size;

  ImGui::InvisibleButton(
    "scene_canvas",
    canvas_size,
    ImGuiButtonFlags_MouseButtonLeft |
    ImGuiButtonFlags_MouseButtonRight |
    ImGuiButtonFlags_MouseButtonMiddle);

  ImVec2 mouse_pos = ImGui::GetMousePos();
  const bool mouse_in_canvas =
    (mouse_pos.x >= canvas_pos.x && mouse_pos.x <= canvas_pos.x + canvas_size.x &&
     mouse_pos.y >= canvas_pos.y && mouse_pos.y <= canvas_pos.y + canvas_size.y);
  const bool scene_hovered =
    ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenBlockedByPopup |
                         ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
  const bool scene_active = ImGui::IsItemActive();

  // 由于 InvisibleButton 会推进光标位置，这里恢复以便后续 worldToScreen 计算
  ImGui::SetCursorScreenPos(canvas_pos);

  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  // static int log_count = 0;
  // if (log_count++ % 60 == 0) {
  //   std::cout << "[Viz]   Canvas pos=(" << canvas_pos.x << ", " << canvas_pos.y
  //             << "), size=(" << canvas_size.x << ", " << canvas_size.y << ")" << std::endl;
  // }

  // 绘制背景
  draw_list->AddRectFilled(canvas_pos,
                           ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                           IM_COL32(20, 20, 20, 255));

  // 🎨 在前10秒或没有数据时，显示明显的加载画面
  auto now = std::chrono::steady_clock::now();
  auto elapsed_since_first_render = std::chrono::duration_cast<std::chrono::seconds>(
    now - first_render_time).count();

  bool show_loading = !has_world_data_ || !has_planning_result_ || elapsed_since_first_render < 2;

  if (show_loading) {
    // 居中显示加载信息
    ImVec2 center(canvas_pos.x + canvas_size.x / 2.0f, canvas_pos.y + canvas_size.y / 2.0f);

    const char* loading_text = "Initializing NavSim Local...";
    ImVec2 text_size = ImGui::CalcTextSize(loading_text);
    draw_list->AddText(
      ImVec2(center.x - text_size.x / 2.0f, center.y - 50.0f),
      IM_COL32(100, 255, 100, 255),  // 绿色
      loading_text
    );

    // 显示状态信息
    std::string status_text;
    if (!has_world_data_) {
      status_text = "Waiting for world data...";
    } else if (!has_planning_result_) {
      status_text = "Running first planning cycle...";
    } else {
      status_text = "Starting simulation...";
    }

    ImVec2 status_size = ImGui::CalcTextSize(status_text.c_str());
    draw_list->AddText(
      ImVec2(center.x - status_size.x / 2.0f, center.y),
      IM_COL32(200, 200, 200, 255),  // 灰色
      status_text.c_str()
    );

    // 绘制旋转的加载指示器
    float angle = render_count * 0.1f;
    for (int i = 0; i < 8; ++i) {
      float a = angle + i * 3.14159f / 4.0f;
      float x = center.x + cos(a) * 30.0f;
      float y = center.y + 80.0f + sin(a) * 30.0f;
      float alpha = 255.0f * (1.0f - i / 8.0f);
      draw_list->AddCircleFilled(ImVec2(x, y), 5.0f, IM_COL32(100, 255, 100, (int)alpha));
    }
  }

  // 如果已经有数据，继续显示小提示
  if (!show_loading && !has_world_data_) {
    draw_list->AddText(ImVec2(canvas_pos.x + 20.0f, canvas_pos.y + 20.0f),
                       IM_COL32(200, 200, 200, 255),
                       "Waiting for world data...");
  } else if (!show_loading && has_world_data_) {
    auto stale_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_world_update_).count();
    if (stale_ms > 1000) {
      std::ostringstream oss;
      oss << "Data stale: " << std::fixed << std::setprecision(1)
          << static_cast<double>(stale_ms) / 1000.0 << " s";
      draw_list->AddText(ImVec2(canvas_pos.x + 20.0f, canvas_pos.y + 20.0f),
                         IM_COL32(255, 200, 0, 255),
                         oss.str().c_str());
    }
  }

  // 🎨 绘制网格（可选）
  if (viz_options_.show_grid_lines) {
    const float grid_step = config_.pixels_per_meter * view_state_.zoom;
    if (grid_step > 10.0f) {  // 只在网格足够大时绘制
      for (float x = fmod(canvas_size.x / 2.0f, grid_step); x < canvas_size.x; x += grid_step) {
        draw_list->AddLine(
          ImVec2(canvas_pos.x + x, canvas_pos.y),
          ImVec2(canvas_pos.x + x, canvas_pos.y + canvas_size.y),
          IM_COL32(40, 40, 40, 255), 1.0f
        );
      }
      for (float y = fmod(canvas_size.y / 2.0f, grid_step); y < canvas_size.y; y += grid_step) {
        draw_list->AddLine(
          ImVec2(canvas_pos.x, canvas_pos.y + y),
          ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + y),
          IM_COL32(40, 40, 40, 255), 1.0f
        );
      }
    }
  }

  // 🎨 绘制坐标轴（可选）
  if (viz_options_.show_coordinate_axes) {
    auto origin = worldToScreen(0, 0);
    auto x_axis = worldToScreen(5, 0);
    auto y_axis = worldToScreen(0, 5);
    draw_list->AddLine(ImVec2(origin.x, origin.y), ImVec2(x_axis.x, x_axis.y),
                       IM_COL32(255, 0, 0, 255), 2.0f);  // X 轴红色
    draw_list->AddLine(ImVec2(origin.x, origin.y), ImVec2(y_axis.x, y_axis.y),
                       IM_COL32(0, 255, 0, 255), 2.0f);  // Y 轴绿色
  }

  // 🎨 0. 绘制栅格地图（可选，在最底层）
  if (viz_options_.show_occupancy_grid && occupancy_grid_) {
    const auto& grid = *occupancy_grid_;
    const auto& cfg = grid.config;

    // 🔧 绘制栅格地图边界框
    double grid_min_x = cfg.origin.x;
    double grid_min_y = cfg.origin.y;
    double grid_max_x = cfg.origin.x + cfg.width * cfg.resolution;
    double grid_max_y = cfg.origin.y + cfg.height * cfg.resolution;

    // 🔍 调试信息：每 60 帧打印一次栅格地图边界信息
    // static int grid_log_count = 0;
    // if (grid_log_count++ % 60 == 0) {
    //   std::cout << "[Viz] Occupancy Grid Boundary:" << std::endl;
    //   std::cout << "  - Grid size: " << cfg.width << "x" << cfg.height << std::endl;
    //   std::cout << "  - Resolution: " << cfg.resolution << " m" << std::endl;
    //   std::cout << "  - Origin: (" << cfg.origin.x << ", " << cfg.origin.y << ")" << std::endl;
    //   std::cout << "  - World bounds: X=[" << grid_min_x << ", " << grid_max_x << "], Y=[" << grid_min_y << ", " << grid_max_y << "]" << std::endl;
    //   std::cout << "  - World size: " << (grid_max_x - grid_min_x) << " x " << (grid_max_y - grid_min_y) << " m" << std::endl;
    //   std::cout << "  - View center: (" << view_state_.center_x << ", " << view_state_.center_y << ")" << std::endl;
    //   std::cout << "  - View zoom: " << view_state_.zoom << std::endl;
    // }

    auto boundary_p1_temp = worldToScreen(grid_min_x, grid_min_y);
    auto boundary_p2_temp = worldToScreen(grid_max_x, grid_min_y);
    auto boundary_p3_temp = worldToScreen(grid_max_x, grid_max_y);
    auto boundary_p4_temp = worldToScreen(grid_min_x, grid_max_y);

    ImVec2 boundary_p1(boundary_p1_temp.x, boundary_p1_temp.y);
    ImVec2 boundary_p2(boundary_p2_temp.x, boundary_p2_temp.y);
    ImVec2 boundary_p3(boundary_p3_temp.x, boundary_p3_temp.y);
    ImVec2 boundary_p4(boundary_p4_temp.x, boundary_p4_temp.y);

    // 🔍 调试信息：打印屏幕坐标
    // if (grid_log_count % 60 == 1) {
    //   std::cout << "  - Screen coords: P1=(" << boundary_p1.x << ", " << boundary_p1.y << "), "
    //             << "P2=(" << boundary_p2.x << ", " << boundary_p2.y << "), "
    //             << "P3=(" << boundary_p3.x << ", " << boundary_p3.y << "), "
    //             << "P4=(" << boundary_p4.x << ", " << boundary_p4.y << ")" << std::endl;
    // }

    // 绘制边界框（亮黄色实线，更粗更明显）
    const float dash_length = 15.0f;  // 🔧 增加虚线长度
    const float gap_length = 8.0f;    // 🔧 增加间隙长度

    // 绘制四条边（使用虚线效果）
    auto drawDashedLine = [&](ImVec2 p1, ImVec2 p2, uint32_t color, float thickness) {
      float dx = p2.x - p1.x;
      float dy = p2.y - p1.y;
      float length = std::sqrt(dx * dx + dy * dy);
      if (length < 0.1f) return;

      float ux = dx / length;
      float uy = dy / length;

      float current = 0.0f;
      while (current < length) {
        float dash_end = std::min(current + dash_length, length);
        ImVec2 start(p1.x + ux * current, p1.y + uy * current);
        ImVec2 end(p1.x + ux * dash_end, p1.y + uy * dash_end);
        draw_list->AddLine(start, end, color, thickness);
        current += dash_length + gap_length;
      }
    };

    // 🔧 使用亮黄色实线，更粗更明显（方便调试）
    drawDashedLine(boundary_p1, boundary_p2, IM_COL32(255, 255, 0, 255), 4.0f);  // 底边（亮黄色）
    drawDashedLine(boundary_p2, boundary_p3, IM_COL32(255, 255, 0, 255), 4.0f);  // 右边（亮黄色）
    drawDashedLine(boundary_p3, boundary_p4, IM_COL32(255, 255, 0, 255), 4.0f);  // 顶边（亮黄色）
    drawDashedLine(boundary_p4, boundary_p1, IM_COL32(255, 255, 0, 255), 4.0f);  // 左边（亮黄色）

    // 只绘制占据的格子（优化性能）
    for (int y = 0; y < cfg.height; ++y) {
      for (int x = 0; x < cfg.width; ++x) {
        int idx = y * cfg.width + x;
        if (idx >= static_cast<int>(grid.data.size())) continue;

        uint8_t value = grid.data[idx];
        if (value < 50) continue;  // 跳过空闲格子（优化性能）

        // 计算格子的世界坐标
        double world_x = cfg.origin.x + x * cfg.resolution;
        double world_y = cfg.origin.y + y * cfg.resolution;

        // 转换到屏幕坐标
        auto p1 = worldToScreen(world_x, world_y);
        auto p2 = worldToScreen(world_x + cfg.resolution, world_y + cfg.resolution);

        // 根据占据概率设置颜色（灰度）
        uint8_t gray = 255 - value;  // 占据越高，颜色越深
        uint32_t color = IM_COL32(gray, gray, gray, 180);

        draw_list->AddRectFilled(
          ImVec2(p1.x, p1.y),
          ImVec2(p2.x, p2.y),
          color
        );
      }
    }
  }

  // 🎨 0.5. 绘制 ESDF 地图（可选，在占据栅格之后）
  // static int esdf_viz_log_count = 0;
  if (viz_options_.show_esdf_map && esdf_map_) {
    const auto& esdf = *esdf_map_;
    const auto& cfg = esdf.config;

    // 调试信息（每 60 帧打印一次）
    // if (esdf_viz_log_count++ % 60 == 0) {
    //   std::cout << "[Viz] Drawing ESDF map: " << cfg.width << "x" << cfg.height
    //             << " @" << cfg.resolution << "m, origin=(" << cfg.origin.x << ", " << cfg.origin.y << ")"
    //             << ", data_size=" << esdf.data.size() << std::endl;
    // }

    // 绘制 ESDF 边界框（青色虚线）
    double esdf_min_x = cfg.origin.x;
    double esdf_min_y = cfg.origin.y;
    double esdf_max_x = cfg.origin.x + cfg.width * cfg.resolution;
    double esdf_max_y = cfg.origin.y + cfg.height * cfg.resolution;

    auto boundary_p1 = worldToScreen(esdf_min_x, esdf_min_y);
    auto boundary_p2 = worldToScreen(esdf_max_x, esdf_min_y);
    auto boundary_p3 = worldToScreen(esdf_max_x, esdf_max_y);
    auto boundary_p4 = worldToScreen(esdf_min_x, esdf_max_y);

    // 绘制边界框（青色虚线）
    auto drawDashedLine = [&](ImVec2 p1, ImVec2 p2, uint32_t color, float thickness) {
      float dx = p2.x - p1.x;
      float dy = p2.y - p1.y;
      float length = std::sqrt(dx * dx + dy * dy);
      if (length < 0.1f) return;

      float ux = dx / length;
      float uy = dy / length;

      float current = 0.0f;
      const float dash_length = 10.0f;
      const float gap_length = 5.0f;
      while (current < length) {
        float dash_end = std::min(current + dash_length, length);
        ImVec2 start(p1.x + ux * current, p1.y + uy * current);
        ImVec2 end(p1.x + ux * dash_end, p1.y + uy * dash_end);
        draw_list->AddLine(start, end, color, thickness);
        current += dash_length + gap_length;
      }
    };

    drawDashedLine(ImVec2(boundary_p1.x, boundary_p1.y), ImVec2(boundary_p2.x, boundary_p2.y), IM_COL32(0, 255, 255, 255), 3.0f);
    drawDashedLine(ImVec2(boundary_p2.x, boundary_p2.y), ImVec2(boundary_p3.x, boundary_p3.y), IM_COL32(0, 255, 255, 255), 3.0f);
    drawDashedLine(ImVec2(boundary_p3.x, boundary_p3.y), ImVec2(boundary_p4.x, boundary_p4.y), IM_COL32(0, 255, 255, 255), 3.0f);
    drawDashedLine(ImVec2(boundary_p4.x, boundary_p4.y), ImVec2(boundary_p1.x, boundary_p1.y), IM_COL32(0, 255, 255, 255), 3.0f);

    // 绘制 ESDF 距离场栅格（使用颜色编码）
    // 采样绘制（每隔几个格子绘制一次，优化性能）
    int sample_step = std::max(1, static_cast<int>(2.0 / view_state_.zoom));  // 根据缩放调整采样率

    for (int y = 0; y < cfg.height; y += sample_step) {
      for (int x = 0; x < cfg.width; x += sample_step) {
        int idx = y * cfg.width + x;
        if (idx >= static_cast<int>(esdf.data.size())) continue;

        double distance = esdf.data[idx];

        // ✅ 可视化时取绝对值（障碍物内部是负值）
        double abs_distance = std::abs(distance);

        // 跳过距离太大的格子（优化性能）
        if (abs_distance >= cfg.max_distance * 0.9) continue;

        // 计算格子的世界坐标（左下角）
        double world_x = cfg.origin.x + x * cfg.resolution;
        double world_y = cfg.origin.y + y * cfg.resolution;

        // 转换到屏幕坐标（左下角和右上角）
        auto p1 = worldToScreen(world_x, world_y);
        auto p2 = worldToScreen(world_x + cfg.resolution * sample_step,
                                world_y + cfg.resolution * sample_step);

        // 7 色渐变方案：
        // 距离 = 0m (障碍物)      -> 深红色 (139, 0, 0)
        // 距离 = 0.5m (很近)      -> 红色 (255, 0, 0)
        // 距离 = 1.0m (近)        -> 橙色 (255, 165, 0)
        // 距离 = 2.0m (中等)      -> 黄色 (255, 255, 0)
        // 距离 = 3.0m (较远)      -> 绿色 (0, 255, 0)
        // 距离 = 4.0m (远)        -> 青色 (0, 255, 255)
        // 距离 >= 5.0m (很远)     -> 蓝色 (0, 0, 255)

        uint8_t r, g, b;
        double normalized_dist = std::clamp(abs_distance / cfg.max_distance, 0.0, 1.0);

        if (normalized_dist < 0.1) {
          // 0.0 - 0.5m: 深红色 -> 红色
          double t = normalized_dist / 0.1;
          r = static_cast<uint8_t>(139 + (255 - 139) * t);
          g = 0;
          b = 0;
        } else if (normalized_dist < 0.2) {
          // 0.5m - 1.0m: 红色 -> 橙色
          double t = (normalized_dist - 0.1) / 0.1;
          r = 255;
          g = static_cast<uint8_t>(165 * t);
          b = 0;
        } else if (normalized_dist < 0.4) {
          // 1.0m - 2.0m: 橙色 -> 黄色
          double t = (normalized_dist - 0.2) / 0.2;
          r = 255;
          g = static_cast<uint8_t>(165 + (255 - 165) * t);
          b = 0;
        } else if (normalized_dist < 0.6) {
          // 2.0m - 3.0m: 黄色 -> 绿色
          double t = (normalized_dist - 0.4) / 0.2;
          r = static_cast<uint8_t>(255 * (1.0 - t));
          g = 255;
          b = 0;
        } else if (normalized_dist < 0.8) {
          // 3.0m - 4.0m: 绿色 -> 青色
          double t = (normalized_dist - 0.6) / 0.2;
          r = 0;
          g = 255;
          b = static_cast<uint8_t>(255 * t);
        } else {
          // 4.0m - 5.0m: 青色 -> 蓝色
          double t = (normalized_dist - 0.8) / 0.2;
          r = 0;
          g = static_cast<uint8_t>(255 * (1.0 - t));
          b = 255;
        }

        uint32_t color = IM_COL32(r, g, b, 150);  // 半透明

        draw_list->AddRectFilled(
          ImVec2(p1.x, p1.y),
          ImVec2(p2.x, p2.y),
          color
        );
      }
    }

    // 鼠标悬停显示 ESDF 距离值
    ImVec2 mouse_pos = ImGui::GetMousePos();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();

    // 检查鼠标是否在画布内
    if (mouse_pos.x >= canvas_pos.x && mouse_pos.x <= canvas_pos.x + canvas_size.x &&
        mouse_pos.y >= canvas_pos.y && mouse_pos.y <= canvas_pos.y + canvas_size.y) {

      // 将鼠标屏幕坐标转换为世界坐标
      float rel_x = mouse_pos.x - (canvas_pos.x + canvas_size.x / 2.0f);
      float rel_y = (canvas_pos.y + canvas_size.y / 2.0f) - mouse_pos.y;  // Y 轴翻转

      double world_x = view_state_.center_x + rel_x / (config_.pixels_per_meter * view_state_.zoom);
      double world_y = view_state_.center_y + rel_y / (config_.pixels_per_meter * view_state_.zoom);

      // 将世界坐标转换为 ESDF 栅格坐标
      int grid_x = static_cast<int>((world_x - cfg.origin.x) / cfg.resolution);
      int grid_y = static_cast<int>((world_y - cfg.origin.y) / cfg.resolution);

      // 检查是否在 ESDF 地图范围内
      if (grid_x >= 0 && grid_x < cfg.width && grid_y >= 0 && grid_y < cfg.height) {
        int idx = grid_y * cfg.width + grid_x;

        if (idx >= 0 && idx < static_cast<int>(esdf.data.size())) {
          double distance = esdf.data[idx];

          // 格式化距离值文本
          // 显示原始值（包括负值），帮助调试
          char dist_text[128];
          if (std::abs(distance) < 0.01) {
            snprintf(dist_text, sizeof(dist_text),
                    "ESDF: OBSTACLE (%.3f m)\nGrid: (%d, %d)\nWorld: (%.2f, %.2f)",
                    distance, grid_x, grid_y, world_x, world_y);
          } else if (distance < 0) {
            snprintf(dist_text, sizeof(dist_text),
                    "ESDF: %.3f m (inside)\nGrid: (%d, %d)\nWorld: (%.2f, %.2f)",
                    distance, grid_x, grid_y, world_x, world_y);
          } else {
            snprintf(dist_text, sizeof(dist_text),
                    "ESDF: %.3f m\nGrid: (%d, %d)\nWorld: (%.2f, %.2f)",
                    distance, grid_x, grid_y, world_x, world_y);
          }

          // 计算文本大小
          ImVec2 text_size = ImGui::CalcTextSize(dist_text);

          // 计算文本位置（鼠标右下方，带偏移）
          ImVec2 text_pos = mouse_pos;
          text_pos.x += 15.0f;  // 向右偏移
          text_pos.y += 15.0f;  // 向下偏移

          // 确保文本不超出画布边界
          if (text_pos.x + text_size.x + 10 > canvas_pos.x + canvas_size.x) {
            text_pos.x = mouse_pos.x - text_size.x - 15.0f;  // 显示在鼠标左侧
          }
          if (text_pos.y + text_size.y + 10 > canvas_pos.y + canvas_size.y) {
            text_pos.y = mouse_pos.y - text_size.y - 15.0f;  // 显示在鼠标上方
          }

          // 绘制背景框
          draw_list->AddRectFilled(
            ImVec2(text_pos.x - 5, text_pos.y - 5),
            ImVec2(text_pos.x + text_size.x + 5, text_pos.y + text_size.y + 5),
            IM_COL32(0, 0, 0, 200)  // 半透明黑色背景
          );

          // 绘制边框
          draw_list->AddRect(
            ImVec2(text_pos.x - 5, text_pos.y - 5),
            ImVec2(text_pos.x + text_size.x + 5, text_pos.y + text_size.y + 5),
            IM_COL32(0, 255, 255, 255),  // 青色边框
            0.0f, 0, 2.0f
          );

          // 绘制文本
          draw_list->AddText(text_pos, IM_COL32(255, 255, 255, 255), dist_text);
        }
      }
    }
  }

  // 🎨 1. 绘制 BEV 障碍物 - 圆形（可选）
  static int obstacle_log_count = 0;
  if (viz_options_.show_bev_obstacles) {
    if (obstacle_log_count++ % 60 == 0) {
      // std::cout << "[Viz]   Drawing " << bev_obstacles_.circles.size() << " BEV circles, "
      //           << bev_obstacles_.rectangles.size() << " rectangles, "
      //           << bev_obstacles_.polygons.size() << " polygons" << std::endl;
      if (!bev_obstacles_.circles.empty()) {
        auto test_center = worldToScreen(bev_obstacles_.circles[0].center);
        // std::cout << "[Viz]     First circle: world=(" << bev_obstacles_.circles[0].center.x
        //           << ", " << bev_obstacles_.circles[0].center.y
        //           << ") -> screen=(" << test_center.x << ", " << test_center.y << ")" << std::endl;
      }
    }

    for (const auto& circle : bev_obstacles_.circles) {
    auto center = worldToScreen(circle.center);
    float radius = circle.radius * config_.pixels_per_meter * view_state_.zoom;
    draw_list->AddCircleFilled(
      ImVec2(center.x, center.y),
      radius,
      IM_COL32(255, 100, 100, 200)  // 红色半透明
    );
    draw_list->AddCircle(
      ImVec2(center.x, center.y),
      radius,
      IM_COL32(255, 0, 0, 255),  // 红色边框
      0, 2.0f
    );
  }

  // 2. 绘制 BEV 障碍物 - 矩形
  // if (obstacle_log_count % 60 == 0 && !bev_obstacles_.rectangles.empty()) {
  //   std::cout << "[Viz]   Drawing " << bev_obstacles_.rectangles.size() << " BEV rectangles" << std::endl;
  //   std::cout << "[Viz]     First rect: world=(" << bev_obstacles_.rectangles[0].pose.x
  //             << ", " << bev_obstacles_.rectangles[0].pose.y
  //             << "), size=(" << bev_obstacles_.rectangles[0].width
  //             << " x " << bev_obstacles_.rectangles[0].height << ")" << std::endl;
  // }

  for (const auto& rect : bev_obstacles_.rectangles) {
    auto center = worldToScreen(rect.pose.x, rect.pose.y);
    float w = rect.width * config_.pixels_per_meter * view_state_.zoom;
    float h = rect.height * config_.pixels_per_meter * view_state_.zoom;

    // if (obstacle_log_count % 60 == 0) {
    //   auto screen_pos = worldToScreen(rect.pose.x, rect.pose.y);
    //   std::cout << "[Viz]       Rect screen pos=(" << screen_pos.x << ", " << screen_pos.y
    //             << "), size=(" << w << " x " << h << "), yaw=" << rect.pose.yaw << std::endl;
    // }

    // 正确绘制带旋转的矩形
    float cos_yaw = std::cos(rect.pose.yaw);
    float sin_yaw = std::sin(rect.pose.yaw);

    // 计算矩形四个顶点相对于中心的偏移
    float half_w = w / 2.0f;
    float half_h = h / 2.0f;

    // 未旋转的四个顶点（相对于中心）
    std::vector<ImVec2> corners = {
      ImVec2(-half_w, -half_h),  // 左下
      ImVec2( half_w, -half_h),  // 右下
      ImVec2( half_w,  half_h),  // 右上
      ImVec2(-half_w,  half_h)   // 左上
    };

    // 应用旋转和平移
    for (auto& corner : corners) {
      float x = corner.x * cos_yaw - corner.y * sin_yaw;
      float y = corner.x * sin_yaw + corner.y * cos_yaw;
      corner.x = center.x + x;
      corner.y = center.y + y;
    }

    // 绘制填充矩形
    draw_list->AddConvexPolyFilled(
      corners.data(),
      corners.size(),
      IM_COL32(100, 255, 100, 200)  // 绿色填充
    );

    // 绘制矩形边框
    draw_list->AddPolyline(
      corners.data(),
      corners.size(),
      IM_COL32(0, 255, 0, 255),  // 绿色边框
      ImDrawFlags_Closed,
      2.0f
    );
  }

  // 3. 绘制 BEV 障碍物 - 多边形
  // if (obstacle_log_count % 60 == 0 && !bev_obstacles_.polygons.empty()) {
  //   std::cout << "[Viz]   Drawing " << bev_obstacles_.polygons.size() << " BEV polygons" << std::endl;
  //   std::cout << "[Viz]     First polygon: " << bev_obstacles_.polygons[0].vertices.size() << " vertices" << std::endl;
  // }

  for (const auto& poly : bev_obstacles_.polygons) {
    if (poly.vertices.empty()) continue;

    // 绘制多边形填充
    std::vector<ImVec2> screen_points;
    for (const auto& vertex : poly.vertices) {
      auto screen_pos = worldToScreen(vertex.x, vertex.y);
      screen_points.push_back(ImVec2(screen_pos.x, screen_pos.y));
    }

    if (screen_points.size() >= 3) {
      draw_list->AddConvexPolyFilled(
        screen_points.data(),
        screen_points.size(),
        IM_COL32(255, 200, 0, 150)  // 黄色半透明填充
      );

      // 绘制多边形边框
      for (size_t i = 0; i < screen_points.size(); ++i) {
        size_t next = (i + 1) % screen_points.size();
        draw_list->AddLine(
          screen_points[i],
          screen_points[next],
          IM_COL32(255, 200, 0, 255),  // 黄色边框
          2.0f
        );
      }
    }
  }
  }  // 🎨 结束 BEV 障碍物绘制

  // 🎨 4. 绘制动态障碍物（可选）
  static int dyn_obs_log_count = 0;
  if (viz_options_.show_dynamic_obstacles) {
    // if (dyn_obs_log_count++ % 60 == 0 && !dynamic_obstacles_.empty()) {
    //   std::cout << "[Viz]   Drawing " << dynamic_obstacles_.size() << " dynamic obstacles" << std::endl;
    // // 🔧 修复问题1：打印所有障碍物的信息
    // for (size_t i = 0; i < dynamic_obstacles_.size(); ++i) {
    //   const auto& obs = dynamic_obstacles_[i];
    //   std::cout << "[Viz]     Dyn obs #" << i << ": shape=" << obs.shape_type
    //             << ", pos=(" << obs.current_pose.x << ", " << obs.current_pose.y
    //             << "), length=" << obs.length << ", width=" << obs.width << std::endl;
    // }
    // }
    dyn_obs_log_count++;  // 保留计数器更新

  for (const auto& dyn_obs : dynamic_obstacles_) {
    auto center = worldToScreen(dyn_obs.current_pose.x, dyn_obs.current_pose.y);

    // 🔧 修复问题4：使用 shape_type 判断，而不是长宽相等
    bool is_circle = (dyn_obs.shape_type == "circle");

    if (is_circle) {
      // 绘制圆形动态障碍物
      float radius = dyn_obs.length / 2.0f * config_.pixels_per_meter * view_state_.zoom;

      // if (dyn_obs_log_count % 60 == 0) {
      //   std::cout << "[Viz]       Dyn obs (circle) radius=" << radius << " pixels (diameter=" << dyn_obs.length << ")" << std::endl;
      // }

      draw_list->AddCircleFilled(
        ImVec2(center.x, center.y),
        radius,
        IM_COL32(255, 0, 255, 200)  // 紫色
      );
      draw_list->AddCircle(
        ImVec2(center.x, center.y),
        radius,
        IM_COL32(255, 0, 255, 255),  // 紫色边框
        0, 2.0f
      );
    } else {
      // 绘制矩形动态障碍物（带旋转）
      float w = dyn_obs.width * config_.pixels_per_meter * view_state_.zoom;
      float h = dyn_obs.length * config_.pixels_per_meter * view_state_.zoom;
      float yaw = dyn_obs.current_pose.yaw;

      // if (dyn_obs_log_count % 60 == 0) {
      //   std::cout << "[Viz]       Dyn obs (rect) visualization:" << std::endl;
      //   std::cout << "[Viz]         dyn_obs.width = " << dyn_obs.width << " m" << std::endl;
      //   std::cout << "[Viz]         dyn_obs.length = " << dyn_obs.length << " m" << std::endl;
      //   std::cout << "[Viz]         dyn_obs.current_pose.yaw = " << yaw << " rad" << std::endl;
      //   std::cout << "[Viz]         Screen size: w=" << w << " px, h=" << h << " px" << std::endl;
      //   std::cout << "[Viz]         Velocity: vx=" << dyn_obs.current_twist.vx
      //             << ", vy=" << dyn_obs.current_twist.vy << std::endl;
      // }

      // 计算矩形的四个顶点（相对于中心）
      float half_w = w / 2.0f;
      float half_h = h / 2.0f;
      float cos_yaw = std::cos(yaw);
      float sin_yaw = std::sin(yaw);

      ImVec2 corners[4];
      float local_corners[4][2] = {
        {-half_w, -half_h},  // 左下
        { half_w, -half_h},  // 右下
        { half_w,  half_h},  // 右上
        {-half_w,  half_h}   // 左上
      };

      for (int i = 0; i < 4; ++i) {
        float lx = local_corners[i][0];
        float ly = local_corners[i][1];
        float rx = lx * cos_yaw - ly * sin_yaw;
        float ry = lx * sin_yaw + ly * cos_yaw;
        corners[i] = ImVec2(center.x + rx, center.y - ry);  // 注意 Y 轴翻转
      }

      // 绘制填充矩形
      draw_list->AddConvexPolyFilled(corners, 4, IM_COL32(255, 0, 255, 200));  // 紫色填充

      // 绘制矩形边框
      for (int i = 0; i < 4; ++i) {
        draw_list->AddLine(corners[i], corners[(i + 1) % 4], IM_COL32(255, 0, 255, 255), 2.0f);
      }

      // 绘制朝向指示（前方中心点）
      float front_x = half_h * cos_yaw;
      float front_y = half_h * sin_yaw;
      draw_list->AddCircleFilled(
        ImVec2(center.x + front_x, center.y - front_y),
        3.0f,
        IM_COL32(255, 255, 0, 255)  // 黄色点表示前方
      );
    }

  }
  }  // 🎨 结束动态障碍物绘制

  // 🎨 4.4 绘制障碍物近似圆（半透明）
  if (!approximation_circles_.empty() && viz_options_.show_bev_obstacles) {
    for (const auto& circle_tuple : approximation_circles_) {
      double x = std::get<0>(circle_tuple);
      double y = std::get<1>(circle_tuple);
      double radius = std::get<2>(circle_tuple);

      auto center = worldToScreen(x, y);
      float screen_radius = radius * config_.pixels_per_meter * view_state_.zoom;

      // 绘制半透明的圆形边界（黄色）
      draw_list->AddCircle(
        ImVec2(center.x, center.y),
        screen_radius,
        IM_COL32(255, 255, 0, 100),  // 黄色，半透明
        64,  // 圆形分段数
        2.0f  // 线宽
      );

      // 绘制半透明的填充圆（黄色）
      draw_list->AddCircleFilled(
        ImVec2(center.x, center.y),
        screen_radius,
        IM_COL32(255, 255, 0, 30)  // 黄色，更透明
      );
    }
  }

  // 🎨 4.5 绘制调试路径（多阶段显示）
  if (!debug_paths_.empty() && viz_options_.show_debug_paths) {
    // 使用 debug_path_colors_ 如果可用，否则使用默认颜色
    std::vector<ImU32> path_colors;
    std::vector<bool> path_dashed;  // Track which paths should be dashed

    if (!debug_path_colors_.empty() && debug_path_colors_.size() == debug_paths_.size()) {
      // Convert color names to ImU32
      for (const auto& color_name : debug_path_colors_) {
        bool is_dashed = false;
        std::string actual_color = color_name;

        // Check for "dashed_" prefix
        if (color_name.substr(0, 7) == "dashed_") {
          is_dashed = true;
          actual_color = color_name.substr(7);  // Remove "dashed_" prefix
        }

        path_dashed.push_back(is_dashed);

        if (actual_color == "red") {
          path_colors.push_back(IM_COL32(255, 100, 100, 255));
        } else if (actual_color == "green") {
          path_colors.push_back(IM_COL32(100, 255, 100, 255));
        } else if (actual_color == "blue") {
          path_colors.push_back(IM_COL32(100, 100, 255, 255));
        } else if (actual_color == "yellow") {
          path_colors.push_back(IM_COL32(255, 255, 0, 255));
        } else if (actual_color == "magenta") {
          path_colors.push_back(IM_COL32(255, 0, 255, 255));
        } else if (actual_color == "cyan") {
          path_colors.push_back(IM_COL32(0, 255, 255, 255));
        } else if (actual_color == "orange") {
          path_colors.push_back(IM_COL32(255, 165, 0, 255));
        } else {
          path_colors.push_back(IM_COL32(255, 255, 255, 255));  // white
        }
      }
    } else {
      // Default colors for JPS planner
      path_colors = {
        IM_COL32(255, 100, 100, 255),  // 红色 - Raw JPS path
        IM_COL32(100, 255, 100, 255),  // 绿色 - Optimized path
        IM_COL32(100, 100, 255, 255),  // 蓝色 - Sample trajectory
        IM_COL32(255, 255, 0, 255),    // 黄色 - MINCO Final (高对比度)
        IM_COL32(255, 0, 255, 255),    // 洋红色 - MINCO Stage1 (高对比度)
        IM_COL32(0, 255, 255, 255)     // 青色 - MINCO Stage2 (高对比度)
      };
    }

    std::vector<bool> path_enabled = {
      viz_options_.show_raw_jps_path,
      viz_options_.show_optimized_path,
      viz_options_.show_sample_trajectory,
      viz_options_.show_minco_trajectory,
      viz_options_.show_minco_stage1_trajectory,
      viz_options_.show_minco_stage2_trajectory
    };

    for (size_t path_idx = 0; path_idx < debug_paths_.size(); ++path_idx) {
      // Check if this path should be displayed
      // For JPS paths (first 6 paths with checkboxes)
      if (path_idx < path_enabled.size() && path_idx < 6 && !path_enabled[path_idx]) {
        continue;  // Skip this path if its checkbox is unchecked
      }

      // For TMPC paths, check path name and corresponding option
      if (path_idx < debug_path_names_.size()) {
        const std::string& path_name = debug_path_names_[path_idx];

        if (path_name == "Reference Path" && !viz_options_.show_tmpc_reference_path) {
          continue;
        } else if (path_name.find("Guidance Path") != std::string::npos && !viz_options_.show_tmpc_guidance_paths) {
          continue;
        } else if (path_name.find("MPC Candidate") != std::string::npos && !viz_options_.show_tmpc_mpc_candidates) {
          continue;
        } else if (path_name.find("Obstacle") != std::string::npos && path_name.find("Prediction") != std::string::npos && !viz_options_.show_tmpc_obstacle_predictions) {
          continue;
        } else if (path_name == "Past Trajectory" && !viz_options_.show_tmpc_past_trajectory) {
          continue;
        }
      }

      const auto& path = debug_paths_[path_idx];
      if (path.size() < 2) continue;

      ImU32 color = path_idx < path_colors.size() ? path_colors[path_idx] : IM_COL32(255, 255, 255, 255);
      bool is_dashed = path_idx < path_dashed.size() ? path_dashed[path_idx] : false;
      float line_width = 3.0f;  // Use consistent line width

      for (size_t i = 1; i < path.size(); ++i) {
        auto p1 = worldToScreen(path[i-1].x, path[i-1].y);
        auto p2 = worldToScreen(path[i].x, path[i].y);

        if (is_dashed) {
          // Draw dashed line
          float dash_length = 10.0f;
          float gap_length = 5.0f;
          float total_length = std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
          float dx = (p2.x - p1.x) / total_length;
          float dy = (p2.y - p1.y) / total_length;

          float current_length = 0.0f;
          bool drawing = true;

          while (current_length < total_length) {
            float segment_length = drawing ? dash_length : gap_length;
            float end_length = std::min(current_length + segment_length, total_length);

            if (drawing) {
              ImVec2 dash_start(p1.x + dx * current_length, p1.y + dy * current_length);
              ImVec2 dash_end(p1.x + dx * end_length, p1.y + dy * end_length);
              draw_list->AddLine(dash_start, dash_end, color, line_width);
            }

            current_length = end_length;
            drawing = !drawing;
          }
        } else {
          // Draw solid line
          draw_list->AddLine(
            ImVec2(p1.x, p1.y),
            ImVec2(p2.x, p2.y),
            color,
            line_width
          );
        }
      }
    }
  }

  // 🎨 5. 绘制规划轨迹（主轨迹 - 绿色高亮加粗）
  if (viz_options_.show_trajectory && trajectory_.size() > 1) {
    static int traj_log_count = 0;
    if (traj_log_count++ % 60 == 0) {
      // std::cout << "[Viz] Drawing trajectory with " << trajectory_.size() << " points" << std::endl;
      // std::cout << "[Viz]   Ego position: (" << ego_.pose.x << ", " << ego_.pose.y << ")" << std::endl;
      // std::cout << "[Viz]   First 5 trajectory points:" << std::endl;
      for (size_t i = 0; i < std::min(size_t(5), trajectory_.size()); ++i) {
        // std::cout << "[Viz]     Point[" << i << "]: ("
        //           << trajectory_[i].pose.x << ", "
        //           << trajectory_[i].pose.y << ")" << std::endl;
      }
      auto test_p1 = worldToScreen(trajectory_[0].pose.x, trajectory_[0].pose.y);
      auto test_p2 = worldToScreen(trajectory_[1].pose.x, trajectory_[1].pose.y);
      // std::cout << "[Viz]   First segment screen coords: (" << test_p1.x << "," << test_p1.y
      //           << ") -> (" << test_p2.x << "," << test_p2.y << ")" << std::endl;
    }

    for (size_t i = 1; i < trajectory_.size(); ++i) {
      auto p1 = worldToScreen(trajectory_[i-1].pose.x, trajectory_[i-1].pose.y);
      auto p2 = worldToScreen(trajectory_[i].pose.x, trajectory_[i].pose.y);
      draw_list->AddLine(
        ImVec2(p1.x, p1.y),
        ImVec2(p2.x, p2.y),
        IM_COL32(0, 255, 0, 255),  // 绿色（高亮）
        5.0f  // 加粗到 5.0f
      );
    }
  }  // 🎨 结束轨迹绘制

  // 🎯 5.5. 绘制轨迹跟踪状态（实际位置、目标位置、误差）
  if (tracking_data_.has_tracking_data) {
    static int tracking_log_count = 0;
    if (tracking_log_count++ % 60 == 0) {
      // std::cout << "[Viz] Drawing trajectory tracking:" << std::endl;
      // std::cout << "[Viz]   Actual pos: (" << tracking_data_.actual_pose.x << ", " << tracking_data_.actual_pose.y << ")" << std::endl;
      // std::cout << "[Viz]   Target pos: (" << tracking_data_.target_pose.x << ", " << tracking_data_.target_pose.y << ")" << std::endl;
      // std::cout << "[Viz]   Position error: " << tracking_data_.position_error * 1000 << " mm" << std::endl;
    }

    auto actual_screen = worldToScreen(tracking_data_.actual_pose.x, tracking_data_.actual_pose.y);

    // 绘制目标位置点（红色圆圈）
    auto target_screen = worldToScreen(tracking_data_.target_pose.x, tracking_data_.target_pose.y);
    draw_list->AddCircle(
      ImVec2(target_screen.x, target_screen.y),
      12.0f,
      IM_COL32(255, 50, 50, 255),  // 红色
      0, 3.0f
    );

    // 绘制目标位置的方向箭头
    double target_yaw = tracking_data_.target_pose.yaw;
    if (tracking_has_prev_yaw_) {
      double delta = normalizeAngle(target_yaw - tracking_prev_yaw_);
      target_yaw = tracking_prev_yaw_ + delta;
    }
    tracking_prev_yaw_ = target_yaw;
    tracking_has_prev_yaw_ = true;

    float arrow_length = 20.0f;
    auto target_arrow_end = worldToScreen(
      tracking_data_.target_pose.x + arrow_length * 0.05f * static_cast<float>(std::cos(target_yaw)),
      tracking_data_.target_pose.y + arrow_length * 0.05f * static_cast<float>(std::sin(target_yaw))
    );
    draw_list->AddLine(
      ImVec2(target_screen.x, target_screen.y),
      ImVec2(target_arrow_end.x, target_arrow_end.y),
      IM_COL32(255, 100, 100, 255),  // 浅红色
      2.0f
    );

    // 绘制实际位置到目标位置的连线（误差线，黄色虚线）
    // 计算虚线绘制
    float dx = target_screen.x - actual_screen.x;
    float dy = target_screen.y - actual_screen.y;
    float length = sqrt(dx * dx + dy * dy);
    if (length > 1.0f) {
      float ux = dx / length;
      float uy = dy / length;

      const float dash_len = 8.0f;
      const float gap_len = 4.0f;

      for (float t = 0; t < length; t += dash_len + gap_len) {
        float end_t = std::min(t + dash_len, length);
        ImVec2 start(actual_screen.x + ux * t, actual_screen.y + uy * t);
        ImVec2 end(actual_screen.x + ux * end_t, actual_screen.y + uy * end_t);
        draw_list->AddLine(start, end, IM_COL32(255, 255, 0, 200), 2.0f);  // 黄色虚线
      }
    }

  }  // 🎯 结束轨迹跟踪绘制
  else {
    tracking_has_prev_yaw_ = false;
  }

  // 🎨 6. 绘制目标点（可选）
  if (viz_options_.show_goal) {
    auto goal_pos = worldToScreen(goal_.x, goal_.y);
    draw_list->AddCircleFilled(
      ImVec2(goal_pos.x, goal_pos.y),
      8.0f,
      IM_COL32(255, 0, 0, 255)  // 红色
    );
    draw_list->AddCircle(
      ImVec2(goal_pos.x, goal_pos.y),
      12.0f,
      IM_COL32(255, 0, 0, 255),
      0, 2.0f
    );
  }  // 🎨 结束目标点绘制

  // 🎨 7. 绘制自车（最后绘制，确保在最上层）（可选）
  if (viz_options_.show_ego) {
    // 🔧 根据底盘类型选择不同的可视化方式
    double cos_yaw = std::cos(ego_.pose.yaw);
    double sin_yaw = std::sin(ego_.pose.yaw);

    if (ego_.chassis_model == "differential") {
      // 🤖 差速底盘：精确矩形包络
      // 🔧 标准定义：驱动轴中点为原点，X 轴向前为正，Y 轴向左为正
      // 注意：差速底盘的 wheelbase = 0（只有一根驱动轴）
      double half_width = ego_.kinematics.body_width / 2.0;

      // 🔧 前保险杠 X 坐标 = front_overhang（从驱动轴开始）
      double x_front = ego_.kinematics.front_overhang;
      // 🔧 后保险杠 X 坐标 = -rear_overhang（从驱动轴开始，向后为负）
      double x_rear = -ego_.kinematics.rear_overhang;

      // 🔧 计算车辆的四个角点（在车辆局部坐标系中，驱动轴中点为原点）
      // 逆时针顺序：前左 → 前右 → 后右 → 后左
      std::vector<std::pair<double, double>> corners_local = {
        {x_front, half_width},   // P1: 前左 = (front_overhang, +width/2)
        {x_front, -half_width},  // P2: 前右 = (front_overhang, -width/2)
        {x_rear, -half_width},   // P3: 后右 = (-rear_overhang, -width/2)
        {x_rear, half_width}     // P4: 后左 = (-rear_overhang, +width/2)
      };

      // 转换到世界坐标系并转换到屏幕坐标
      std::vector<ImVec2> corners_screen;
      for (const auto& corner : corners_local) {
        double world_x = ego_.pose.x + corner.first * cos_yaw - corner.second * sin_yaw;
        double world_y = ego_.pose.y + corner.first * sin_yaw + corner.second * cos_yaw;
        auto screen_pos = worldToScreen(world_x, world_y);
        corners_screen.push_back(ImVec2(screen_pos.x, screen_pos.y));
      }

      // 绘制车辆轮廓
      draw_list->AddConvexPolyFilled(
        corners_screen.data(),
        corners_screen.size(),
        IM_COL32(0, 200, 0, 180)  // 绿色半透明
      );
      draw_list->AddPolyline(
        corners_screen.data(),
        corners_screen.size(),
        IM_COL32(0, 255, 0, 255),  // 绿色边框
        ImDrawFlags_Closed,
        2.0f
      );

      // 🔧 绘制车头方向指示（黄色圆点）
      double front_center_x = ego_.pose.x + x_front * cos_yaw;
      double front_center_y = ego_.pose.y + x_front * sin_yaw;
      auto front_pos = worldToScreen(front_center_x, front_center_y);
      draw_list->AddCircleFilled(
        ImVec2(front_pos.x, front_pos.y),
        5.0f,
        IM_COL32(255, 255, 0, 255)  // 黄色圆点
      );

      // 🔧 绘制驱动轴位置（红色小圆点，原点）
      auto drive_axle_pos = worldToScreen(ego_.pose.x, ego_.pose.y);
      draw_list->AddCircleFilled(
        ImVec2(drive_axle_pos.x, drive_axle_pos.y),
        3.0f,
        IM_COL32(255, 0, 0, 255)  // 红色圆点
      );

    } else if (ego_.chassis_model == "ackermann" || ego_.chassis_model == "four_wheel") {
      // 🚗 阿克曼/四轮底盘：矩形车辆轮廓
      // 🔧 标准定义：后轴中心为原点，X 轴向前为正，Y 轴向左为正
      double half_width = ego_.kinematics.body_width / 2.0;

      // 🔧 前保险杠 X 坐标 = wheelbase + front_overhang（从后轴开始）
      double x_front = ego_.kinematics.wheelbase + ego_.kinematics.front_overhang;
      // 🔧 后保险杠 X 坐标 = -rear_overhang（从后轴开始，向后为负）
      double x_rear = -ego_.kinematics.rear_overhang;

      // 🔧 计算车辆的四个角点（在车辆局部坐标系中，后轴为原点）
      // 逆时针顺序：前左 → 前右 → 后右 → 后左
      std::vector<std::pair<double, double>> corners_local = {
        {x_front, half_width},   // P1: 前左 = (wheelbase + front_overhang, +width/2)
        {x_front, -half_width},  // P2: 前右 = (wheelbase + front_overhang, -width/2)
        {x_rear, -half_width},   // P3: 后右 = (-rear_overhang, -width/2)
        {x_rear, half_width}     // P4: 后左 = (-rear_overhang, +width/2)
      };

      // 转换到世界坐标系并转换到屏幕坐标
      std::vector<ImVec2> corners_screen;
      for (const auto& corner : corners_local) {
        double world_x = ego_.pose.x + corner.first * cos_yaw - corner.second * sin_yaw;
        double world_y = ego_.pose.y + corner.first * sin_yaw + corner.second * cos_yaw;
        auto screen_pos = worldToScreen(world_x, world_y);
        corners_screen.push_back(ImVec2(screen_pos.x, screen_pos.y));
      }

      // 绘制车辆轮廓
      draw_list->AddConvexPolyFilled(
        corners_screen.data(),
        corners_screen.size(),
        IM_COL32(0, 200, 0, 180)  // 绿色半透明
      );
      draw_list->AddPolyline(
        corners_screen.data(),
        corners_screen.size(),
        IM_COL32(0, 255, 0, 255),  // 绿色边框
        ImDrawFlags_Closed,
        2.0f
      );

      // 🔧 绘制车头方向指示（黄色圆点）
      // 车头中心 = 后轴 + (wheelbase + front_overhang) * 方向向量
      double front_center_x = ego_.pose.x + x_front * cos_yaw;
      double front_center_y = ego_.pose.y + x_front * sin_yaw;
      auto front_pos = worldToScreen(front_center_x, front_center_y);
      draw_list->AddCircleFilled(
        ImVec2(front_pos.x, front_pos.y),
        5.0f,
        IM_COL32(255, 255, 0, 255)  // 黄色圆点
      );

      // 绘制后轴位置（红色小圆点）
      auto rear_axle_pos = worldToScreen(ego_.pose.x, ego_.pose.y);
      draw_list->AddCircleFilled(
        ImVec2(rear_axle_pos.x, rear_axle_pos.y),
        3.0f,
        IM_COL32(255, 0, 0, 255)  // 红色圆点
      );

    } else if (ego_.chassis_model == "tracked") {
      // 🚜 履带底盘：矩形 + 履带纹理
      double total_length = ego_.kinematics.front_overhang +
                           ego_.kinematics.wheelbase +
                           ego_.kinematics.rear_overhang;
      double half_width = ego_.kinematics.body_width / 2.0;

      // 车体矩形
      std::vector<std::pair<double, double>> body_corners = {
        {total_length, half_width * 0.8},   // 前左（车体稍窄）
        {total_length, -half_width * 0.8},  // 前右
        {-ego_.kinematics.rear_overhang, -half_width * 0.8},  // 后右
        {-ego_.kinematics.rear_overhang, half_width * 0.8}    // 后左
      };

      std::vector<ImVec2> body_screen;
      for (const auto& corner : body_corners) {
        double world_x = ego_.pose.x + corner.first * cos_yaw - corner.second * sin_yaw;
        double world_y = ego_.pose.y + corner.first * sin_yaw + corner.second * cos_yaw;
        auto screen_pos = worldToScreen(world_x, world_y);
        body_screen.push_back(ImVec2(screen_pos.x, screen_pos.y));
      }

      // 绘制车体
      draw_list->AddConvexPolyFilled(
        body_screen.data(),
        body_screen.size(),
        IM_COL32(0, 200, 0, 180)
      );
      draw_list->AddPolyline(
        body_screen.data(),
        body_screen.size(),
        IM_COL32(0, 255, 0, 255),
        ImDrawFlags_Closed,
        2.0f
      );

      // 绘制左右履带（矩形）
      for (int side = -1; side <= 1; side += 2) {  // -1 = 右侧, 1 = 左侧
        std::vector<std::pair<double, double>> track_corners = {
          {total_length, side * half_width},
          {total_length, side * half_width * 0.8},
          {-ego_.kinematics.rear_overhang, side * half_width * 0.8},
          {-ego_.kinematics.rear_overhang, side * half_width}
        };

        std::vector<ImVec2> track_screen;
        for (const auto& corner : track_corners) {
          double world_x = ego_.pose.x + corner.first * cos_yaw - corner.second * sin_yaw;
          double world_y = ego_.pose.y + corner.first * sin_yaw + corner.second * cos_yaw;
          auto screen_pos = worldToScreen(world_x, world_y);
          track_screen.push_back(ImVec2(screen_pos.x, screen_pos.y));
        }

        draw_list->AddConvexPolyFilled(
          track_screen.data(),
          track_screen.size(),
          IM_COL32(50, 50, 50, 200)  // 深灰色履带
        );
      }

      // 绘制车头方向指示
      double front_center_x = ego_.pose.x + total_length * cos_yaw;
      double front_center_y = ego_.pose.y + total_length * sin_yaw;
      auto front_pos = worldToScreen(front_center_x, front_center_y);
      draw_list->AddCircleFilled(
        ImVec2(front_pos.x, front_pos.y),
        5.0f,
        IM_COL32(255, 255, 0, 255)
      );

    } else {
      // 未知底盘类型：使用默认矩形
      double half_length = ego_.kinematics.body_length / 2.0;
      double half_width = ego_.kinematics.body_width / 2.0;

      std::vector<std::pair<double, double>> corners_local = {
        {half_length, half_width},
        {half_length, -half_width},
        {-half_length, -half_width},
        {-half_length, half_width}
      };

      std::vector<ImVec2> corners_screen;
      for (const auto& corner : corners_local) {
        double world_x = ego_.pose.x + corner.first * cos_yaw - corner.second * sin_yaw;
        double world_y = ego_.pose.y + corner.first * sin_yaw + corner.second * cos_yaw;
        auto screen_pos = worldToScreen(world_x, world_y);
        corners_screen.push_back(ImVec2(screen_pos.x, screen_pos.y));
      }

      draw_list->AddConvexPolyFilled(
        corners_screen.data(),
        corners_screen.size(),
        IM_COL32(0, 200, 0, 180)
      );
      draw_list->AddPolyline(
        corners_screen.data(),
        corners_screen.size(),
        IM_COL32(0, 255, 0, 255),
        ImDrawFlags_Closed,
        2.0f
      );
    }

    if (has_world_data_) {
      // 🚗 绘制车头朝向箭头
      const double arrow_length_m = 0.9;
      double arrow_end_x = ego_.pose.x + arrow_length_m * cos_yaw;
      double arrow_end_y = ego_.pose.y + arrow_length_m * sin_yaw;

      auto arrow_start = worldToScreen(ego_.pose.x, ego_.pose.y);
      auto arrow_end = worldToScreen(arrow_end_x, arrow_end_y);

      draw_list->AddLine(
        ImVec2(arrow_start.x, arrow_start.y),
        ImVec2(arrow_end.x, arrow_end.y),
        IM_COL32(255, 170, 0, 255),
        3.5f
      );
      draw_list->AddCircleFilled(ImVec2(arrow_end.x, arrow_end.y), 6.0f, IM_COL32(255, 170, 0, 255));

      // 🏎️ 绘制速度矢量（按速度大小缩放，区分正反向）
      const double speed_body = std::hypot(ego_.twist.vx, ego_.twist.vy);
      if (speed_body > 1e-3) {
        constexpr double velocity_scale = 0.6;      // 将 m/s 转成场景中的长度
        constexpr double min_visual_length = 0.25;  // 低速时的最短箭头长度（米）
        double vel_world_x = ego_.twist.vx * cos_yaw - ego_.twist.vy * sin_yaw;
        double vel_world_y = ego_.twist.vx * sin_yaw + ego_.twist.vy * cos_yaw;

        double scaled_length = speed_body * velocity_scale;
        if (scaled_length < min_visual_length) {
          double scale_up = min_visual_length / std::max(scaled_length, 1e-4);
          vel_world_x *= scale_up;
          vel_world_y *= scale_up;
          scaled_length = min_visual_length;
        }

        double vel_end_x = ego_.pose.x + vel_world_x * velocity_scale;
        double vel_end_y = ego_.pose.y + vel_world_y * velocity_scale;

        auto vel_start = worldToScreen(ego_.pose.x, ego_.pose.y);
        auto vel_end = worldToScreen(vel_end_x, vel_end_y);
        const bool moving_forward = ego_.twist.vx >= 0.0;
        ImU32 velocity_color = moving_forward
          ? IM_COL32(80, 200, 255, 255)   // 蓝色：前进
          : IM_COL32(255, 120, 120, 255); // 红色：倒退

        draw_list->AddLine(
          ImVec2(vel_start.x, vel_start.y),
          ImVec2(vel_end.x, vel_end.y),
          velocity_color,
          3.0f
        );
        ImVec2 dir_screen = ImVec2(vel_end.x - vel_start.x, vel_end.y - vel_start.y);
        float dir_len = std::sqrt(dir_screen.x * dir_screen.x + dir_screen.y * dir_screen.y);
        if (dir_len > 1e-3f) {
          ImVec2 dir_norm = ImVec2(dir_screen.x / dir_len, dir_screen.y / dir_len);
          ImVec2 normal = ImVec2(-dir_norm.y, dir_norm.x);
          const float head_len = std::min(14.0f, dir_len * 0.35f);
          const float head_width = head_len * 0.6f;
          ImVec2 tip = ImVec2(vel_end.x, vel_end.y);
          ImVec2 left = ImVec2(
            tip.x - dir_norm.x * head_len + normal.x * head_width,
            tip.y - dir_norm.y * head_len + normal.y * head_width);
          ImVec2 right = ImVec2(
            tip.x - dir_norm.x * head_len - normal.x * head_width,
            tip.y - dir_norm.y * head_len - normal.y * head_width);
          draw_list->AddTriangleFilled(tip, left, right, velocity_color);
        } else {
          draw_list->AddCircleFilled(ImVec2(vel_end.x, vel_end.y), 5.0f, velocity_color);
        }
      }
    }
  }  // 🎨 结束自车绘制

  // 🔍 处理鼠标滑轮缩放
  if (wheel_delta_ != 0) {
    if (mouse_in_canvas) {
      const double zoom_factor = 1.1;
      const double min_zoom = 0.1;
      const double max_zoom = 15.0;

      const double previous_zoom = view_state_.zoom;
      double new_zoom = previous_zoom;
      if (wheel_delta_ > 0) {
        new_zoom *= zoom_factor;
      } else {
        new_zoom /= zoom_factor;
      }
      new_zoom = std::clamp(new_zoom, min_zoom, max_zoom);

      // 如果缩放没有变化，则不再调整（例如已经达到边界）
      if (std::abs(new_zoom - previous_zoom) > 1e-6) {
        if (view_state_.follow_ego) {
          view_state_.follow_ego = false;
        }

        // 计算鼠标在缩放前的世界坐标
        const float rel_x = mouse_pos.x - (canvas_pos.x + canvas_size.x / 2.0f);
        const float rel_y = (canvas_pos.y + canvas_size.y / 2.0f) - mouse_pos.y;
        const double meters_per_pixel_prev = 1.0 / (config_.pixels_per_meter * previous_zoom);
        const double mouse_world_x = view_state_.center_x + rel_x * meters_per_pixel_prev;
        const double mouse_world_y = view_state_.center_y + rel_y * meters_per_pixel_prev;

        view_state_.zoom = new_zoom;

        const double meters_per_pixel_new = 1.0 / (config_.pixels_per_meter * view_state_.zoom);
        view_state_.center_x = mouse_world_x - rel_x * meters_per_pixel_new;
        view_state_.center_y = mouse_world_y - rel_y * meters_per_pixel_new;

        debug_info_["🔍 Zoom"] = formatDouble(view_state_.zoom, 2) + "x";
        debug_info_["🎯 View Center"] = "(" + formatDouble(view_state_.center_x, 1) + ", " + formatDouble(view_state_.center_y, 1) + ")";
      }
    }
    wheel_delta_ = 0;
  }

  // 鼠标拖动平移视图（中键或右键）
  const bool pan_button_down = scene_active &&
    (ImGui::IsMouseDown(ImGuiMouseButton_Middle) ||
     (ImGui::IsMouseDown(ImGuiMouseButton_Right) && !goal_setting_mode_));

  if (pan_button_down) {
    if (!pan_state_.active) {
      pan_state_.active = true;
      pan_state_.last_mouse_x = mouse_pos.x;
      pan_state_.last_mouse_y = mouse_pos.y;
      if (view_state_.follow_ego) {
        view_state_.follow_ego = false;
      }
    } else {
      const float dx_pixels = mouse_pos.x - pan_state_.last_mouse_x;
      const float dy_pixels = mouse_pos.y - pan_state_.last_mouse_y;
      if (std::abs(dx_pixels) > 1e-3f || std::abs(dy_pixels) > 1e-3f) {
        const double meters_per_pixel = 1.0 / (config_.pixels_per_meter * view_state_.zoom);
        view_state_.center_x -= dx_pixels * meters_per_pixel;
        view_state_.center_y += dy_pixels * meters_per_pixel;
        pan_state_.last_mouse_x = mouse_pos.x;
        pan_state_.last_mouse_y = mouse_pos.y;
        debug_info_["🎯 View Center"] = "(" + formatDouble(view_state_.center_x, 1) + ", " + formatDouble(view_state_.center_y, 1) + ")";
      }
    }
  } else {
    pan_state_.active = false;
  }

  // 处理目标点设置的鼠标点击事件
  if (goal_setting_mode_) {
    // 检查鼠标是否在画布区域内
    ImVec2 mouse_pos = ImGui::GetMousePos();
    if (mouse_pos.x >= canvas_pos.x && mouse_pos.x <= canvas_pos.x + canvas_size.x &&
        mouse_pos.y >= canvas_pos.y && mouse_pos.y <= canvas_pos.y + canvas_size.y) {

      // 检查鼠标左键是否被点击
      if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        // 计算相对于画布中心的坐标
        float rel_x = mouse_pos.x - (canvas_pos.x + canvas_size.x / 2.0f);
        float rel_y = (canvas_pos.y + canvas_size.y / 2.0f) - mouse_pos.y;  // Y轴翻转

        // 转换为世界坐标
        double world_x = view_state_.center_x + rel_x / (config_.pixels_per_meter * view_state_.zoom);
        double world_y = view_state_.center_y + rel_y / (config_.pixels_per_meter * view_state_.zoom);

        // 设置新的目标点
        new_goal_.x = world_x;
        new_goal_.y = world_y;
        new_goal_.yaw = 0.0;  // 默认朝向
        has_new_goal_ = true;

        // 退出目标点设置模式
        setGoalSettingMode(false);
      }
    }
  }

  ImGui::End();
}

void ImGuiVisualizer::addButtonLog(const std::string& log) {
  // 避免悬停事件产生太多日志（每秒最多记录一次悬停）
  static std::string last_hover_log;
  static auto last_hover_time = std::chrono::steady_clock::now();

  if (log.find("HOVERED") != std::string::npos) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_hover_time).count();

    // 如果是相同的悬停事件且距离上次记录不到1秒，跳过
    if (log == last_hover_log && elapsed < 1000) {
      return;
    }

    last_hover_log = log;
    last_hover_time = now;
  }

  // 添加时间戳
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;

  char time_str[32];
  std::strftime(time_str, sizeof(time_str), "%H:%M:%S", std::localtime(&time_t));

  std::string timestamped_log = std::string(time_str) + "." +
    std::to_string(ms.count()) + " - " + log;

  button_logs_.push_back(timestamped_log);

  // 只保留最近20条日志
  if (button_logs_.size() > 20) {
    button_logs_.erase(button_logs_.begin());
  }
}

// 🔧 公共方法：添加日志到按钮日志显示区域
void ImGuiVisualizer::addLog(const std::string& log) {
  addButtonLog(log);
}

void ImGuiVisualizer::renderDebugPanel() {
  // 创建调试信息面板 - 右侧区域
  // 位置：紧贴 Scene View 右侧，宽度600，高度850（与场景区域高度一致）
  ImGui::SetNextWindowPos(ImVec2(1190, 0), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(610, 850), ImGuiCond_Always);

  ImGui::Begin("Debug Info", nullptr, ImGuiWindowFlags_NoCollapse);

  ImGui::Text("NavSim Local Visualizer");
  ImGui::Separator();

  // 🎮 仿真控制按钮区域
  ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "Simulation Control:");

  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10, 5));

  // Start 按钮（绿色）
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.8f, 0.3f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.4f, 0.9f, 0.4f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.2f, 0.7f, 0.2f, 1.0f));
  bool start_clicked = ImGui::Button("Start", ImVec2(80, 0));
  bool start_hovered = ImGui::IsItemHovered();
  bool start_active = ImGui::IsItemActive();
  ImGui::PopStyleColor(3);

  if (start_hovered) {
    addButtonLog("Start HOVERED");
  }
  if (start_active) {
    addButtonLog("Start ACTIVE (mouse down)");
  }
  if (start_clicked) {
    addButtonLog("Start CLICKED (returned true)");
    if (sim_start_callback_) {
      sim_start_callback_();
    }
  }

  ImGui::SameLine();

  // Pause 按钮（黄色）
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.8f, 0.3f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.9f, 0.4f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.7f, 0.7f, 0.2f, 1.0f));
  bool pause_clicked = ImGui::Button("Pause", ImVec2(80, 0));
  bool pause_hovered = ImGui::IsItemHovered();
  bool pause_active = ImGui::IsItemActive();
  bool pause_released = pause_hovered && ImGui::IsMouseReleased(ImGuiMouseButton_Left);
  ImGui::PopStyleColor(3);

  if (pause_hovered) {
    addButtonLog("Pause HOVERED");
  }
  if (pause_active) {
    addButtonLog("Pause ACTIVE (mouse down)");
  }
  if (pause_released) {
    addButtonLog("Pause RELEASED (manual detection)");
  }
  if (pause_clicked) {
    addButtonLog("Pause CLICKED (returned true)");
    if (sim_pause_callback_) {
      sim_pause_callback_();
    }
  }
  // 手动触发回调（如果检测到释放）
  if (pause_released && !pause_clicked) {
    addButtonLog("Pause MANUAL TRIGGER");
    if (sim_pause_callback_) {
      sim_pause_callback_();
    }
  }

  ImGui::SameLine();

  // Reset 按钮（红色）
  ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.3f, 0.3f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.4f, 0.4f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.7f, 0.2f, 0.2f, 1.0f));
  bool reset_clicked = ImGui::Button("Reset", ImVec2(80, 0));
  bool reset_hovered = ImGui::IsItemHovered();
  bool reset_active = ImGui::IsItemActive();
  bool reset_released = reset_hovered && ImGui::IsMouseReleased(ImGuiMouseButton_Left);
  ImGui::PopStyleColor(3);

  if (reset_hovered) {
    addButtonLog("Reset HOVERED");
  }
  if (reset_active) {
    addButtonLog("Reset ACTIVE (mouse down)");
  }
  if (reset_released) {
    addButtonLog("Reset RELEASED (manual detection)");
  }
  if (reset_clicked) {
    addButtonLog("Reset CLICKED (returned true)");
    clearAllVisualizationData();  // 清空所有可视化数据
    if (sim_reset_callback_) {
      sim_reset_callback_();
    }
  }
  // 手动触发回调（如果检测到释放）
  if (reset_released && !reset_clicked) {
    addButtonLog("Reset MANUAL TRIGGER");
    clearAllVisualizationData();  // 清空所有可视化数据
    if (sim_reset_callback_) {
      sim_reset_callback_();
    }
  }

  ImGui::PopStyleVar();

  // 显示仿真状态
  ImGui::SameLine();
  if (simulation_is_paused_) {
    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "[PAUSED]");
  } else {
    ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "[RUNNING]");
  }

  // 按钮日志显示区域
  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Button Logs:");
  ImGui::BeginChild("ButtonLogs", ImVec2(0, 150), true, ImGuiWindowFlags_HorizontalScrollbar);
  for (const auto& log : button_logs_) {
    ImGui::TextWrapped("%s", log.c_str());
  }
  // 自动滚动到底部
  if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
    ImGui::SetScrollHereY(1.0f);
  }
  ImGui::EndChild();

  ImGui::Separator();

  // 🔧 场景加载功能
  ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "Load Scenario:");

  // 输入框（设置宽度为 180 像素）
  ImGui::PushItemWidth(180);
  // 使用 ImGuiInputTextFlags_EnterReturnsTrue 标志，按回车键也可以加载
  bool enter_pressed = ImGui::InputText("##ScenarioFile", scenario_path_input_,
                                         sizeof(scenario_path_input_),
                                         ImGuiInputTextFlags_EnterReturnsTrue);
  ImGui::PopItemWidth();

  ImGui::SameLine();
  bool load_clicked = ImGui::Button("Load");

  // 按回车键或点击 Load 按钮都可以加载
  if ((enter_pressed || load_clicked) && strlen(scenario_path_input_) > 0) {
    // 构建完整路径
    std::string filename = scenario_path_input_;
    std::string full_path;

    // 如果用户输入的是绝对路径，直接使用
    if (filename[0] == '/') {
      full_path = filename;
    }
    // 如果用户输入的已经包含 scenarios/ 或 ../scenarios/，直接使用
    else if (filename.find("scenarios/") == 0 || filename.find("../scenarios/") == 0) {
      full_path = filename;
    }
    // 否则，在 ../scenarios/ 目录下查找（相对于 build 目录）
    else {
      full_path = "../scenarios/" + filename;
    }

    // 🔧 如果文件名没有 .json 后缀，自动添加
    if (full_path.find(".json") == std::string::npos) {
      full_path += ".json";
    }

    scenario_path_request_ = full_path;
    has_scenario_load_request_ = true;

    // 添加到按钮日志
    addButtonLog("Load Scenario: " + full_path);
  }

  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Default dir: ../scenarios/");
  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Example: map1.json or map2.json");
  ImGui::Separator();

  // 🎨 可折叠菜单：显示控制提示
  if (ImGui::CollapsingHeader("Controls", ImGuiTreeNodeFlags_None)) {
    ImGui::BulletText("F: Toggle follow ego");
    ImGui::BulletText("+/-: Zoom in/out");
    ImGui::BulletText("ESC: Close window");
  }

  // 🎨 可折叠菜单：连接状态
  if (ImGui::CollapsingHeader("Connection", ImGuiTreeNodeFlags_None)) {
    ImGui::BulletText("Status: %s", connection_status_.connected ? "Connected" : "Disconnected");
    if (!connection_status_.label.empty()) {
      ImGui::BulletText("Target: %s", connection_status_.label.c_str());
    }
    if (!connection_status_.message.empty()) {
      ImGui::BulletText("Detail: %s", connection_status_.message.c_str());
    }
  }

  // 🎨 可折叠菜单：系统信息
  if (ImGui::CollapsingHeader("System Info", ImGuiTreeNodeFlags_None)) {
    if (system_info_.general.empty()) {
      ImGui::BulletText("No system info");
    } else {
      for (const auto& [key, value] : system_info_.general) {
        ImGui::BulletText("%s: %s", key.c_str(), value.c_str());
      }
    }
  }

  // 🎨 可折叠菜单：感知插件
  if (ImGui::CollapsingHeader("Perception Plugins", ImGuiTreeNodeFlags_None)) {
    if (system_info_.perception_plugins.empty()) {
      ImGui::BulletText("None");
    } else {
      for (const auto& name : system_info_.perception_plugins) {
        ImGui::BulletText("%s", name.c_str());
      }
    }
  }

  // 🎨 可折叠菜单：规划插件
  if (ImGui::CollapsingHeader("Planner Plugins", ImGuiTreeNodeFlags_None)) {
    if (system_info_.planner_plugins.empty()) {
      ImGui::BulletText("None");
    } else {
      for (const auto& name : system_info_.planner_plugins) {
        ImGui::BulletText("%s", name.c_str());
      }
    }
  }

  // 🎨 可折叠菜单：视图状态
  if (ImGui::CollapsingHeader("View State", ImGuiTreeNodeFlags_None)) {
    ImGui::BulletText("Follow Ego: %s", view_state_.follow_ego ? "ON" : "OFF");
    ImGui::BulletText("Zoom: %.2f", view_state_.zoom);
    ImGui::BulletText("Center: (%.2f, %.2f)", view_state_.center_x, view_state_.center_y);
  }

  // 🎨 可折叠菜单：规划上下文
  if (ImGui::CollapsingHeader("Planning Context", ImGuiTreeNodeFlags_None)) {
    if (context_info_.empty()) {
      ImGui::BulletText("Waiting for PlanningContext");
    } else {
      for (const auto& [key, value] : context_info_) {
        ImGui::BulletText("%s: %s", key.c_str(), value.c_str());
      }
    }
  }

  // 🎨 可折叠菜单：规划结果
  if (ImGui::CollapsingHeader("Planning Result", ImGuiTreeNodeFlags_None)) {
    if (!has_planning_result_) {
      ImGui::BulletText("Waiting for PlanningResult");
    } else {
      for (const auto& [key, value] : result_info_) {
        ImGui::BulletText("%s: %s", key.c_str(), value.c_str());
      }
    }
  }

  // 🎨 可折叠菜单：运行时调试信息
  if (ImGui::CollapsingHeader("Runtime Debug", ImGuiTreeNodeFlags_None)) {
    if (debug_info_.empty()) {
      ImGui::BulletText("No runtime data");
    } else {
      for (const auto& [key, value] : debug_info_) {
        ImGui::BulletText("%s: %s", key.c_str(), value.c_str());
      }
    }
  }

  ImGui::Separator();

  // 🎨 面板显示控制按钮
  if (ImGui::Button(show_legend_panel_ ? "Hide Legend" : "Show Legend")) {
    show_legend_panel_ = !show_legend_panel_;
  }
  ImGui::SameLine();
  if (ImGui::Button(show_plot_panel_ ? "Hide Plots" : "Show Plots")) {
    show_plot_panel_ = !show_plot_panel_;
  }

  ImGui::End();
}

bool ImGuiVisualizer::shouldClose() const {
  return should_close_;
}

void ImGuiVisualizer::renderLoadingScreen() {
  if (!initialized_) return;

  // 渲染单帧"加载中"画面
  // 处理事件
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (event.type == SDL_QUIT) {
      should_close_ = true;
    }
  }

  // 开始新的 ImGui 帧
  ImGui_ImplSDLRenderer2_NewFrame();
  ImGui_ImplSDL2_NewFrame();
  ImGui::NewFrame();

  // 创建全屏窗口
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(config_.window_width, config_.window_height));
  ImGui::Begin("Loading", nullptr,
               ImGuiWindowFlags_NoTitleBar |
               ImGuiWindowFlags_NoResize |
               ImGuiWindowFlags_NoMove |
               ImGuiWindowFlags_NoCollapse);

  // 居中显示"加载中"文本
  ImVec2 window_size = ImGui::GetWindowSize();
  const char* loading_text = "Initializing NavSim Local...";
  const char* status_text = "Loading scenario and plugins...";

  ImVec2 text_size = ImGui::CalcTextSize(loading_text);
  ImVec2 status_size = ImGui::CalcTextSize(status_text);

  ImGui::SetCursorPos(ImVec2(
    (window_size.x - text_size.x) * 0.5f,
    (window_size.y - text_size.y) * 0.5f - 30.0f
  ));
  ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "%s", loading_text);

  ImGui::SetCursorPos(ImVec2(
    (window_size.x - status_size.x) * 0.5f,
    (window_size.y - status_size.y) * 0.5f + 10.0f
  ));
  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "%s", status_text);

  ImGui::End();

  // 渲染
  ImGui::Render();
  SDL_SetRenderDrawColor(sdl_renderer_, 20, 20, 24, 255);
  SDL_RenderClear(sdl_renderer_);
  ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), sdl_renderer_);
  SDL_RenderPresent(sdl_renderer_);
}

void ImGuiVisualizer::shutdown() {
  if (!initialized_) return;

  // std::cout << "[ImGuiVisualizer] Shutting down..." << std::endl;

  // 清理 ImPlot
  ImPlot::DestroyContext();

  // 清理 ImGui
  ImGui_ImplSDLRenderer2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  if (imgui_context_) {
    ImGui::DestroyContext(imgui_context_);
    imgui_context_ = nullptr;
  }

  // 清理 SDL2
  if (sdl_renderer_) {
    SDL_DestroyRenderer(sdl_renderer_);
    sdl_renderer_ = nullptr;
  }
  if (window_) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }
  SDL_Quit();

  initialized_ = false;
}

// 坐标转换辅助函数
ImGuiVisualizer::Point2D ImGuiVisualizer::worldToScreen(double world_x, double world_y) const {
  // 计算相对于视图中心的偏移
  double dx = world_x - view_state_.center_x;
  double dy = world_y - view_state_.center_y;

  // 应用缩放
  dx *= config_.pixels_per_meter * view_state_.zoom;
  dy *= config_.pixels_per_meter * view_state_.zoom;

  // 获取画布信息
  ImVec2 canvas_pos = scene_canvas_pos_;
  ImVec2 canvas_size = scene_canvas_size_;

  // 转换到屏幕坐标（Y 轴翻转，因为屏幕 Y 向下，世界 Y 向上）
  float screen_x = canvas_pos.x + canvas_size.x / 2.0f + static_cast<float>(dx);
  float screen_y = canvas_pos.y + canvas_size.y / 2.0f - static_cast<float>(dy);

  return Point2D{screen_x, screen_y};
}

ImGuiVisualizer::Point2D ImGuiVisualizer::worldToScreen(const planning::Point2d& point) const {
  return worldToScreen(point.x, point.y);
}

ImGuiVisualizer::Point2D ImGuiVisualizer::screenToWorld(float screen_x, float screen_y) const {
  ImVec2 canvas_pos = scene_canvas_pos_;
  ImVec2 canvas_size = scene_canvas_size_;
  double dx = (screen_x - (canvas_pos.x + canvas_size.x / 2.0f)) / (config_.pixels_per_meter * view_state_.zoom);
  double dy = ((canvas_pos.y + canvas_size.y / 2.0f) - screen_y) / (config_.pixels_per_meter * view_state_.zoom);
  return Point2D{static_cast<float>(view_state_.center_x + dx), static_cast<float>(view_state_.center_y + dy)};
}

std::string ImGuiVisualizer::formatBool(bool value) {
  return value ? "Yes" : "No";
}

std::string ImGuiVisualizer::formatDouble(double value, int precision) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

double ImGuiVisualizer::normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

void ImGuiVisualizer::renderLegendPanel() {
  // 只有在 show_legend_panel_ 为 true 时才显示
  if (!show_legend_panel_) {
    return;
  }

  // 创建图例面板 - 浮动窗口，位置在右下角
  ImGui::SetNextWindowPos(ImVec2(1200, 450), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(600, 400), ImGuiCond_FirstUseEver);

  ImGui::Begin("Legend & Visualization Options", &show_legend_panel_, ImGuiWindowFlags_NoCollapse);

  ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Visualization Options");
  ImGui::Separator();
  ImGui::Spacing();

  // 🎨 可视化选项勾选框
  ImGui::Text("Elements:");
  ImGui::Checkbox("Show Ego Vehicle", &viz_options_.show_ego);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "[Green]");

  ImGui::Checkbox("Show Goal Point", &viz_options_.show_goal);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "[Red]");

  // Debug paths for JPS planner
  ImGui::Checkbox("Show Debug Paths", &viz_options_.show_debug_paths);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "[JPS Planner]");

  ImGui::Indent();
  if (viz_options_.show_debug_paths) {
    ImGui::Checkbox("Raw JPS Path", &viz_options_.show_raw_jps_path);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "[Red - Original Search]");

    ImGui::Checkbox("Optimized Path", &viz_options_.show_optimized_path);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "[Green - After Corner Removal]");

    ImGui::Checkbox("Sample Trajectory", &viz_options_.show_sample_trajectory);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.4f, 0.4f, 1.0f, 1.0f), "[Blue - Sampled Path]");

    ImGui::Checkbox("MINCO Final Trajectory", &viz_options_.show_minco_trajectory);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "[Yellow - Final Result]");

    ImGui::Checkbox("MINCO Stage 1 (Preprocessing)", &viz_options_.show_minco_stage1_trajectory);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.0f, 1.0f, 1.0f), "[Magenta - After Preprocessing]");

    ImGui::Checkbox("MINCO Stage 2 (Main Opt)", &viz_options_.show_minco_stage2_trajectory);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "[Cyan - After Main Optimization]");
  }
  ImGui::Unindent();

  // TMPC Planner debug paths
  ImGui::Separator();
  ImGui::Text("TMPC Planner Paths:");
  ImGui::Indent();

  ImGui::Checkbox("Main Trajectory (Final)", &viz_options_.show_trajectory);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "[Green - Bold Solid]");

  ImGui::Checkbox("Reference Path", &viz_options_.show_tmpc_reference_path);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.4f, 0.4f, 1.0f, 1.0f), "[Blue - Solid]");

  ImGui::Checkbox("Guidance Trajectories", &viz_options_.show_tmpc_guidance_paths);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "[Cyan - Dashed]");

  ImGui::Checkbox("MPC Candidates", &viz_options_.show_tmpc_mpc_candidates);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.0f, 1.0f), "[Orange - Solid]");

  ImGui::Checkbox("Obstacle Predictions", &viz_options_.show_tmpc_obstacle_predictions);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "[Red - Solid]");

  ImGui::Checkbox("Past Trajectory", &viz_options_.show_tmpc_past_trajectory);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "[Green - Solid]");

  ImGui::Unindent();

  ImGui::Checkbox("Show BEV Obstacles", &viz_options_.show_bev_obstacles);
  ImGui::Indent();
  if (viz_options_.show_bev_obstacles) {
    ImGui::BulletText("Circles:");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "[Red]");

    ImGui::BulletText("Rectangles:");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "[Green]");

    ImGui::BulletText("Polygons:");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "[Yellow]");
  }
  ImGui::Unindent();

  ImGui::Checkbox("Show Dynamic Obstacles", &viz_options_.show_dynamic_obstacles);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(1.0f, 0.0f, 1.0f, 1.0f), "[Purple]");

  ImGui::Checkbox("Show Occupancy Grid", &viz_options_.show_occupancy_grid);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "[Gray]");

  ImGui::Checkbox("Show ESDF Map", &viz_options_.show_esdf_map);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "[Cyan Border]");
  ImGui::Indent();
  if (viz_options_.show_esdf_map) {
    ImGui::BulletText("Color gradient (distance from obstacles):");
    ImGui::Indent();
    ImGui::TextColored(ImVec4(0.545f, 0.0f, 0.0f, 1.0f), "  0.0m: Dark Red");
    ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "  0.5m: Red");
    ImGui::TextColored(ImVec4(1.0f, 0.647f, 0.0f, 1.0f), "  1.0m: Orange");
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "  2.0m: Yellow");
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "  3.0m: Green");
    ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "  4.0m: Cyan");
    ImGui::TextColored(ImVec4(0.0f, 0.0f, 1.0f, 1.0f), "  5.0m: Blue");
    ImGui::Unindent();
    ImGui::BulletText("Hover mouse to see exact distance value");
  }
  ImGui::Unindent();

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Text("Display Options:");
  ImGui::Checkbox("Show Coordinate Axes", &viz_options_.show_coordinate_axes);
  ImGui::Checkbox("Show Grid Lines", &viz_options_.show_grid_lines);

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Text("View Options:");
  if (ImGui::Checkbox("Follow Ego Vehicle", &view_state_.follow_ego)) {
    // std::cout << "[ImGuiVisualizer] Follow ego: "
    //           << (view_state_.follow_ego ? "ON" : "OFF") << " (toggled from Legend panel)" << std::endl;
  }
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "(F key)");

  // 🔧 添加"适应栅格地图"按钮
  if (occupancy_grid_ && ImGui::Button("Fit Occupancy Grid")) {
    const auto& cfg = occupancy_grid_->config;

    // 计算栅格地图的中心和尺寸
    double grid_center_x = cfg.origin.x + (cfg.width * cfg.resolution) / 2.0;
    double grid_center_y = cfg.origin.y + (cfg.height * cfg.resolution) / 2.0;
    double grid_width = cfg.width * cfg.resolution;
    double grid_height = cfg.height * cfg.resolution;

    // 设置视图中心为栅格地图中心
    view_state_.center_x = grid_center_x;
    view_state_.center_y = grid_center_y;

    // 关闭跟随模式
    view_state_.follow_ego = false;

    // 计算合适的缩放倍数（留 10% 边距）
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    double zoom_x = (canvas_size.x * 0.9) / (grid_width * config_.pixels_per_meter);
    double zoom_y = (canvas_size.y * 0.9) / (grid_height * config_.pixels_per_meter);
    view_state_.zoom = std::min(zoom_x, zoom_y);

    // std::cout << "[ImGuiVisualizer] Fit occupancy grid:" << std::endl;
    // std::cout << "  - Grid center: (" << grid_center_x << ", " << grid_center_y << ")" << std::endl;
    // std::cout << "  - Grid size: " << grid_width << " x " << grid_height << " m" << std::endl;
    // std::cout << "  - Canvas size: " << canvas_size.x << " x " << canvas_size.y << " px" << std::endl;
    // std::cout << "  - New zoom: " << view_state_.zoom << std::endl;
  }
  if (!occupancy_grid_) {
    ImGui::BeginDisabled();
    ImGui::Button("Fit Occupancy Grid");
    ImGui::EndDisabled();
  }

  ImGui::Spacing();
  ImGui::Separator();

  // 快捷按钮
  ImGui::Text("Quick Actions:");
  if (ImGui::Button("Show All")) {
    viz_options_.show_ego = true;
    viz_options_.show_goal = true;
    viz_options_.show_trajectory = true;
    viz_options_.show_debug_paths = true;
    viz_options_.show_raw_jps_path = true;
    viz_options_.show_optimized_path = true;
    viz_options_.show_sample_trajectory = true;
    viz_options_.show_minco_trajectory = true;
    viz_options_.show_minco_stage1_trajectory = true;
    viz_options_.show_minco_stage2_trajectory = true;
    viz_options_.show_tmpc_reference_path = true;
    viz_options_.show_tmpc_guidance_paths = true;
    viz_options_.show_tmpc_mpc_candidates = true;
    viz_options_.show_tmpc_obstacle_predictions = true;
    viz_options_.show_tmpc_past_trajectory = true;
    viz_options_.show_bev_obstacles = true;
    viz_options_.show_dynamic_obstacles = true;
    viz_options_.show_occupancy_grid = true;
    viz_options_.show_coordinate_axes = true;
    viz_options_.show_grid_lines = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Hide All")) {
    viz_options_.show_ego = false;
    viz_options_.show_goal = false;
    viz_options_.show_trajectory = false;
    viz_options_.show_debug_paths = false;
    viz_options_.show_raw_jps_path = false;
    viz_options_.show_optimized_path = false;
    viz_options_.show_sample_trajectory = false;
    viz_options_.show_minco_trajectory = false;
    viz_options_.show_minco_stage1_trajectory = false;
    viz_options_.show_minco_stage2_trajectory = false;
    viz_options_.show_tmpc_reference_path = false;
    viz_options_.show_tmpc_guidance_paths = false;
    viz_options_.show_tmpc_mpc_candidates = false;
    viz_options_.show_tmpc_obstacle_predictions = false;
    viz_options_.show_tmpc_past_trajectory = false;
    viz_options_.show_bev_obstacles = false;
    viz_options_.show_dynamic_obstacles = false;
    viz_options_.show_occupancy_grid = false;
    viz_options_.show_coordinate_axes = false;
    viz_options_.show_grid_lines = false;
  }

  ImGui::Spacing();
  ImGui::Separator();

  // 目标点设置按钮
  ImGui::Text("Goal Setting:");
  if (goal_setting_mode_) {
    if (ImGui::Button("Cancel Goal Setting")) {
      setGoalSettingMode(false);
    }
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Click on scene to set goal");
  } else {
    if (ImGui::Button("Set New Goal")) {
      setGoalSettingMode(true);
    }
  }

  ImGui::Spacing();
  ImGui::Separator();

  // 统计信息
  ImGui::Text("Statistics:");
  ImGui::BulletText("BEV Circles: %zu", bev_obstacles_.circles.size());
  ImGui::BulletText("BEV Rectangles: %zu", bev_obstacles_.rectangles.size());
  ImGui::BulletText("BEV Polygons: %zu", bev_obstacles_.polygons.size());
  ImGui::BulletText("Dynamic Obstacles: %zu", dynamic_obstacles_.size());
  ImGui::BulletText("Trajectory Points: %zu", trajectory_.size());

  ImGui::End();
}

bool ImGuiVisualizer::hasNewGoal(planning::Pose2d& new_goal) {
  if (has_new_goal_) {
    new_goal = new_goal_;
    has_new_goal_ = false;  // 重置标志
    return true;
  }
  return false;
}

void ImGuiVisualizer::setScenarioLoadCallback(std::function<void(const std::string&)> callback) {
  scenario_load_callback_ = callback;
}

bool ImGuiVisualizer::hasScenarioLoadRequest(std::string& scenario_path) {
  if (has_scenario_load_request_) {
    scenario_path = scenario_path_request_;
    has_scenario_load_request_ = false;  // 重置标志
    return true;
  }
  return false;
}

void ImGuiVisualizer::setGoalSettingMode(bool enable) {
  goal_setting_mode_ = enable;
  // if (enable) {
  //   std::cout << "[ImGuiVisualizer] Goal setting mode enabled. Click on the scene to set new goal." << std::endl;
  // } else {
  //   std::cout << "[ImGuiVisualizer] Goal setting mode disabled." << std::endl;
  // }
}

void ImGuiVisualizer::setSimulationControlCallbacks(
  std::function<void()> start_callback,
  std::function<void()> pause_callback,
  std::function<void()> reset_callback) {
  sim_start_callback_ = start_callback;
  sim_pause_callback_ = pause_callback;
  sim_reset_callback_ = reset_callback;
}

void ImGuiVisualizer::updateSimulationStatus(bool is_paused) {
  simulation_is_paused_ = is_paused;
}

void ImGuiVisualizer::renderPlotPanel() {
  // 只有在 show_plot_panel_ 为 true 时才显示
  if (!show_plot_panel_) {
    return;
  }

  // 创建曲线图面板 - 放置在窗口底部，横跨整个窗口宽度
  // 位置：(0, 850)，尺寸：(1800, 550) - 增加高度以容纳完整的 4 个子图
  ImGui::SetNextWindowPos(ImVec2(0, 850), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(1800, 550), ImGuiCond_Always);

  ImGui::Begin("Planning Result Plots", &show_plot_panel_, ImGuiWindowFlags_NoCollapse);

  // 检查是否有规划结果数据（用于 v-s 和 omega-s 图）
  bool has_trajectory_data = has_planning_result_ && latest_planning_result_.success && !latest_planning_result_.trajectory.empty();

  // 检查是否有历史数据（用于 v-t 和 omega-t 图）
  bool has_history_data = !history_time_.empty();

  // 准备轨迹数据（用于 v-s 和 omega-s 图）
  std::vector<double> s_values, v_values, omega_values;

  if (has_trajectory_data) {
    const auto& trajectory = latest_planning_result_.trajectory;

    double cumulative_s = 0.0;
    s_values.push_back(cumulative_s);
    v_values.push_back(trajectory[0].twist.vx);
    omega_values.push_back(trajectory[0].twist.omega);

    for (size_t i = 1; i < trajectory.size(); ++i) {
      double dx = trajectory[i].pose.x - trajectory[i-1].pose.x;
      double dy = trajectory[i].pose.y - trajectory[i-1].pose.y;
      cumulative_s += std::hypot(dx, dy);

      s_values.push_back(cumulative_s);
      v_values.push_back(trajectory[i].twist.vx);
      omega_values.push_back(trajectory[i].twist.omega);
    }
  }

  // 2x2 网格布局
  float plot_width = ImGui::GetContentRegionAvail().x * 0.48f;
  float plot_height = 230.0f;

  // 第一行：v-s 图和 omega-s 图（当前规划轨迹段）
  if (ImPlot::BeginPlot("Velocity vs Distance", ImVec2(plot_width, plot_height))) {
    ImPlot::SetupAxes("Distance s (m)", "Velocity v (m/s)");
    if (has_trajectory_data && !s_values.empty()) {
      // 自适应 X 轴范围（距离）
      ImPlot::SetupAxisLimits(ImAxis_X1, 0, s_values.back() * 1.05, ImPlotCond_Always);
      // 自适应 Y 轴范围（速度），留 10% 边距，允许负值（倒车）
      auto [min_v, max_v] = std::minmax_element(v_values.begin(), v_values.end());
      double v_range = std::max(0.1, static_cast<double>(*max_v - *min_v));
      ImPlot::SetupAxisLimits(ImAxis_Y1,
        *min_v - v_range * 0.1,
        *max_v + v_range * 0.1,
        ImPlotCond_Always);
      ImPlot::PlotLine("v", s_values.data(), v_values.data(), s_values.size());
    } else {
      ImPlot::SetupAxisLimits(ImAxis_X1, 0, 10, ImPlotCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, -2, 2, ImPlotCond_Always);
    }
    ImPlot::EndPlot();
  }

  ImGui::SameLine();

  if (ImPlot::BeginPlot("Angular Velocity vs Distance", ImVec2(plot_width, plot_height))) {
    ImPlot::SetupAxes("Distance s (m)", "Angular Velocity omega (rad/s)");
    if (has_trajectory_data && !s_values.empty()) {
      // 自适应 X 轴范围（距离）
      ImPlot::SetupAxisLimits(ImAxis_X1, 0, s_values.back() * 1.05, ImPlotCond_Always);
      // 自适应 Y 轴范围（角速度），留 10% 边距，允许负值
      auto [min_omega, max_omega] = std::minmax_element(omega_values.begin(), omega_values.end());
      double omega_range = std::max(0.1, static_cast<double>(*max_omega - *min_omega));
      ImPlot::SetupAxisLimits(ImAxis_Y1,
        *min_omega - omega_range * 0.1,
        *max_omega + omega_range * 0.1,
        ImPlotCond_Always);
      ImPlot::PlotLine("omega", s_values.data(), omega_values.data(), s_values.size());
    } else {
      ImPlot::SetupAxisLimits(ImAxis_X1, 0, 10, ImPlotCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Always);
    }
    ImPlot::EndPlot();
  }

  // 第二行：v-t 图和 omega-t 图（累积历史数据）
  if (ImPlot::BeginPlot("Velocity vs Time", ImVec2(plot_width, plot_height))) {
    ImPlot::SetupAxes("Time t (s)", "Velocity v (m/s)");
    if (has_history_data) {
      // 自适应 X 轴范围（时间）
      ImPlot::SetupAxisLimits(ImAxis_X1, 0, history_time_.back() * 1.05, ImPlotCond_Always);
      // 自适应 Y 轴范围（速度），留 10% 边距，允许负值（倒车）
      auto [min_v, max_v] = std::minmax_element(history_velocity_.begin(), history_velocity_.end());
      double v_range = std::max(0.1, static_cast<double>(*max_v - *min_v));
      ImPlot::SetupAxisLimits(ImAxis_Y1,
        *min_v - v_range * 0.1,
        *max_v + v_range * 0.1,
        ImPlotCond_Always);
      ImPlot::PlotLine("v", history_time_.data(), history_velocity_.data(), history_time_.size());
    } else {
      ImPlot::SetupAxisLimits(ImAxis_X1, 0, 10, ImPlotCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, -2, 2, ImPlotCond_Always);
    }
    ImPlot::EndPlot();
  }

  ImGui::SameLine();

  if (ImPlot::BeginPlot("Angular Velocity vs Time", ImVec2(plot_width, plot_height))) {
    ImPlot::SetupAxes("Time t (s)", "Angular Velocity omega (rad/s)");
    if (has_history_data) {
      // 自适应 X 轴范围（时间）
      ImPlot::SetupAxisLimits(ImAxis_X1, 0, history_time_.back() * 1.05, ImPlotCond_Always);
      // 自适应 Y 轴范围（角速度），留 10% 边距，允许负值
      auto [min_omega, max_omega] = std::minmax_element(history_omega_.begin(), history_omega_.end());
      double omega_range = std::max(0.1, static_cast<double>(*max_omega - *min_omega));
      ImPlot::SetupAxisLimits(ImAxis_Y1,
        *min_omega - omega_range * 0.1,
        *max_omega + omega_range * 0.1,
        ImPlotCond_Always);
      ImPlot::PlotLine("omega", history_time_.data(), history_omega_.data(), history_time_.size());
    } else {
      ImPlot::SetupAxisLimits(ImAxis_X1, 0, 10, ImPlotCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, -1, 1, ImPlotCond_Always);
    }
    ImPlot::EndPlot();
  }

  ImGui::Separator();
  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                     "Note: Top row shows current trajectory; Bottom row shows cumulative history");

  ImGui::End();
}

} // namespace viz
} // namespace navsim

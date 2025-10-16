#include "viz/imgui_visualizer.hpp"
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>  // 使用 SDL_Renderer 后端
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
  std::cout << "[ImGuiVisualizer] Available render drivers (" << num_drivers << "):" << std::endl;
  for (int i = 0; i < num_drivers; ++i) {
    SDL_RendererInfo info;
    SDL_GetRenderDriverInfo(i, &info);
    std::cout << "  [" << i << "] " << info.name << std::endl;
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

  initialized_ = true;

  // 获取渲染器信息
  SDL_RendererInfo renderer_info;
  SDL_GetRendererInfo(sdl_renderer_, &renderer_info);

  std::cout << "[ImGuiVisualizer] ========== Initialized successfully ==========" << std::endl;
  std::cout << "[ImGuiVisualizer] Window size: " << config_.window_width << "x" << config_.window_height << std::endl;
  std::cout << "[ImGuiVisualizer] Renderer: " << renderer_info.name << std::endl;
  std::cout << "[ImGuiVisualizer] Using SDL_Renderer (no OpenGL dependency)" << std::endl;
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

  // 调试输出
  static int frame_count = 0;
  if (frame_count++ % 60 == 0) {  // 每 60 帧输出一次
    std::cout << "[Viz] Frame " << frame_count
              << ", Ego: (" << ego_.pose.x << ", " << ego_.pose.y << ")"
              << ", Trajectory: " << trajectory_.size() << " points"
              << ", BEV circles: " << bev_obstacles_.circles.size()
              << std::endl;
  }
}

void ImGuiVisualizer::handleEvents() {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);
    
    if (event.type == SDL_QUIT) {
      should_close_ = true;
    }
    
    if (event.type == SDL_WINDOWEVENT && 
        event.window.event == SDL_WINDOWEVENT_CLOSE &&
        event.window.windowID == SDL_GetWindowID(window_)) {
      should_close_ = true;
    }

    // 处理键盘事件
    if (event.type == SDL_KEYDOWN) {
      switch (event.key.keysym.sym) {
        case SDLK_ESCAPE:
          should_close_ = true;
          break;
        case SDLK_f:
          view_state_.follow_ego = !view_state_.follow_ego;
          std::cout << "[ImGuiVisualizer] Follow ego: " 
                    << (view_state_.follow_ego ? "ON" : "OFF") << std::endl;
          break;
        case SDLK_EQUALS:  // '+' key
          view_state_.zoom *= 1.2;
          std::cout << "[ImGuiVisualizer] Zoom: " << view_state_.zoom << std::endl;
          break;
        case SDLK_MINUS:   // '-' key
          view_state_.zoom /= 1.2;
          std::cout << "[ImGuiVisualizer] Zoom: " << view_state_.zoom << std::endl;
          break;
      }
    }
  }
}

void ImGuiVisualizer::drawEgo(const planning::EgoVehicle& ego) {
  static int call_count = 0;
  if (call_count++ % 60 == 0) {
    std::cout << "[Viz] drawEgo called: pos=(" << ego.pose.x << ", " << ego.pose.y
              << "), yaw=" << ego.pose.yaw << std::endl;
  }
  ego_ = ego;
  has_world_data_ = true;
  last_world_update_ = std::chrono::steady_clock::now();
  debug_info_["Ego Pose"] = "x=" + formatDouble(ego.pose.x) +
                            ", y=" + formatDouble(ego.pose.y) +
                            ", yaw=" + formatDouble(ego.pose.yaw, 3);
  debug_info_["Ego Speed"] = formatDouble(std::hypot(ego.twist.vx, ego.twist.vy)) + " m/s";
  
  // 更新视图中心（如果跟随自车）
  if (view_state_.follow_ego) {
    view_state_.center_x = ego.pose.x;
    view_state_.center_y = ego.pose.y;
  }
}

void ImGuiVisualizer::drawGoal(const planning::Pose2d& goal) {
  goal_ = goal;
  debug_info_["Goal"] = "x=" + formatDouble(goal.x) +
                        ", y=" + formatDouble(goal.y);
}

void ImGuiVisualizer::drawBEVObstacles(const planning::BEVObstacles& obstacles) {
  // 🔍 调试日志：检查传入的障碍物数据
  std::cout << "[ImGuiVisualizer] drawBEVObstacles called:" << std::endl;
  std::cout << "[ImGuiVisualizer]   Input circles: " << obstacles.circles.size() << std::endl;
  std::cout << "[ImGuiVisualizer]   Input rectangles: " << obstacles.rectangles.size() << std::endl;
  std::cout << "[ImGuiVisualizer]   Input polygons: " << obstacles.polygons.size() << std::endl;

  // 缓存障碍物数据，在 renderScene() 中绘制
  bev_obstacles_ = obstacles;

  // 🔍 调试日志：检查缓存后的数据
  std::cout << "[ImGuiVisualizer]   Cached circles: " << bev_obstacles_.circles.size() << std::endl;
  std::cout << "[ImGuiVisualizer]   Cached rectangles: " << bev_obstacles_.rectangles.size() << std::endl;
  std::cout << "[ImGuiVisualizer]   Cached polygons: " << bev_obstacles_.polygons.size() << std::endl;

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
  static int call_count = 0;
  if (call_count++ % 60 == 0) {
    std::cout << "[Viz] drawTrajectory called: " << trajectory.size() << " points, planner=" << planner_name << std::endl;
    if (!trajectory.empty()) {
      std::cout << "[Viz]   First point: (" << trajectory[0].pose.x << ", " << trajectory[0].pose.y << ")" << std::endl;
    }
  }
  trajectory_ = trajectory;
  planner_name_ = planner_name;
  has_planning_result_ = true;
  debug_info_["Trajectory Points"] = std::to_string(trajectory.size());
  debug_info_["Planner"] = planner_name;
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

  if (!result.metadata.empty()) {
    std::ostringstream oss;
    for (auto it = result.metadata.begin(); it != result.metadata.end(); ++it) {
      if (it != result.metadata.begin()) {
        oss << ", ";
      }
      oss << it->first << "=" << formatDouble(it->second, 3);
    }
    result_info_["Metadata"] = oss.str();
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

  // 渲染 ImGui - SDL_Renderer 流程
  ImGui::Render();

  // 1. 设置渲染颜色并清屏
  SDL_SetRenderDrawColor(sdl_renderer_, 20, 20, 24, 255);  // 深灰色背景
  SDL_RenderClear(sdl_renderer_);

  // 2. 渲染 ImGui 绘制数据（需要传入 renderer）
  ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), sdl_renderer_);

  // 3. 呈现到屏幕
  SDL_RenderPresent(sdl_renderer_);
}

void ImGuiVisualizer::renderScene() {
  static int render_count = 0;
  if (render_count++ % 60 == 0) {
    std::cout << "[Viz] renderScene called #" << render_count
              << ", has_world_data=" << has_world_data_
              << ", has_planning_result=" << has_planning_result_ << std::endl;
  }

  // 创建主场景窗口
  ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(1000, 900), ImGuiCond_FirstUseEver);

  ImGui::Begin("Scene View", nullptr, ImGuiWindowFlags_NoCollapse);

  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
  ImVec2 canvas_size = ImGui::GetContentRegionAvail();

  static int log_count = 0;
  if (log_count++ % 60 == 0) {
    std::cout << "[Viz]   Canvas pos=(" << canvas_pos.x << ", " << canvas_pos.y
              << "), size=(" << canvas_size.x << ", " << canvas_size.y << ")" << std::endl;
  }

  // 绘制背景
  draw_list->AddRectFilled(canvas_pos,
                           ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                           IM_COL32(20, 20, 20, 255));

  auto now = std::chrono::steady_clock::now();
  if (!has_world_data_) {
    draw_list->AddText(ImVec2(canvas_pos.x + 20.0f, canvas_pos.y + 20.0f),
                       IM_COL32(200, 200, 200, 255),
                       "Waiting for world data...");
  } else {
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

  // 测试绘制 - 仅在尚未收到数据时提示
  if (!has_world_data_) {
    ImVec2 center(canvas_pos.x + canvas_size.x / 2.0f, canvas_pos.y + canvas_size.y / 2.0f);
    draw_list->AddCircleFilled(center, 50.0f, IM_COL32(255, 255, 255, 255));
    draw_list->AddText(ImVec2(center.x - 50, center.y - 100), IM_COL32(255, 255, 0, 255),
                       "TEST CIRCLE - If you see this, rendering works!");
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
    static int grid_log_count = 0;
    if (grid_log_count++ % 60 == 0) {
      std::cout << "[Viz] Occupancy Grid Boundary:" << std::endl;
      std::cout << "  - Grid size: " << cfg.width << "x" << cfg.height << std::endl;
      std::cout << "  - Resolution: " << cfg.resolution << " m" << std::endl;
      std::cout << "  - Origin: (" << cfg.origin.x << ", " << cfg.origin.y << ")" << std::endl;
      std::cout << "  - World bounds: X=[" << grid_min_x << ", " << grid_max_x << "], Y=[" << grid_min_y << ", " << grid_max_y << "]" << std::endl;
      std::cout << "  - World size: " << (grid_max_x - grid_min_x) << " x " << (grid_max_y - grid_min_y) << " m" << std::endl;
      std::cout << "  - View center: (" << view_state_.center_x << ", " << view_state_.center_y << ")" << std::endl;
      std::cout << "  - View zoom: " << view_state_.zoom << std::endl;
    }

    auto boundary_p1_temp = worldToScreen(grid_min_x, grid_min_y);
    auto boundary_p2_temp = worldToScreen(grid_max_x, grid_min_y);
    auto boundary_p3_temp = worldToScreen(grid_max_x, grid_max_y);
    auto boundary_p4_temp = worldToScreen(grid_min_x, grid_max_y);

    ImVec2 boundary_p1(boundary_p1_temp.x, boundary_p1_temp.y);
    ImVec2 boundary_p2(boundary_p2_temp.x, boundary_p2_temp.y);
    ImVec2 boundary_p3(boundary_p3_temp.x, boundary_p3_temp.y);
    ImVec2 boundary_p4(boundary_p4_temp.x, boundary_p4_temp.y);

    // 🔍 调试信息：打印屏幕坐标
    if (grid_log_count % 60 == 1) {
      std::cout << "  - Screen coords: P1=(" << boundary_p1.x << ", " << boundary_p1.y << "), "
                << "P2=(" << boundary_p2.x << ", " << boundary_p2.y << "), "
                << "P3=(" << boundary_p3.x << ", " << boundary_p3.y << "), "
                << "P4=(" << boundary_p4.x << ", " << boundary_p4.y << ")" << std::endl;
    }

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
  static int esdf_viz_log_count = 0;
  if (viz_options_.show_esdf_map && esdf_map_) {
    const auto& esdf = *esdf_map_;
    const auto& cfg = esdf.config;

    // 调试信息（每 60 帧打印一次）
    if (esdf_viz_log_count++ % 60 == 0) {
      std::cout << "[Viz] Drawing ESDF map: " << cfg.width << "x" << cfg.height
                << " @" << cfg.resolution << "m, origin=(" << cfg.origin.x << ", " << cfg.origin.y << ")"
                << ", data_size=" << esdf.data.size() << std::endl;
    }

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
    if (obstacle_log_count++ % 60 == 0 && !bev_obstacles_.circles.empty()) {
      std::cout << "[Viz]   Drawing " << bev_obstacles_.circles.size() << " BEV circles" << std::endl;
    auto test_center = worldToScreen(bev_obstacles_.circles[0].center);
    std::cout << "[Viz]     First circle: world=(" << bev_obstacles_.circles[0].center.x
              << ", " << bev_obstacles_.circles[0].center.y
              << ") -> screen=(" << test_center.x << ", " << test_center.y << ")" << std::endl;
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
  if (obstacle_log_count % 60 == 0 && !bev_obstacles_.rectangles.empty()) {
    std::cout << "[Viz]   Drawing " << bev_obstacles_.rectangles.size() << " BEV rectangles" << std::endl;
    std::cout << "[Viz]     First rect: world=(" << bev_obstacles_.rectangles[0].pose.x
              << ", " << bev_obstacles_.rectangles[0].pose.y
              << "), size=(" << bev_obstacles_.rectangles[0].width
              << " x " << bev_obstacles_.rectangles[0].height << ")" << std::endl;
  }

  for (const auto& rect : bev_obstacles_.rectangles) {
    auto center = worldToScreen(rect.pose.x, rect.pose.y);
    float w = rect.width * config_.pixels_per_meter * view_state_.zoom;
    float h = rect.height * config_.pixels_per_meter * view_state_.zoom;

    // 简化：绘制为圆形（完整的旋转矩形需要更复杂的计算）
    float radius = std::sqrt(w * w + h * h) / 2.0f;

    if (obstacle_log_count % 60 == 0) {
      auto screen_pos = worldToScreen(rect.pose.x, rect.pose.y);
      std::cout << "[Viz]       Rect screen pos=(" << screen_pos.x << ", " << screen_pos.y
                << "), radius=" << radius << std::endl;
    }

    draw_list->AddCircleFilled(
      ImVec2(center.x, center.y),
      radius,
      IM_COL32(100, 255, 100, 200)  // 🔧 改为绿色，更容易区分
    );
    draw_list->AddCircle(
      ImVec2(center.x, center.y),
      radius,
      IM_COL32(0, 255, 0, 255),  // 绿色边框
      0, 2.0f
    );
  }

  // 3. 绘制 BEV 障碍物 - 多边形
  if (obstacle_log_count % 60 == 0 && !bev_obstacles_.polygons.empty()) {
    std::cout << "[Viz]   Drawing " << bev_obstacles_.polygons.size() << " BEV polygons" << std::endl;
    std::cout << "[Viz]     First polygon: " << bev_obstacles_.polygons[0].vertices.size() << " vertices" << std::endl;
  }

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
    if (dyn_obs_log_count++ % 60 == 0 && !dynamic_obstacles_.empty()) {
      std::cout << "[Viz]   Drawing " << dynamic_obstacles_.size() << " dynamic obstacles" << std::endl;
    // 🔧 修复问题1：打印所有障碍物的信息
    for (size_t i = 0; i < dynamic_obstacles_.size(); ++i) {
      const auto& obs = dynamic_obstacles_[i];
      std::cout << "[Viz]     Dyn obs #" << i << ": shape=" << obs.shape_type
                << ", pos=(" << obs.current_pose.x << ", " << obs.current_pose.y
                << "), length=" << obs.length << ", width=" << obs.width << std::endl;
    }
  }

  for (const auto& dyn_obs : dynamic_obstacles_) {
    auto center = worldToScreen(dyn_obs.current_pose.x, dyn_obs.current_pose.y);

    // 🔧 修复问题4：使用 shape_type 判断，而不是长宽相等
    bool is_circle = (dyn_obs.shape_type == "circle");

    if (is_circle) {
      // 绘制圆形动态障碍物
      float radius = dyn_obs.length / 2.0f * config_.pixels_per_meter * view_state_.zoom;

      if (dyn_obs_log_count % 60 == 0) {
        std::cout << "[Viz]       Dyn obs (circle) radius=" << radius << " pixels (diameter=" << dyn_obs.length << ")" << std::endl;
      }

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

      if (dyn_obs_log_count % 60 == 0) {
        std::cout << "[Viz]       Dyn obs (rect) visualization:" << std::endl;
        std::cout << "[Viz]         dyn_obs.width = " << dyn_obs.width << " m" << std::endl;
        std::cout << "[Viz]         dyn_obs.length = " << dyn_obs.length << " m" << std::endl;
        std::cout << "[Viz]         dyn_obs.current_pose.yaw = " << yaw << " rad" << std::endl;
        std::cout << "[Viz]         Screen size: w=" << w << " px, h=" << h << " px" << std::endl;
        std::cout << "[Viz]         Velocity: vx=" << dyn_obs.current_twist.vx
                  << ", vy=" << dyn_obs.current_twist.vy << std::endl;
      }

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

    // 绘制速度箭头
    if (std::abs(dyn_obs.current_twist.vx) > 0.01 || std::abs(dyn_obs.current_twist.vy) > 0.01) {
      auto vel_end = worldToScreen(
        dyn_obs.current_pose.x + dyn_obs.current_twist.vx,
        dyn_obs.current_pose.y + dyn_obs.current_twist.vy
      );
      draw_list->AddLine(
        ImVec2(center.x, center.y),
        ImVec2(vel_end.x, vel_end.y),
        IM_COL32(255, 255, 0, 255),  // 黄色箭头
        2.0f
      );
    }
  }
  }  // 🎨 结束动态障碍物绘制

  // 🎨 5. 绘制规划轨迹（可选）
  if (viz_options_.show_trajectory) {
    static int traj_log_count = 0;
    if (traj_log_count++ % 60 == 0 && trajectory_.size() > 1) {
      std::cout << "[Viz]   Drawing trajectory with " << trajectory_.size() << " points" << std::endl;
    auto test_p1 = worldToScreen(trajectory_[0].pose.x, trajectory_[0].pose.y);
    auto test_p2 = worldToScreen(trajectory_[1].pose.x, trajectory_[1].pose.y);
    std::cout << "[Viz]     First segment: (" << test_p1.x << "," << test_p1.y
              << ") -> (" << test_p2.x << "," << test_p2.y << ")" << std::endl;
  }

    if (trajectory_.size() > 1) {
      for (size_t i = 1; i < trajectory_.size(); ++i) {
        auto p1 = worldToScreen(trajectory_[i-1].pose.x, trajectory_[i-1].pose.y);
        auto p2 = worldToScreen(trajectory_[i].pose.x, trajectory_[i].pose.y);
        draw_list->AddLine(
          ImVec2(p1.x, p1.y),
          ImVec2(p2.x, p2.y),
          IM_COL32(0, 255, 255, 255),  // 青色
          3.0f
        );
      }
    }
  }  // 🎨 结束轨迹绘制

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
  }  // 🎨 结束自车绘制

  ImGui::End();
}

void ImGuiVisualizer::renderDebugPanel() {
  // 创建调试信息面板
  ImGui::SetNextWindowPos(ImVec2(1010, 0), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(390, 900), ImGuiCond_FirstUseEver);
  
  ImGui::Begin("Debug Info", nullptr, ImGuiWindowFlags_NoCollapse);
  
  ImGui::Text("NavSim Local Visualizer");
  ImGui::Separator();
  
  // 显示控制提示
  ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "Controls:");
  ImGui::BulletText("F: Toggle follow ego");
  ImGui::BulletText("+/-: Zoom in/out");
  ImGui::BulletText("ESC: Close window");
  ImGui::Separator();

  ImGui::Text("Connection:");
  ImGui::BulletText("Status: %s", connection_status_.connected ? "Connected" : "Disconnected");
  if (!connection_status_.label.empty()) {
    ImGui::BulletText("Target: %s", connection_status_.label.c_str());
  }
  if (!connection_status_.message.empty()) {
    ImGui::BulletText("Detail: %s", connection_status_.message.c_str());
  }
  ImGui::Separator();

  ImGui::Text("System Info:");
  if (system_info_.general.empty()) {
    ImGui::BulletText("No system info");
  } else {
    for (const auto& [key, value] : system_info_.general) {
      ImGui::BulletText("%s: %s", key.c_str(), value.c_str());
    }
  }
  ImGui::Separator();

  ImGui::Text("Perception Plugins:");
  if (system_info_.perception_plugins.empty()) {
    ImGui::BulletText("None");
  } else {
    for (const auto& name : system_info_.perception_plugins) {
      ImGui::BulletText("%s", name.c_str());
    }
  }
  ImGui::Separator();

  ImGui::Text("Planner Plugins:");
  if (system_info_.planner_plugins.empty()) {
    ImGui::BulletText("None");
  } else {
    for (const auto& name : system_info_.planner_plugins) {
      ImGui::BulletText("%s", name.c_str());
    }
  }
  ImGui::Separator();

  // 显示视图状态
  ImGui::Text("View State:");
  ImGui::BulletText("Follow Ego: %s", view_state_.follow_ego ? "ON" : "OFF");
  ImGui::BulletText("Zoom: %.2f", view_state_.zoom);
  ImGui::BulletText("Center: (%.2f, %.2f)", view_state_.center_x, view_state_.center_y);
  ImGui::Separator();

  ImGui::Text("Planning Context:");
  if (context_info_.empty()) {
    ImGui::BulletText("Waiting for PlanningContext");
  } else {
    for (const auto& [key, value] : context_info_) {
      ImGui::BulletText("%s: %s", key.c_str(), value.c_str());
    }
  }
  ImGui::Separator();

  ImGui::Text("Planning Result:");
  if (!has_planning_result_) {
    ImGui::BulletText("Waiting for PlanningResult");
  } else {
    for (const auto& [key, value] : result_info_) {
      ImGui::BulletText("%s: %s", key.c_str(), value.c_str());
    }
  }
  ImGui::Separator();

  // 显示调试信息
  ImGui::Text("Runtime Debug:");
  if (debug_info_.empty()) {
    ImGui::BulletText("No runtime data");
  } else {
    for (const auto& [key, value] : debug_info_) {
      ImGui::BulletText("%s: %s", key.c_str(), value.c_str());
    }
  }
  
  ImGui::End();
}

bool ImGuiVisualizer::shouldClose() const {
  return should_close_;
}

void ImGuiVisualizer::shutdown() {
  if (!initialized_) return;

  std::cout << "[ImGuiVisualizer] Shutting down..." << std::endl;

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
  ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
  ImVec2 canvas_size = ImGui::GetContentRegionAvail();

  // 转换到屏幕坐标（Y 轴翻转，因为屏幕 Y 向下，世界 Y 向上）
  float screen_x = canvas_pos.x + canvas_size.x / 2.0f + static_cast<float>(dx);
  float screen_y = canvas_pos.y + canvas_size.y / 2.0f - static_cast<float>(dy);

  return Point2D{screen_x, screen_y};
}

ImGuiVisualizer::Point2D ImGuiVisualizer::worldToScreen(const planning::Point2d& point) const {
  return worldToScreen(point.x, point.y);
}

std::string ImGuiVisualizer::formatBool(bool value) {
  return value ? "Yes" : "No";
}

std::string ImGuiVisualizer::formatDouble(double value, int precision) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

void ImGuiVisualizer::renderLegendPanel() {
  // 创建图例面板（Legend Panel）
  ImGui::SetNextWindowPos(ImVec2(1010, 450), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(390, 450), ImGuiCond_FirstUseEver);

  ImGui::Begin("Legend & Visualization Options", nullptr, ImGuiWindowFlags_NoCollapse);

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

  ImGui::Checkbox("Show Trajectory", &viz_options_.show_trajectory);
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.0f, 1.0f, 1.0f, 1.0f), "[Cyan]");

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
    std::cout << "[ImGuiVisualizer] Follow ego: "
              << (view_state_.follow_ego ? "ON" : "OFF") << " (toggled from Legend panel)" << std::endl;
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

    std::cout << "[ImGuiVisualizer] Fit occupancy grid:" << std::endl;
    std::cout << "  - Grid center: (" << grid_center_x << ", " << grid_center_y << ")" << std::endl;
    std::cout << "  - Grid size: " << grid_width << " x " << grid_height << " m" << std::endl;
    std::cout << "  - Canvas size: " << canvas_size.x << " x " << canvas_size.y << " px" << std::endl;
    std::cout << "  - New zoom: " << view_state_.zoom << std::endl;
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
    viz_options_.show_bev_obstacles = false;
    viz_options_.show_dynamic_obstacles = false;
    viz_options_.show_occupancy_grid = false;
    viz_options_.show_coordinate_axes = false;
    viz_options_.show_grid_lines = false;
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

} // namespace viz
} // namespace navsim

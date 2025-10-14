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
  // 缓存障碍物数据，在 renderScene() 中绘制
  bev_obstacles_ = obstacles;
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

  // 绘制网格
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

  // 绘制坐标轴
  auto origin = worldToScreen(0, 0);
  auto x_axis = worldToScreen(5, 0);
  auto y_axis = worldToScreen(0, 5);
  draw_list->AddLine(ImVec2(origin.x, origin.y), ImVec2(x_axis.x, x_axis.y),
                     IM_COL32(255, 0, 0, 255), 2.0f);  // X 轴红色
  draw_list->AddLine(ImVec2(origin.x, origin.y), ImVec2(y_axis.x, y_axis.y),
                     IM_COL32(0, 255, 0, 255), 2.0f);  // Y 轴绿色

  // 1. 绘制 BEV 障碍物 - 圆形
  static int obstacle_log_count = 0;
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
  for (const auto& rect : bev_obstacles_.rectangles) {
    auto center = worldToScreen(rect.pose.x, rect.pose.y);
    float w = rect.width * config_.pixels_per_meter * view_state_.zoom;
    float h = rect.height * config_.pixels_per_meter * view_state_.zoom;

    // 简化：绘制为圆形（完整的旋转矩形需要更复杂的计算）
    float radius = std::sqrt(w * w + h * h) / 2.0f;
    draw_list->AddCircleFilled(
      ImVec2(center.x, center.y),
      radius,
      IM_COL32(255, 150, 100, 200)
    );
  }

  // 3. 绘制动态障碍物
  for (const auto& dyn_obs : dynamic_obstacles_) {
    auto center = worldToScreen(dyn_obs.current_pose.x, dyn_obs.current_pose.y);
    float radius = std::max(dyn_obs.length, dyn_obs.width) / 2.0f * config_.pixels_per_meter * view_state_.zoom;
    draw_list->AddCircleFilled(
      ImVec2(center.x, center.y),
      radius,
      IM_COL32(255, 0, 255, 200)  // 紫色
    );

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

  // 4. 绘制规划轨迹
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

  // 5. 绘制目标点
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

  // 6. 绘制自车（最后绘制，确保在最上层）
  auto ego_pos = worldToScreen(ego_.pose.x, ego_.pose.y);
  float car_length = ego_.kinematics.wheelbase * config_.pixels_per_meter * view_state_.zoom;
  float car_width = ego_.kinematics.width * config_.pixels_per_meter * view_state_.zoom;

  // 简化：绘制为圆形 + 朝向箭头
  draw_list->AddCircleFilled(
    ImVec2(ego_pos.x, ego_pos.y),
    std::max(car_length, car_width) / 2.0f,
    IM_COL32(0, 255, 0, 200)  // 绿色半透明
  );

  // 绘制朝向箭头
  float arrow_len = car_length * 0.8f;
  auto arrow_end = worldToScreen(
    ego_.pose.x + arrow_len / config_.pixels_per_meter / view_state_.zoom * std::cos(ego_.pose.yaw),
    ego_.pose.y + arrow_len / config_.pixels_per_meter / view_state_.zoom * std::sin(ego_.pose.yaw)
  );
  draw_list->AddLine(
    ImVec2(ego_pos.x, ego_pos.y),
    ImVec2(arrow_end.x, arrow_end.y),
    IM_COL32(0, 255, 0, 255),  // 绿色箭头
    3.0f
  );

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

} // namespace viz
} // namespace navsim

cmake_minimum_required(VERSION 3.16)
project(navsim_local LANGUAGES CXX)

# ========== Protobuf ==========
find_package(Protobuf REQUIRED)
set(Protobuf_USE_STATIC_LIBS OFF)

set(PROTO_FILES
    proto/world_tick.proto
    proto/plan_update.proto
    proto/ego_cmd.proto)

set(PROTOBUF_IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_library(navsim_proto STATIC ${PROTO_SRCS} ${PROTO_HDRS})
set_target_properties(navsim_proto PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(navsim_proto
    PUBLIC
      ${CMAKE_CURRENT_BINARY_DIR}
      ${Protobuf_INCLUDE_DIRS})

target_link_libraries(navsim_proto PUBLIC ${Protobuf_LIBRARIES})

target_compile_features(navsim_proto PUBLIC cxx_std_17)

# ========== ixwebsocket ==========
# 配置 ixwebsocket 选项
set(USE_TLS ON CACHE BOOL "Enable TLS support for wss://")
set(USE_ZLIB ON CACHE BOOL "Enable zlib compression")

# 添加 ixwebsocket 子目录
add_subdirectory(third_party/ixwebsocket)

# ========== nlohmann/json ==========
# header-only library, 只需添加 include 路径

# ========== Eigen3 (for linear algebra) ==========
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
    # 如果系统没有安装Eigen3，可以下载header-only版本
    message(STATUS "Eigen3 not found, using header-only fallback")
    set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")
endif()

# ========== Plugin Framework Library (without concrete plugins) ==========
add_library(navsim_plugin_framework STATIC
    # Plugin framework
    src/plugin/framework/perception_plugin_manager.cpp
    src/plugin/framework/planner_plugin_manager.cpp
    src/plugin/framework/config_loader.cpp
    src/plugin/framework/plugin_init.cpp
    src/plugin/framework/dynamic_plugin_loader.cpp
    # Preprocessing
    src/plugin/preprocessing/bev_extractor.cpp
    src/plugin/preprocessing/dynamic_predictor.cpp
    src/plugin/preprocessing/basic_converter.cpp
    src/plugin/preprocessing/preprocessing_pipeline.cpp)

target_include_directories(navsim_plugin_framework
    PUBLIC
      include
      ${CMAKE_CURRENT_BINARY_DIR}
      third_party/nlohmann)

if(Eigen3_FOUND)
    target_link_libraries(navsim_plugin_framework PUBLIC Eigen3::Eigen)
    target_include_directories(navsim_plugin_framework PUBLIC ${EIGEN3_INCLUDE_DIRS})
else()
    target_include_directories(navsim_plugin_framework PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

target_link_libraries(navsim_plugin_framework
    PUBLIC
        navsim_proto
    PRIVATE
        ${CMAKE_DL_LIBS})  # 链接 libdl (dlopen/dlsym)
target_compile_features(navsim_plugin_framework PUBLIC cxx_std_17)

# ========== Plugin Sub-projects ==========
option(BUILD_PLUGINS "Build built-in plugins" ON)
if(BUILD_PLUGINS)
    add_subdirectory(plugins)
    # 定义 C++ 宏，让代码知道插件已启用
    add_compile_definitions(BUILD_PLUGINS)
endif()

# ========== Visualization (ImGui + SDL2) ==========
option(ENABLE_VISUALIZATION "Enable ImGui visualization" OFF)
if(ENABLE_VISUALIZATION)
    message(STATUS "Visualization enabled")

    # 查找 SDL2
    find_package(SDL2 REQUIRED)

    # 添加 ImGui（假设在 third_party/imgui）
    set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui)
    if(NOT EXISTS ${IMGUI_DIR})
        message(FATAL_ERROR "ImGui not found at ${IMGUI_DIR}. Please run: cd third_party && git clone https://github.com/ocornut/imgui.git")
    endif()

    # 创建 ImGui 库 - 使用 SDL_Renderer 后端（不依赖 OpenGL）
    add_library(imgui STATIC
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_demo.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
        ${IMGUI_DIR}/backends/imgui_impl_sdlrenderer2.cpp  # 使用 SDL_Renderer!
    )

    target_include_directories(imgui PUBLIC
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
        ${SDL2_INCLUDE_DIRS}
    )

    target_link_libraries(imgui PUBLIC
        ${SDL2_LIBRARIES}
        ${CMAKE_DL_LIBS}
    )

    # 不再需要 OpenGL!

    # 定义宏
    add_compile_definitions(ENABLE_VISUALIZATION)
endif()

# ========== Core Library ==========
set(NAVSIM_PLANNING_SOURCES
    src/core/planning_context.cpp
    src/core/algorithm_manager.cpp
    src/core/websocket_visualizer.cpp
    src/viz/visualizer_factory.cpp
)

# 添加可视化源文件（如果启用）
if(ENABLE_VISUALIZATION)
    list(APPEND NAVSIM_PLANNING_SOURCES
        src/viz/imgui_visualizer.cpp
    )
endif()

add_library(navsim_planning STATIC ${NAVSIM_PLANNING_SOURCES})

target_include_directories(navsim_planning
    PUBLIC
      include
      ${CMAKE_CURRENT_BINARY_DIR}
      third_party/nlohmann)

if(Eigen3_FOUND)
    target_link_libraries(navsim_planning PUBLIC Eigen3::Eigen)
    target_include_directories(navsim_planning PUBLIC ${EIGEN3_INCLUDE_DIRS})
else()
    target_include_directories(navsim_planning PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

target_link_libraries(navsim_planning
    PUBLIC
      navsim_proto
      navsim_plugin_framework
      $<$<BOOL:${BUILD_PLUGINS}>:navsim_builtin_plugins>
      $<$<BOOL:${ENABLE_VISUALIZATION}>:imgui>
)
target_compile_features(navsim_planning PUBLIC cxx_std_17)

# ========== navsim_algo executable ==========
add_executable(navsim_algo
    src/core/main.cpp
    src/core/bridge.cpp)

target_include_directories(navsim_algo
    PRIVATE
      include
      ${CMAKE_CURRENT_BINARY_DIR}
      third_party/nlohmann)  # nlohmann/json

target_link_libraries(navsim_algo
    PRIVATE
      navsim_planning  # 新增规划和感知模块
      navsim_proto
      ${Protobuf_LIBRARIES}
      ixwebsocket)  # ixwebsocket 库

target_compile_features(navsim_algo PRIVATE cxx_std_17)

# 配置 ixwebsocket 重连参数（指数回退 0.5s → 5s）
target_compile_definitions(navsim_algo PRIVATE
    IX_WS_MIN_WAIT_BETWEEN_RECONNECTION_RETRIES=500
    IX_WS_MAX_WAIT_BETWEEN_RECONNECTION_RETRIES=5000)

# ========== Test Executable ==========
add_executable(test_plugin_system
    tests/test_plugin_system.cpp
    src/core/bridge.cpp)  # 添加 bridge.cpp 以解决链接问题

target_include_directories(test_plugin_system
    PRIVATE
      include
      ${CMAKE_CURRENT_BINARY_DIR}
      third_party/nlohmann)

target_link_libraries(test_plugin_system
    PRIVATE
      navsim_planning
      navsim_proto
      ${Protobuf_LIBRARIES}
      ixwebsocket)  # 添加 ixwebsocket 库

target_compile_features(test_plugin_system PRIVATE cxx_std_17)

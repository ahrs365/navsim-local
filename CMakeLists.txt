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

# ========== Plugin System Library ==========
add_library(navsim_plugin_system STATIC
    src/plugin/perception_plugin_manager.cpp
    src/plugin/planner_plugin_manager.cpp
    src/plugin/config_loader.cpp
    src/plugin/plugin_init.cpp
    src/perception/bev_extractor.cpp
    src/perception/dynamic_predictor.cpp
    src/perception/basic_converter.cpp
    src/perception/preprocessing_pipeline.cpp
    plugins/perception/grid_map_builder_plugin.cpp
    plugins/planning/straight_line_planner_plugin.cpp
    plugins/planning/astar_planner_plugin.cpp)

target_include_directories(navsim_plugin_system
    PUBLIC
      include
      ${CMAKE_CURRENT_BINARY_DIR}
      third_party/nlohmann)

if(Eigen3_FOUND)
    target_link_libraries(navsim_plugin_system PUBLIC Eigen3::Eigen)
    target_include_directories(navsim_plugin_system PUBLIC ${EIGEN3_INCLUDE_DIRS})
else()
    target_include_directories(navsim_plugin_system PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

target_link_libraries(navsim_plugin_system PUBLIC navsim_proto)
target_compile_features(navsim_plugin_system PUBLIC cxx_std_17)

# ========== Planning and Perception Libraries ==========
add_library(navsim_planning STATIC
    src/planning_context.cpp
    src/perception_processor.cpp
    src/planner_interface.cpp
    src/algorithm_manager.cpp
    src/websocket_visualizer.cpp)

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
      navsim_plugin_system)
target_compile_features(navsim_planning PUBLIC cxx_std_17)

# ========== navsim_algo executable ==========
add_executable(navsim_algo
    src/main.cpp
    src/planner.cpp
    src/bridge.cpp)

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
    src/bridge.cpp)  # 添加 bridge.cpp 以解决链接问题

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

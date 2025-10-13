# A* Planner Plugin
#
# A grid-based A* path planner.

# ========== 插件库 ==========
# 编译为动态库（共享库），支持运行时加载
add_library(astar_planner_plugin SHARED
    src/astar_planner_plugin.cpp
    src/register.cpp)

# 设置动态库输出名称和版本
set_target_properties(astar_planner_plugin PROPERTIES
    OUTPUT_NAME "astar_planner_plugin"
    VERSION 1.0.0
    SOVERSION 1)

# ========== 包含目录 ==========
target_include_directories(astar_planner_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/navsim_plugins/astar>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ========== 依赖项 ==========
target_link_libraries(astar_planner_plugin
    PUBLIC
        navsim_plugin_framework
    PRIVATE
        Eigen3::Eigen)

# ========== 编译选项 ==========
target_compile_features(astar_planner_plugin PUBLIC cxx_std_17)

# 定义宏，让 plugin_init.cpp 知道这个插件已启用
target_compile_definitions(astar_planner_plugin PUBLIC BUILD_ASTAR_PLANNER_PLUGIN)

target_compile_definitions(astar_planner_plugin
    PRIVATE
        PLUGIN_NAME="AStarPlanner"
        PLUGIN_VERSION="1.0.0")

# ========== 安装配置（可选）==========
if(INSTALL_PLUGINS)
    install(TARGETS astar_planner_plugin
        EXPORT AStarPlannerPluginTargets
        LIBRARY DESTINATION lib/navsim_plugins
        ARCHIVE DESTINATION lib/navsim_plugins)

    install(DIRECTORY include/
        DESTINATION include/navsim_plugins/astar
        FILES_MATCHING PATTERN "*.hpp")

    install(EXPORT AStarPlannerPluginTargets
        FILE AStarPlannerPluginTargets.cmake
        NAMESPACE NavSimPlugin::
        DESTINATION lib/cmake/NavSim)
endif()

message(STATUS "    A* planner plugin configured")


# StraightLine Planner Plugin
#
# A simple geometric planner that generates straight-line trajectories.

# ========== 插件库 ==========
# 编译为动态库（共享库），支持运行时加载
add_library(straight_line_planner_plugin SHARED
    src/straight_line_planner_plugin.cpp
    src/register.cpp)

# 设置动态库输出名称和版本
set_target_properties(straight_line_planner_plugin PROPERTIES
    OUTPUT_NAME "straight_line_planner_plugin"
    VERSION 1.0.0
    SOVERSION 1)

# ========== 包含目录 ==========
target_include_directories(straight_line_planner_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/navsim_plugins/straight_line>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ========== 依赖项 ==========
target_link_libraries(straight_line_planner_plugin
    PUBLIC
        navsim_plugin_framework)

# ========== 编译选项 ==========
target_compile_features(straight_line_planner_plugin PUBLIC cxx_std_17)

# 定义宏，让 plugin_init.cpp 知道这个插件已启用
target_compile_definitions(straight_line_planner_plugin PUBLIC BUILD_STRAIGHT_LINE_PLANNER_PLUGIN)

target_compile_definitions(straight_line_planner_plugin
    PRIVATE
        PLUGIN_NAME="StraightLinePlanner"
        PLUGIN_VERSION="1.0.0")

# ========== 安装配置（可选）==========
if(INSTALL_PLUGINS)
    install(TARGETS straight_line_planner_plugin
        EXPORT StraightLinePlannerPluginTargets
        LIBRARY DESTINATION lib/navsim_plugins
        ARCHIVE DESTINATION lib/navsim_plugins)

    install(DIRECTORY include/
        DESTINATION include/navsim_plugins/straight_line
        FILES_MATCHING PATTERN "*.hpp")

    install(EXPORT StraightLinePlannerPluginTargets
        FILE StraightLinePlannerPluginTargets.cmake
        NAMESPACE NavSimPlugin::
        DESTINATION lib/cmake/NavSim)
endif()

message(STATUS "    StraightLine planner plugin configured")


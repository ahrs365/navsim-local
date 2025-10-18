# NavSim Planning Plugins
#
# Each planning plugin is an independent sub-project.
# Users can enable/disable specific plugins.

message(STATUS "--- Configuring Planning Plugins ---")

# ========== 插件选项 ==========
option(BUILD_STRAIGHT_LINE_PLANNER_PLUGIN "Build StraightLine planner" ON)
option(BUILD_JPS_PLANNER_PLUGIN "Build JPS planner" ON)

# ========== 插件列表 ==========
set(PLANNING_PLUGIN_LIBS "")

# StraightLine 规划器
if(BUILD_STRAIGHT_LINE_PLANNER_PLUGIN)
    message(STATUS "  [+] StraightLine planner plugin")
    add_subdirectory(straight_line_planner)
    list(APPEND PLANNING_PLUGIN_LIBS straight_line_plugin)
else()
    message(STATUS "  [-] StraightLine planner plugin (disabled)")
endif()

# JPS 规划器
if(BUILD_JPS_PLANNER_PLUGIN)
    message(STATUS "  [+] JPS planner plugin")
    add_subdirectory(jps_planner_plugin)
    list(APPEND PLANNING_PLUGIN_LIBS jps_planner_plugin)
else()
    message(STATUS "  [-] JPS planner plugin (disabled)")
endif()

# ========== 创建规划器插件聚合库 ==========
add_library(navsim_planning_plugins INTERFACE)

if(PLANNING_PLUGIN_LIBS)
    target_link_libraries(navsim_planning_plugins INTERFACE
        ${PLANNING_PLUGIN_LIBS})
    message(STATUS "Planning plugins: ${PLANNING_PLUGIN_LIBS}")
else()
    message(STATUS "No planning plugins enabled")
endif()

message(STATUS "--- Planning Plugins Configuration Complete ---")


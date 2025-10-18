# NavSim Planning Plugins
#
# Each planning plugin is an independent sub-project.
# Users can enable/disable specific plugins.

message(STATUS "--- Configuring Planning Plugins ---")

# ========== 插件选项 ==========
option(BUILD_STRAIGHT_LINE_PLANNER_PLUGIN "Build StraightLine planner" ON)
option(BUILD_ASTAR_PLANNER_PLUGIN "Build A* planner" ON)
option(BUILD_JPS_PLANNER_PLUGIN "Build JPS planner" ON)
option(BUILD_RRT_STAR_PLANNER_PLUGIN "Build RRT* planner" OFF)

# ========== 插件列表 ==========
set(PLANNING_PLUGIN_LIBS "")

# StraightLine 规划器
if(BUILD_STRAIGHT_LINE_PLANNER_PLUGIN)
    message(STATUS "  [+] StraightLine planner plugin")
    add_subdirectory(straight_line)
    list(APPEND PLANNING_PLUGIN_LIBS straight_line_planner_plugin)
else()
    message(STATUS "  [-] StraightLine planner plugin (disabled)")
endif()

# A* 规划器
if(BUILD_ASTAR_PLANNER_PLUGIN)
    message(STATUS "  [+] A* planner plugin")
    add_subdirectory(astar)
    list(APPEND PLANNING_PLUGIN_LIBS astar_planner_plugin)
else()
    message(STATUS "  [-] A* planner plugin (disabled)")
endif()

# JPS 规划器
if(BUILD_JPS_PLANNER_PLUGIN)
    message(STATUS "  [+] JPS planner plugin")
    add_subdirectory(jps_planner_plugin)
    list(APPEND PLANNING_PLUGIN_LIBS jps_planner_plugin)
else()
    message(STATUS "  [-] JPS planner plugin (disabled)")
endif()

# RRT* 规划器（示例 - 未实现）
if(BUILD_RRT_STAR_PLANNER_PLUGIN)
    message(STATUS "  [+] RRT* planner plugin")
    # add_subdirectory(rrt_star)
    # list(APPEND PLANNING_PLUGIN_LIBS rrt_star_planner_plugin)
    message(WARNING "RRT* planner plugin not implemented yet")
else()
    message(STATUS "  [-] RRT* planner plugin (disabled)")
endif()

# Test 规划器 (验证插件脚手架工具)
option(BUILD_TEST_PLANNER_PLUGIN "Build Test planner (validation)" ON)
if(BUILD_TEST_PLANNER_PLUGIN)
    message(STATUS "  [+] Test planner plugin (validation)")
    add_subdirectory(test_planner)
    list(APPEND PLANNING_PLUGIN_LIBS test_planner_plugin)
else()
    message(STATUS "  [-] Test planner plugin (disabled)")
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


# TMPCPlanner Planner Plugin
#
# Topology-based Model Predictive Control (T-MPC) planner

# ========== T-MPC Algorithm Library ==========
# First, build the T-MPC algorithm as a subdirectory
add_subdirectory(algorithm ${CMAKE_CURRENT_BINARY_DIR}/t_mpc_algorithm)

# ========== 插件库 ==========
# 编译为动态库（共享库），支持运行时加载
add_library(tmpc_planner_plugin SHARED
    adapter/tmpc_planner_plugin.cpp
    adapter/register.cpp)

# 设置动态库输出名称和版本
set_target_properties(tmpc_planner_plugin PROPERTIES
    OUTPUT_NAME "tmpc_planner_plugin"
    VERSION 1.0.0
    SOVERSION 1)

# ========== 包含目录 ==========
target_include_directories(tmpc_planner_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/navsim_plugins/tmpc_planner>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/adapter
        ${CMAKE_SOURCE_DIR}/platform/include  # Platform headers (required)
        # T-MPC algorithm headers
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/mpc_planner/include
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/mpc_planner_types/include
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/mpc_planner_solver/include
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/mpc_planner_util/include
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/mpc_planner_modules/include
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/guidance_planner/include
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/ros_tools_no_ros/include
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm/DecompUtil/include
)

# ========== 依赖项 ==========
target_link_libraries(tmpc_planner_plugin
    PUBLIC
        navsim_plugin_framework
    PRIVATE
        Eigen3::Eigen
        yaml-cpp
        # T-MPC algorithm libraries
        mpc_planner
        mpc_planner_types
        mpc_planner_solver
        mpc_planner_util
        mpc_planner_modules
        guidance_planner
        ros_tools_no_ros
        decomp_util
)

# ========== 编译选项 ==========
target_compile_features(tmpc_planner_plugin PUBLIC cxx_std_17)

# 定义宏
target_compile_definitions(tmpc_planner_plugin
    PRIVATE
        PLUGIN_NAME="TMPCPlanner"
        PLUGIN_VERSION="1.0.0")

# ========== 安装配置（可选）==========
if(INSTALL_PLUGINS)
    install(TARGETS tmpc_planner_plugin
        EXPORT TMPCPlannerPluginTargets
        LIBRARY DESTINATION lib/navsim_plugins
        ARCHIVE DESTINATION lib/navsim_plugins)

    install(DIRECTORY adapter/
        DESTINATION include/navsim_plugins/tmpc_planner
        FILES_MATCHING PATTERN "*.hpp")

    install(EXPORT TMPCPlannerPluginTargets
        FILE TMPCPlannerPluginTargets.cmake
        NAMESPACE NavSimPlugin::
        DESTINATION lib/cmake/NavSim)
endif()

message(STATUS "    TMPCPlanner planner plugin configured")


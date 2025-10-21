# JpsPlanner Planner Plugin
#
# Jump Point Search path planner with ESDF map

# ========== 插件库 ==========
# 编译为动态库（共享库），支持运行时加载
add_library(jps_planner_plugin SHARED
    algorithm/jps_planner.cpp
    algorithm/graph_search.cpp
    algorithm/opt/optimizer.cpp
    adapter/jps_planner_plugin.cpp
    adapter/register.cpp)

# 设置动态库输出名称和版本
set_target_properties(jps_planner_plugin PROPERTIES
    OUTPUT_NAME "jps_planner_plugin"
    VERSION 1.0.0
    SOVERSION 1)

# ========== 包含目录 ==========
target_include_directories(jps_planner_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/navsim_plugins/jps_planner>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
        ${CMAKE_CURRENT_SOURCE_DIR}/adapter
        ${CMAKE_SOURCE_DIR}/platform/include  # Platform headers (required)
        ${CMAKE_SOURCE_DIR}/plugins/perception/esdf_builder/include  # ESDF map (required for JPS)
)

# ========== 依赖项 ==========
# Find Boost (required for JPS)
find_package(Boost REQUIRED)

target_link_libraries(jps_planner_plugin
    PUBLIC
        navsim_plugin_framework
    PRIVATE
        Eigen3::Eigen
        Boost::boost
        esdf_builder_plugin  # Required for ESDF map
)

# ========== 编译选项 ==========
target_compile_features(jps_planner_plugin PUBLIC cxx_std_17)

# 定义宏
target_compile_definitions(jps_planner_plugin
    PRIVATE
        PLUGIN_NAME="JpsPlanner"
        PLUGIN_VERSION="1.0.0")

# ========== 安装配置（可选）==========
if(INSTALL_PLUGINS)
    install(TARGETS jps_planner_plugin
        EXPORT JpsPlannerPluginTargets
        LIBRARY DESTINATION lib/navsim_plugins
        ARCHIVE DESTINATION lib/navsim_plugins)

    install(DIRECTORY algorithm/ adapter/
        DESTINATION include/navsim_plugins/jps_planner
        FILES_MATCHING PATTERN "*.hpp")

    install(EXPORT JpsPlannerPluginTargets
        FILE JpsPlannerPluginTargets.cmake
        NAMESPACE NavSimPlugin::
        DESTINATION lib/cmake/NavSim)
endif()

message(STATUS "    JpsPlanner planner plugin configured")


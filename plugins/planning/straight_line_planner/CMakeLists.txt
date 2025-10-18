# StraightLine Planner Plugin
#
# Simple straight-line path planner

# ========== 插件库 ==========
# 编译为动态库（共享库），支持运行时加载
add_library(straight_line_plugin SHARED
    algorithm/straight_line.cpp
    adapter/straight_line_plugin.cpp
    adapter/register.cpp)

# 设置动态库输出名称和版本
set_target_properties(straight_line_plugin PROPERTIES
    OUTPUT_NAME "straight_line_plugin"
    VERSION 1.0.0
    SOVERSION 1)

# ========== 包含目录 ==========
target_include_directories(straight_line_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/navsim_plugins/straight_line>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
        ${CMAKE_CURRENT_SOURCE_DIR}/adapter
        ${CMAKE_SOURCE_DIR}/platform/include  # Platform headers (required)
        # ${CMAKE_SOURCE_DIR}/plugins/perception/esdf_builder/include  # Uncomment if you need ESDF map
        # ${CMAKE_SOURCE_DIR}/plugins/perception/grid_map_builder/include  # Uncomment if you need occupancy grid
)

# ========== 依赖项 ==========
# Uncomment if you need additional dependencies:
# find_package(Boost REQUIRED)              # For advanced data structures (heap, graph, etc.)
# find_package(OpenCV REQUIRED)             # For image processing
# find_package(PCL REQUIRED)                # For point cloud processing

target_link_libraries(straight_line_plugin
    PUBLIC
        navsim_plugin_framework
    PRIVATE
        Eigen3::Eigen
        # Boost::boost                        # Uncomment if using Boost
        # ${OpenCV_LIBS}                      # Uncomment if using OpenCV
        # ${PCL_LIBRARIES}                    # Uncomment if using PCL
        # esdf_builder_plugin                 # Uncomment if you need ESDF map
        # grid_map_builder_plugin             # Uncomment if you need occupancy grid
)

# ========== 编译选项 ==========
target_compile_features(straight_line_plugin PUBLIC cxx_std_17)

# 定义宏
target_compile_definitions(straight_line_plugin
    PRIVATE
        PLUGIN_NAME="StraightLine"
        PLUGIN_VERSION="1.0.0")

# ========== 安装配置（可选）==========
if(INSTALL_PLUGINS)
    install(TARGETS straight_line_plugin
        EXPORT StraightLinePluginTargets
        LIBRARY DESTINATION lib/navsim_plugins
        ARCHIVE DESTINATION lib/navsim_plugins)

    install(DIRECTORY algorithm/ adapter/
        DESTINATION include/navsim_plugins/straight_line
        FILES_MATCHING PATTERN "*.hpp")

    install(EXPORT StraightLinePluginTargets
        FILE StraightLinePluginTargets.cmake
        NAMESPACE NavSimPlugin::
        DESTINATION lib/cmake/NavSim)
endif()

message(STATUS "    StraightLine planner plugin configured")


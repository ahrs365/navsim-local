# {{PLUGIN_NAME}} Planner Plugin
#
# {{DESCRIPTION}}

# ========== 插件库 ==========
# 编译为动态库（共享库），支持运行时加载
add_library({{PLUGIN_NAME_SNAKE}}_plugin SHARED
    algorithm/{{PLUGIN_NAME_SNAKE}}.cpp
    adapter/{{PLUGIN_NAME_SNAKE}}_plugin.cpp
    adapter/register.cpp)

# 设置动态库输出名称和版本
set_target_properties({{PLUGIN_NAME_SNAKE}}_plugin PROPERTIES
    OUTPUT_NAME "{{PLUGIN_NAME_SNAKE}}_plugin"
    VERSION 1.0.0
    SOVERSION 1)

# ========== 包含目录 ==========
target_include_directories({{PLUGIN_NAME_SNAKE}}_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/navsim_plugins/{{PLUGIN_NAME_SNAKE}}>
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

target_link_libraries({{PLUGIN_NAME_SNAKE}}_plugin
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
target_compile_features({{PLUGIN_NAME_SNAKE}}_plugin PUBLIC cxx_std_17)

# 定义宏
target_compile_definitions({{PLUGIN_NAME_SNAKE}}_plugin
    PRIVATE
        PLUGIN_NAME="{{PLUGIN_NAME}}"
        PLUGIN_VERSION="1.0.0")

# ========== 安装配置（可选）==========
if(INSTALL_PLUGINS)
    install(TARGETS {{PLUGIN_NAME_SNAKE}}_plugin
        EXPORT {{PLUGIN_NAME}}PluginTargets
        LIBRARY DESTINATION lib/navsim_plugins
        ARCHIVE DESTINATION lib/navsim_plugins)

    install(DIRECTORY algorithm/ adapter/
        DESTINATION include/navsim_plugins/{{PLUGIN_NAME_SNAKE}}
        FILES_MATCHING PATTERN "*.hpp")

    install(EXPORT {{PLUGIN_NAME}}PluginTargets
        FILE {{PLUGIN_NAME}}PluginTargets.cmake
        NAMESPACE NavSimPlugin::
        DESTINATION lib/cmake/NavSim)
endif()

message(STATUS "    {{PLUGIN_NAME}} planner plugin configured")


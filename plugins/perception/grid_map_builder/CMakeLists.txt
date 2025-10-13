# GridMapBuilder Perception Plugin
#
# This plugin builds an occupancy grid map from BEV obstacles.

# ========== 插件库 ==========
# 编译为动态库（共享库），支持运行时加载
add_library(grid_map_builder_plugin SHARED
    src/grid_map_builder_plugin.cpp
    src/register.cpp)

# 设置动态库输出名称和版本
set_target_properties(grid_map_builder_plugin PROPERTIES
    OUTPUT_NAME "grid_map_builder_plugin"
    VERSION 1.0.0
    SOVERSION 1)

# ========== 包含目录 ==========
target_include_directories(grid_map_builder_plugin
    PUBLIC
        # 构建时使用当前目录的 include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        # 安装后使用安装目录
        $<INSTALL_INTERFACE:include/navsim_plugins/grid_map_builder>
    PRIVATE
        # 插件内部使用
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ========== 依赖项 ==========
# 链接到插件框架（提供接口和数据结构）
target_link_libraries(grid_map_builder_plugin
    PUBLIC
        navsim_plugin_framework
    PRIVATE
        Eigen3::Eigen)

# ========== 编译选项 ==========
target_compile_features(grid_map_builder_plugin PUBLIC cxx_std_17)

# 定义宏，让 plugin_init.cpp 知道这个插件已启用
target_compile_definitions(grid_map_builder_plugin PUBLIC BUILD_GRID_MAP_BUILDER_PLUGIN)

# 添加编译定义（可选）
target_compile_definitions(grid_map_builder_plugin
    PRIVATE
        PLUGIN_NAME="GridMapBuilder"
        PLUGIN_VERSION="1.0.0")

# ========== 安装配置（可选）==========
# 如果需要独立分发此插件，可以启用安装
if(INSTALL_PLUGINS)
    install(TARGETS grid_map_builder_plugin
        EXPORT GridMapBuilderPluginTargets
        LIBRARY DESTINATION lib/navsim_plugins
        ARCHIVE DESTINATION lib/navsim_plugins
        RUNTIME DESTINATION bin)

    install(DIRECTORY include/
        DESTINATION include/navsim_plugins/grid_map_builder
        FILES_MATCHING PATTERN "*.hpp")

    install(EXPORT GridMapBuilderPluginTargets
        FILE GridMapBuilderPluginTargets.cmake
        NAMESPACE NavSimPlugin::
        DESTINATION lib/cmake/NavSim)
endif()

message(STATUS "    GridMapBuilder plugin configured")


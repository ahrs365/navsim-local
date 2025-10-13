#!/bin/bash

# 更新所有 #include 路径的脚本

echo "Updating #include paths..."

# 查找所有 .hpp 和 .cpp 文件
FILES=$(find include src tests -type f \( -name "*.hpp" -o -name "*.cpp" \))

for file in $FILES; do
  # 核心模块
  sed -i 's|#include "algorithm_manager\.hpp"|#include "core/algorithm_manager.hpp"|g' "$file"
  sed -i 's|#include "bridge\.hpp"|#include "core/bridge.hpp"|g' "$file"
  sed -i 's|#include "planning_context\.hpp"|#include "core/planning_context.hpp"|g' "$file"
  sed -i 's|#include "websocket_visualizer\.hpp"|#include "core/websocket_visualizer.hpp"|g' "$file"
  
  # 插件框架
  sed -i 's|#include "plugin/perception_plugin_interface\.hpp"|#include "plugin/framework/perception_plugin_interface.hpp"|g' "$file"
  sed -i 's|#include "plugin/planner_plugin_interface\.hpp"|#include "plugin/framework/planner_plugin_interface.hpp"|g' "$file"
  sed -i 's|#include "plugin/plugin_metadata\.hpp"|#include "plugin/framework/plugin_metadata.hpp"|g' "$file"
  sed -i 's|#include "plugin/plugin_registry\.hpp"|#include "plugin/framework/plugin_registry.hpp"|g' "$file"
  sed -i 's|#include "plugin/perception_plugin_manager\.hpp"|#include "plugin/framework/perception_plugin_manager.hpp"|g' "$file"
  sed -i 's|#include "plugin/planner_plugin_manager\.hpp"|#include "plugin/framework/planner_plugin_manager.hpp"|g' "$file"
  sed -i 's|#include "plugin/config_loader\.hpp"|#include "plugin/framework/config_loader.hpp"|g' "$file"
  sed -i 's|#include "plugin/plugin_init\.hpp"|#include "plugin/framework/plugin_init.hpp"|g' "$file"
  
  # 数据结构
  sed -i 's|#include "plugin/perception_input\.hpp"|#include "plugin/data/perception_input.hpp"|g' "$file"
  sed -i 's|#include "plugin/planning_result\.hpp"|#include "plugin/data/planning_result.hpp"|g' "$file"
  
  # 前置处理
  sed -i 's|#include "perception/preprocessing\.hpp"|#include "plugin/preprocessing/preprocessing.hpp"|g' "$file"
  
  # 具体插件（相对路径）
  sed -i 's|#include "grid_map_builder_plugin\.hpp"|#include "plugin/plugins/perception/grid_map_builder_plugin.hpp"|g' "$file"
  sed -i 's|#include "straight_line_planner_plugin\.hpp"|#include "plugin/plugins/planning/straight_line_planner_plugin.hpp"|g' "$file"
  sed -i 's|#include "astar_planner_plugin\.hpp"|#include "plugin/plugins/planning/astar_planner_plugin.hpp"|g' "$file"
done

echo "Include paths updated successfully!"


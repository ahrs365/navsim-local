#pragma once

#include <string>
#include <vector>

namespace navsim {
namespace plugin {

/**
 * @brief 插件元数据
 * 
 * 描述插件的基本信息，用于插件管理和调试。
 */
struct PluginMetadata {
  /**
   * @brief 插件名称
   * 必须唯一，用于插件注册和配置
   */
  std::string name;
  
  /**
   * @brief 插件版本
   * 格式：major.minor.patch (例如: "1.0.0")
   */
  std::string version;
  
  /**
   * @brief 插件描述
   * 简要说明插件的功能
   */
  std::string description;
  
  /**
   * @brief 插件作者
   */
  std::string author;
  
  /**
   * @brief 插件依赖
   * 列出此插件依赖的其他插件名称
   */
  std::vector<std::string> dependencies;
  
  /**
   * @brief 默认构造函数
   */
  PluginMetadata() = default;
  
  /**
   * @brief 完整构造函数
   */
  PluginMetadata(
      const std::string& name_,
      const std::string& version_,
      const std::string& description_,
      const std::string& author_ = "",
      const std::vector<std::string>& dependencies_ = {})
      : name(name_),
        version(version_),
        description(description_),
        author(author_),
        dependencies(dependencies_) {}
};

/**
 * @brief 感知插件元数据
 * 
 * 扩展基础元数据，添加感知插件特定的信息。
 */
struct PerceptionPluginMetadata : public PluginMetadata {
  /**
   * @brief 是否需要访问原始数据
   * 如果为 true，插件可能需要访问 PerceptionInput::raw_world_tick
   */
  bool requires_raw_data = false;
  
  /**
   * @brief 输出数据类型
   * 描述插件输出到 PlanningContext 的数据类型
   * 例如: "occupancy_grid", "esdf_map", "point_cloud_map"
   */
  std::vector<std::string> output_data_types;
  
  /**
   * @brief 默认构造函数
   */
  PerceptionPluginMetadata() = default;
  
  /**
   * @brief 完整构造函数
   */
  PerceptionPluginMetadata(
      const std::string& name_,
      const std::string& version_,
      const std::string& description_,
      const std::string& author_ = "",
      const std::vector<std::string>& dependencies_ = {},
      bool requires_raw_data_ = false,
      const std::vector<std::string>& output_data_types_ = {})
      : PluginMetadata(name_, version_, description_, author_, dependencies_),
        requires_raw_data(requires_raw_data_),
        output_data_types(output_data_types_) {}
};

/**
 * @brief 规划器插件元数据
 * 
 * 扩展基础元数据，添加规划器插件特定的信息。
 */
struct PlannerPluginMetadata : public PluginMetadata {
  /**
   * @brief 规划器类型
   * 例如: "geometric", "search", "optimization", "learning"
   */
  std::string type;
  
  /**
   * @brief 必需的感知数据
   * 列出规划器必需的感知数据类型
   * 例如: "occupancy_grid", "bev_obstacles", "lane_lines"
   */
  std::vector<std::string> required_perception_data;
  
  /**
   * @brief 是否可以作为降级规划器
   * 降级规划器应该简单、快速、可靠
   */
  bool can_be_fallback = false;
  
  /**
   * @brief 默认构造函数
   */
  PlannerPluginMetadata() = default;
  
  /**
   * @brief 完整构造函数
   */
  PlannerPluginMetadata(
      const std::string& name_,
      const std::string& version_,
      const std::string& description_,
      const std::string& type_,
      const std::string& author_ = "",
      const std::vector<std::string>& dependencies_ = {},
      const std::vector<std::string>& required_perception_data_ = {},
      bool can_be_fallback_ = false)
      : PluginMetadata(name_, version_, description_, author_, dependencies_),
        type(type_),
        required_perception_data(required_perception_data_),
        can_be_fallback(can_be_fallback_) {}
};

} // namespace plugin
} // namespace navsim


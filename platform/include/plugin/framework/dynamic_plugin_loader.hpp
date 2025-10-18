#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

namespace navsim {
namespace plugin {

// 前向声明
class ConfigLoader;

/**
 * @brief 动态插件加载器
 * 
 * 负责在运行时动态加载插件共享库（.so 文件）
 */
class DynamicPluginLoader {
public:
  /**
   * @brief 插件信息
   */
  struct PluginInfo {
    std::string name;           // 插件名称
    std::string library_path;   // 库文件路径
    void* handle;               // dlopen 返回的句柄
    bool loaded;                // 是否已加载
    
    PluginInfo() : handle(nullptr), loaded(false) {}
  };

  DynamicPluginLoader();
  ~DynamicPluginLoader();

  // 禁止拷贝
  DynamicPluginLoader(const DynamicPluginLoader&) = delete;
  DynamicPluginLoader& operator=(const DynamicPluginLoader&) = delete;

  /**
   * @brief 从配置文件加载插件列表
   * 
   * @param config_path 配置文件路径
   * @return 成功加载的插件数量
   */
  int loadPluginsFromConfig(const std::string& config_path);

  /**
   * @brief 加载单个插件
   * 
   * @param plugin_name 插件名称（如 "GridMapBuilder"）
   * @param library_path 库文件路径（如 "plugins/libgrid_map_builder_plugin.so"）
   * @return 是否成功加载
   */
  bool loadPlugin(const std::string& plugin_name, const std::string& library_path);

  /**
   * @brief 卸载单个插件
   * 
   * @param plugin_name 插件名称
   * @return 是否成功卸载
   */
  bool unloadPlugin(const std::string& plugin_name);

  /**
   * @brief 卸载所有插件
   */
  void unloadAllPlugins();

  /**
   * @brief 获取已加载的插件列表
   */
  std::vector<std::string> getLoadedPlugins() const;

  /**
   * @brief 检查插件是否已加载
   */
  bool isPluginLoaded(const std::string& plugin_name) const;

  /**
   * @brief 设置插件搜索路径
   * 
   * @param paths 搜索路径列表
   */
  void setSearchPaths(const std::vector<std::string>& paths);

  /**
   * @brief 添加插件搜索路径
   */
  void addSearchPath(const std::string& path);

  /**
   * @brief 查找插件库文件
   *
   * @param plugin_name 插件名称
   * @return 库文件的完整路径，如果未找到则返回空字符串
   */
  std::string findPluginLibrary(const std::string& plugin_name) const;

  /**
   * @brief 解析插件路径（支持短名称和完整路径）
   *
   * 如果 plugin_spec 包含 '/' 或以 '.so' 结尾，视为完整路径
   * 否则视为短名称，按以下顺序查找：
   * 1. plugins/planning/lib{name}.so
   * 2. plugins/perception/lib{name}.so
   * 3. ~/.navsim/plugins/lib{name}.so
   * 4. ./external_plugins/{name}/build/lib{name}.so
   * 5. $NAVSIM_PLUGIN_PATH/lib{name}.so
   *
   * @param plugin_spec 插件规格（短名称或完整路径）
   * @return 解析后的完整路径，如果未找到则返回空字符串
   */
  std::string resolvePluginPath(const std::string& plugin_spec) const;

  /**
   * @brief 获取配置加载器
   *
   * @return 配置加载器指针，如果未加载配置则返回 nullptr
   */
  const ConfigLoader* getConfigLoader() const { return config_loader_.get(); }

private:
  // 插件信息映射表
  std::unordered_map<std::string, PluginInfo> plugins_;

  // 插件搜索路径
  std::vector<std::string> search_paths_;

  // 配置加载器
  std::unique_ptr<ConfigLoader> config_loader_;

  /**
   * @brief 初始化默认搜索路径
   */
  void initializeDefaultSearchPaths();

  /**
   * @brief 解析插件名称到库文件名
   *
   * 例如: "GridMapBuilder" -> "libgrid_map_builder_plugin.so"
   */
  std::string pluginNameToLibraryName(const std::string& plugin_name) const;
};

} // namespace plugin
} // namespace navsim


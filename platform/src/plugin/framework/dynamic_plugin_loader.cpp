#include "plugin/framework/dynamic_plugin_loader.hpp"
#include "plugin/framework/config_loader.hpp"
#include <dlfcn.h>
#include <unistd.h>  // for readlink
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <cctype>
#include <cstdlib>  // for getenv

namespace fs = std::filesystem;

namespace navsim {
namespace plugin {

DynamicPluginLoader::DynamicPluginLoader() {
  initializeDefaultSearchPaths();
}

DynamicPluginLoader::~DynamicPluginLoader() {
  unloadAllPlugins();
}

void DynamicPluginLoader::initializeDefaultSearchPaths() {
  // 按照 REFACTORING_PROPOSAL.md 中定义的顺序添加搜索路径

  // 1. 平台内置插件目录
  search_paths_.push_back("./plugins/planning");
  search_paths_.push_back("./plugins/perception");
  search_paths_.push_back("./build/plugins/planning");
  search_paths_.push_back("./build/plugins/perception");

  // 2. 用户插件目录 - ~/.navsim/plugins/
  const char* home = getenv("HOME");
  if (home) {
    fs::path user_plugin_dir = fs::path(home) / ".navsim" / "plugins";
    search_paths_.push_back(user_plugin_dir.string());
  }

  // 3. 外部插件目录 - ./external_plugins/
  search_paths_.push_back("./external_plugins");

  // 4. 环境变量 - $NAVSIM_PLUGIN_PATH
  const char* env_path = getenv("NAVSIM_PLUGIN_PATH");
  if (env_path) {
    search_paths_.push_back(env_path);
  }

  // 5. 系统插件目录
  search_paths_.push_back("/usr/local/lib/navsim_plugins");
  search_paths_.push_back("/usr/lib/navsim_plugins");

  // 6. 添加当前可执行文件所在目录的 plugins 子目录
  char exe_path[1024];
  ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
  if (len != -1) {
    exe_path[len] = '\0';
    fs::path exe_dir = fs::path(exe_path).parent_path();
    search_paths_.push_back((exe_dir / "plugins").string());
  }
}

void DynamicPluginLoader::setSearchPaths(const std::vector<std::string>& paths) {
  search_paths_ = paths;
}

void DynamicPluginLoader::addSearchPath(const std::string& path) {
  search_paths_.push_back(path);
}

std::string DynamicPluginLoader::pluginNameToLibraryName(const std::string& plugin_name) const {
  // 将驼峰命名转换为下划线命名
  // GridMapBuilder -> grid_map_builder
  std::string snake_case;
  for (size_t i = 0; i < plugin_name.size(); ++i) {
    char c = plugin_name[i];
    if (std::isupper(c) && i > 0) {
      snake_case += '_';
    }
    snake_case += std::tolower(c);
  }
  
  // 添加前缀和后缀
  // grid_map_builder -> libgrid_map_builder_plugin.so
  return "lib" + snake_case + "_plugin.so";
}

std::string DynamicPluginLoader::findPluginLibrary(const std::string& plugin_name) const {
  std::string lib_name = pluginNameToLibraryName(plugin_name);

  // 在搜索路径中查找
  for (const auto& search_path : search_paths_) {
    // 尝试直接路径
    fs::path direct_path = fs::path(search_path) / lib_name;
    if (fs::exists(direct_path)) {
      return direct_path.string();
    }

    // 尝试在子目录中查找
    // 例如: plugins/perception/grid_map_builder/libgrid_map_builder_plugin.so
    try {
      for (const auto& entry : fs::recursive_directory_iterator(search_path)) {
        if (entry.is_regular_file() && entry.path().filename() == lib_name) {
          return entry.path().string();
        }
      }
    } catch (const fs::filesystem_error& e) {
      // 忽略无法访问的目录
      continue;
    }
  }

  return "";  // 未找到
}

std::string DynamicPluginLoader::resolvePluginPath(const std::string& plugin_spec) const {
  // 判断是否为完整路径
  // 如果包含 '/' 或以 '.so' 结尾，视为完整路径
  if (plugin_spec.find('/') != std::string::npos ||
      plugin_spec.size() >= 3 && plugin_spec.substr(plugin_spec.size() - 3) == ".so") {
    // 完整路径 - 直接检查是否存在
    if (fs::exists(plugin_spec)) {
      return fs::absolute(plugin_spec).string();
    } else {
      std::cerr << "[DynamicPluginLoader] Plugin file not found: " << plugin_spec << std::endl;
      return "";
    }
  }

  // 短名称 - 按照定义的顺序查找
  std::string lib_name = pluginNameToLibraryName(plugin_spec);

  // 定义查找路径（按优先级排序）
  std::vector<std::string> search_order;

  // 1. plugins/planning/lib{name}.so
  search_order.push_back("plugins/planning/" + lib_name);
  search_order.push_back("build/plugins/planning/" + lib_name);

  // 2. plugins/perception/lib{name}.so
  search_order.push_back("plugins/perception/" + lib_name);
  search_order.push_back("build/plugins/perception/" + lib_name);

  // 3. ~/.navsim/plugins/lib{name}.so
  const char* home = getenv("HOME");
  if (home) {
    fs::path user_plugin_path = fs::path(home) / ".navsim" / "plugins" / lib_name;
    search_order.push_back(user_plugin_path.string());
  }

  // 4. ./external_plugins/{name}/build/lib{name}.so
  // 从短名称提取插件名（去掉 Plugin 后缀）
  std::string plugin_dir_name = plugin_spec;
  if (plugin_dir_name.size() > 6 && plugin_dir_name.substr(plugin_dir_name.size() - 6) == "Plugin") {
    plugin_dir_name = plugin_dir_name.substr(0, plugin_dir_name.size() - 6);
  }
  // 转换为 snake_case
  std::string snake_case;
  for (size_t i = 0; i < plugin_dir_name.size(); ++i) {
    char c = plugin_dir_name[i];
    if (std::isupper(c) && i > 0) {
      snake_case += '_';
    }
    snake_case += std::tolower(c);
  }
  search_order.push_back("external_plugins/" + snake_case + "/build/" + lib_name);

  // 5. $NAVSIM_PLUGIN_PATH/lib{name}.so
  const char* env_path = getenv("NAVSIM_PLUGIN_PATH");
  if (env_path) {
    fs::path env_plugin_path = fs::path(env_path) / lib_name;
    search_order.push_back(env_plugin_path.string());
  }

  // 按顺序查找
  for (const auto& path : search_order) {
    if (fs::exists(path)) {
      std::cout << "[DynamicPluginLoader] Found plugin '" << plugin_spec
                << "' at: " << path << std::endl;
      return fs::absolute(path).string();
    }
  }

  // 未找到 - 打印搜索路径
  std::cerr << "[DynamicPluginLoader] Failed to find plugin: " << plugin_spec << std::endl;
  std::cerr << "[DynamicPluginLoader] Searched in:" << std::endl;
  for (const auto& path : search_order) {
    std::cerr << "  - " << path << std::endl;
  }

  return "";
}

bool DynamicPluginLoader::loadPlugin(const std::string& plugin_name, const std::string& library_path) {
  // 检查是否已加载
  if (isPluginLoaded(plugin_name)) {
    std::cout << "[DynamicPluginLoader] Plugin '" << plugin_name << "' is already loaded" << std::endl;
    return true;
  }

  // 确定库文件路径
  std::string lib_path = library_path;
  if (lib_path.empty()) {
    // 使用新的 resolvePluginPath() 方法，支持短名称和完整路径
    lib_path = resolvePluginPath(plugin_name);
    if (lib_path.empty()) {
      // resolvePluginPath() 已经打印了详细的错误信息
      return false;
    }
  } else {
    // 如果提供了路径，也使用 resolvePluginPath() 进行解析
    lib_path = resolvePluginPath(library_path);
    if (lib_path.empty()) {
      return false;
    }
  }
  
  std::cout << "[DynamicPluginLoader] Loading plugin '" << plugin_name << "' from: " << lib_path << std::endl;
  
  // 使用 dlopen 加载动态库
  // RTLD_NOW: 立即解析所有符号
  // RTLD_GLOBAL: 使符号对后续加载的库可用
  void* handle = dlopen(lib_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
  
  if (!handle) {
    std::cerr << "[DynamicPluginLoader] Failed to load plugin: " << dlerror() << std::endl;
    return false;
  }
  
  // 查找注册函数
  // 例如: registerGridMapBuilderPlugin
  std::string register_func_name = "register" + plugin_name + "Plugin";
  
  // 清除之前的错误
  dlerror();
  
  // 获取注册函数指针
  typedef void (*RegisterFunc)();
  RegisterFunc register_func = (RegisterFunc)dlsym(handle, register_func_name.c_str());
  
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    std::cerr << "[DynamicPluginLoader] Warning: Cannot find register function '" 
              << register_func_name << "': " << dlsym_error << std::endl;
    std::cerr << "[DynamicPluginLoader] Plugin may use static registration" << std::endl;
    // 不返回 false，因为插件可能使用静态注册
  } else if (register_func) {
    // 调用注册函数
    std::cout << "[DynamicPluginLoader] Calling registration function: " << register_func_name << std::endl;
    register_func();
  }
  
  // 保存插件信息
  PluginInfo info;
  info.name = plugin_name;
  info.library_path = lib_path;
  info.handle = handle;
  info.loaded = true;
  
  plugins_[plugin_name] = info;
  
  std::cout << "[DynamicPluginLoader] Successfully loaded plugin: " << plugin_name << std::endl;
  return true;
}

bool DynamicPluginLoader::unloadPlugin(const std::string& plugin_name) {
  auto it = plugins_.find(plugin_name);
  if (it == plugins_.end()) {
    std::cerr << "[DynamicPluginLoader] Plugin not found: " << plugin_name << std::endl;
    return false;
  }
  
  if (it->second.handle) {
    std::cout << "[DynamicPluginLoader] Unloading plugin: " << plugin_name << std::endl;
    dlclose(it->second.handle);
    it->second.handle = nullptr;
    it->second.loaded = false;
  }
  
  plugins_.erase(it);
  return true;
}

void DynamicPluginLoader::unloadAllPlugins() {
  std::cout << "[DynamicPluginLoader] Unloading all plugins..." << std::endl;
  
  for (auto& pair : plugins_) {
    if (pair.second.handle) {
      std::cout << "[DynamicPluginLoader] Unloading: " << pair.first << std::endl;
      dlclose(pair.second.handle);
    }
  }
  
  plugins_.clear();
}

std::vector<std::string> DynamicPluginLoader::getLoadedPlugins() const {
  std::vector<std::string> result;
  for (const auto& pair : plugins_) {
    if (pair.second.loaded) {
      result.push_back(pair.first);
    }
  }
  return result;
}

bool DynamicPluginLoader::isPluginLoaded(const std::string& plugin_name) const {
  auto it = plugins_.find(plugin_name);
  return it != plugins_.end() && it->second.loaded;
}

int DynamicPluginLoader::loadPluginsFromConfig(const std::string& config_path) {
  std::cout << "[DynamicPluginLoader] Loading plugins from config: " << config_path << std::endl;

  // 加载配置文件
  config_loader_ = std::make_unique<ConfigLoader>();
  if (!config_loader_->loadFromFile(config_path)) {
    std::cerr << "[DynamicPluginLoader] Failed to load config file: " << config_path << std::endl;
    config_loader_.reset();
    return 0;
  }

  int loaded_count = 0;

  // 加载感知插件
  auto perception_configs = config_loader_->getPerceptionPluginConfigs();
  std::cout << "[DynamicPluginLoader] Found " << perception_configs.size() << " perception plugins in config" << std::endl;
  for (const auto& config : perception_configs) {
    std::cout << "[DynamicPluginLoader] Perception plugin: " << config.name
              << " (enabled: " << config.enabled << ")" << std::endl;
    if (config.enabled) {
      if (loadPlugin(config.name, "")) {
        loaded_count++;
      }
    } else {
      std::cout << "[DynamicPluginLoader] Skipping disabled plugin: " << config.name << std::endl;
    }
  }

  // 加载规划器插件
  std::string primary_planner = config_loader_->getPrimaryPlannerName();
  std::string fallback_planner = config_loader_->getFallbackPlannerName();

  // 加载主规划器
  if (!primary_planner.empty()) {
    if (loadPlugin(primary_planner, "")) {
      loaded_count++;
    }
  }

  // 加载降级规划器（如果与主规划器不同）
  if (!fallback_planner.empty() && fallback_planner != primary_planner) {
    if (loadPlugin(fallback_planner, "")) {
      loaded_count++;
    }
  }
  
  std::cout << "[DynamicPluginLoader] Loaded " << loaded_count << " plugins from config" << std::endl;
  return loaded_count;
}

} // namespace plugin
} // namespace navsim


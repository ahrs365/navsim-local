#include "plugin/framework/config_loader.hpp"
#include <fstream>
#include <iostream>

namespace navsim {
namespace plugin {

bool ConfigLoader::loadFromFile(const std::string& config_file) {
  std::ifstream file(config_file);
  if (!file.is_open()) {
    std::cerr << "[ConfigLoader] Failed to open config file: " << config_file << std::endl;
    return false;
  }
  
  try {
    nlohmann::json json;
    file >> json;
    return loadFromJson(json);
  } catch (const std::exception& e) {
    std::cerr << "[ConfigLoader] Failed to parse config file: " << e.what() << std::endl;
    return false;
  }
}

bool ConfigLoader::loadFromString(const std::string& json_str) {
  try {
    nlohmann::json json = nlohmann::json::parse(json_str);
    return loadFromJson(json);
  } catch (const std::exception& e) {
    std::cerr << "[ConfigLoader] Failed to parse JSON string: " << e.what() << std::endl;
    return false;
  }
}

bool ConfigLoader::loadFromJson(const nlohmann::json& json) {
  config_ = json;
  
  // 验证配置
  auto [valid, error_msg] = validateConfig(json);
  if (!valid) {
    std::cerr << "[ConfigLoader] Invalid config: " << error_msg << std::endl;
    return false;
  }
  
  // 解析感知配置
  if (json.contains("perception")) {
    if (!parsePerceptionConfig(json["perception"])) {
      return false;
    }
  }

  // 支持旧格式：perception_plugins 直接在根级别
  if (json.contains("perception_plugins")) {
    const auto& plugins = json["perception_plugins"];
    if (plugins.is_array()) {
      perception_plugin_configs_.clear();
      for (const auto& plugin_json : plugins) {
        PerceptionPluginConfig config;
        if (!plugin_json.contains("name")) {
          std::cerr << "[ConfigLoader] Plugin config missing 'name' field" << std::endl;
          return false;
        }
        config.name = plugin_json["name"];
        if (plugin_json.contains("enabled")) {
          config.enabled = plugin_json["enabled"];
        }
        if (plugin_json.contains("priority")) {
          config.priority = plugin_json["priority"];
        }
        if (plugin_json.contains("params")) {
          config.params = plugin_json["params"];
        }
        perception_plugin_configs_.push_back(config);
      }
    }
  }
  
  // 解析规划配置
  if (json.contains("planning")) {
    if (!parsePlanningConfig(json["planning"])) {
      return false;
    }
  }
  
  std::cout << "[ConfigLoader] Config loaded successfully" << std::endl;
  return true;
}

bool ConfigLoader::saveToFile(const std::string& config_file) const {
  std::ofstream file(config_file);
  if (!file.is_open()) {
    std::cerr << "[ConfigLoader] Failed to open file for writing: " << config_file << std::endl;
    return false;
  }
  
  try {
    file << config_.dump(2);  // 缩进 2 个空格
    return true;
  } catch (const std::exception& e) {
    std::cerr << "[ConfigLoader] Failed to write config file: " << e.what() << std::endl;
    return false;
  }
}

bool ConfigLoader::parsePerceptionConfig(const nlohmann::json& perception_config) {
  // 解析前置处理配置
  if (perception_config.contains("preprocessing")) {
    preprocessing_config_ = perception_config["preprocessing"];
  }
  
  // 解析插件配置
  if (perception_config.contains("plugins")) {
    const auto& plugins = perception_config["plugins"];
    
    if (!plugins.is_array()) {
      std::cerr << "[ConfigLoader] 'perception.plugins' must be an array" << std::endl;
      return false;
    }
    
    perception_plugin_configs_.clear();
    
    for (const auto& plugin_json : plugins) {
      PerceptionPluginConfig config;
      
      // 必需字段：name
      if (!plugin_json.contains("name")) {
        std::cerr << "[ConfigLoader] Plugin config missing 'name' field" << std::endl;
        return false;
      }
      config.name = plugin_json["name"];
      
      // 可选字段
      if (plugin_json.contains("enabled")) {
        config.enabled = plugin_json["enabled"];
      }
      
      if (plugin_json.contains("priority")) {
        config.priority = plugin_json["priority"];
      }
      
      if (plugin_json.contains("params")) {
        config.params = plugin_json["params"];
      }
      
      perception_plugin_configs_.push_back(config);
    }
    
    std::cout << "[ConfigLoader] Loaded " << perception_plugin_configs_.size() 
              << " perception plugin configs" << std::endl;
  }
  
  return true;
}

bool ConfigLoader::parsePlanningConfig(const nlohmann::json& planning_config) {
  // 解析主规划器
  if (planning_config.contains("primary_planner")) {
    primary_planner_name_ = planning_config["primary_planner"];
  } else {
    std::cerr << "[ConfigLoader] Missing 'planning.primary_planner'" << std::endl;
    return false;
  }
  
  // 解析降级规划器
  if (planning_config.contains("fallback_planner")) {
    fallback_planner_name_ = planning_config["fallback_planner"];
  }
  
  // 解析是否启用降级
  if (planning_config.contains("enable_fallback")) {
    enable_fallback_ = planning_config["enable_fallback"];
  }
  
  // 解析规划器参数
  if (planning_config.contains("planners")) {
    planner_configs_ = planning_config["planners"];
  }
  
  std::cout << "[ConfigLoader] Primary planner: " << primary_planner_name_ << std::endl;
  if (enable_fallback_) {
    std::cout << "[ConfigLoader] Fallback planner: " << fallback_planner_name_ << std::endl;
  }
  
  return true;
}

nlohmann::json createDefaultConfig() {
  nlohmann::json config;
  
  // 感知配置
  config["perception"]["preprocessing"]["prediction_horizon"] = 5.0;
  config["perception"]["preprocessing"]["prediction_time_step"] = 0.1;
  
  config["perception"]["plugins"] = nlohmann::json::array();
  
  // 规划配置
  config["planning"]["primary_planner"] = "StraightLinePlannerPlugin";
  config["planning"]["fallback_planner"] = "StraightLinePlannerPlugin";
  config["planning"]["enable_fallback"] = false;
  config["planning"]["planners"] = nlohmann::json::object();
  
  return config;
}

std::pair<bool, std::string> validateConfig(const nlohmann::json& config) {
  // 检查是否是对象
  if (!config.is_object()) {
    return {false, "Config must be a JSON object"};
  }
  
  // 检查规划配置
  if (config.contains("planning")) {
    const auto& planning = config["planning"];
    
    if (!planning.is_object()) {
      return {false, "'planning' must be an object"};
    }
    
    if (!planning.contains("primary_planner")) {
      return {false, "Missing 'planning.primary_planner'"};
    }
    
    if (!planning["primary_planner"].is_string()) {
      return {false, "'planning.primary_planner' must be a string"};
    }
  }
  
  // 检查感知配置
  if (config.contains("perception")) {
    const auto& perception = config["perception"];
    
    if (!perception.is_object()) {
      return {false, "'perception' must be an object"};
    }
    
    if (perception.contains("plugins")) {
      if (!perception["plugins"].is_array()) {
        return {false, "'perception.plugins' must be an array"};
      }
      
      for (const auto& plugin : perception["plugins"]) {
        if (!plugin.contains("name")) {
          return {false, "Plugin config missing 'name' field"};
        }
        
        if (!plugin["name"].is_string()) {
          return {false, "Plugin 'name' must be a string"};
        }
      }
    }
  }
  
  return {true, ""};
}

} // namespace plugin
} // namespace navsim


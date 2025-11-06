#ifndef __ROS_TOOLS_PATHS_H__
#define __ROS_TOOLS_PATHS_H__

#include <string>
#include <cstdlib>
#include <filesystem>

#ifdef MPC_PLANNER_ROS
// ROS1 版本
#include <ros/package.h>

inline std::string getPackagePath(const std::string &&package_name)
{
    return ros::package::getPath(package_name) + "/";
}

inline std::string getPackagePath(const std::string &package_name)
{
    return ros::package::getPath(package_name) + "/";
}

#elif defined(MPC_PLANNER_ROS2)
// ROS2 版本
#include <ament_index_cpp/get_package_share_directory.hpp>

inline std::string getPackagePath(const std::string &&package_name)
{
    return ament_index_cpp::get_package_share_directory(package_name) + "/";
}

inline std::string getPackagePath(const std::string &package_name)
{
    return ament_index_cpp::get_package_share_directory(package_name) + "/";
}

#else
// 纯 C++ 版本（无 ROS 依赖）
// 使用环境变量或当前工作目录作为包路径
inline std::string getPackagePath(const std::string &&package_name)
{
    // 首先尝试从环境变量获取
    std::string env_var = package_name + "_PATH";
    const char* env_path = std::getenv(env_var.c_str());

    if (env_path != nullptr) {
        return std::string(env_path) + "/";
    }

    // 如果环境变量不存在，返回当前工作目录
    return std::filesystem::current_path().string() + "/" + package_name + "/";
}

inline std::string getPackagePath(const std::string &package_name)
{
    // 首先尝试从环境变量获取
    std::string env_var = package_name + "_PATH";
    const char* env_path = std::getenv(env_var.c_str());

    if (env_path != nullptr) {
        return std::string(env_path) + "/";
    }

    // 如果环境变量不存在，返回当前工作目录
    return std::filesystem::current_path().string() + "/" + package_name + "/";
}

#endif

#endif // __ROS_TOOLS_PATHS_H__

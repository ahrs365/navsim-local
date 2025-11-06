
#ifndef ros_tools_LOGGING_H_
#define ros_tools_LOGGING_H_

#ifdef MPC_PLANNER_ROS
// ROS1 版本
#include <ros/ros.h>
#define LOG_INFO(...) ROS_INFO_STREAM(__VA_ARGS__)
#define LOG_WARN(...) ROS_WARN_STREAM("\033[33m" << __VA_ARGS__ << "\033[0m")
#define LOG_ERROR(...) ROS_ERROR_STREAM(__VA_ARGS__)
#define LOG_DEBUG(...) ROS_DEBUG_STREAM(__VA_ARGS__)
#define LOG_SUCCESS(...) ROS_INFO_STREAM("\033[32m" << __VA_ARGS__ << "\033[0m")
#define LOG_INFO_THROTTLE(rate, ...) ROS_INFO_STREAM_THROTTLE(rate / 1000., __VA_ARGS__)
#define LOG_WARN_THROTTLE(rate, ...) ROS_WARN_STREAM_THROTTLE(rate / 1000., "\033[33m" << __VA_ARGS__ << "\033[0m")
#define LOG_ERROR_THROTTLE(rate, ...) ROS_ERROR_STREAM_THROTTLE(rate / 1000., __VA_ARGS__)
#define LOG_DEBUG_THROTTLE(rate, ...) ROS_DEBUG_STREAM_THROTTLE(rate / 1000., __VA_ARGS__)

#elif defined(MPC_PLANNER_ROS2)
// ROS2 版本
#include <rclcpp/logging.hpp>
#include <rclcpp/clock.hpp>
#include <ros_tools/ros2_wrappers.h>

#include <filesystem>
#define LOGGING_NAME std::filesystem::path(__FILE__).filename().replace_extension("").string()
#define LOG_INFO(...) RCLCPP_INFO_STREAM(GET_STATIC_NODE_POINTER()->get_logger(), __VA_ARGS__)
#define LOG_WARN(...) RCLCPP_WARN_STREAM(GET_STATIC_NODE_POINTER()->get_logger(), "\033[33m" << __VA_ARGS__ << "\033[0m")
#define LOG_ERROR(...) RCLCPP_ERROR_STREAM(GET_STATIC_NODE_POINTER()->get_logger(), __VA_ARGS__)
#define LOG_SUCCESS(...) RCLCPP_INFO_STREAM(GET_STATIC_NODE_POINTER()->get_logger(), "\033[32m" << __VA_ARGS__ "\033[0m")
#define LOG_DEBUG(...) RCLCPP_DEBUG_STREAM(GET_STATIC_NODE_POINTER()->get_logger(), __VA_ARGS__)

inline void __RCLCPP_WARN_STREAM_THROTTLE(const double rate, const std::string &msg)
{
    auto clock = *GET_STATIC_NODE_POINTER()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(GET_STATIC_NODE_POINTER()->get_logger(),
                                clock, rate, "\033[33m" << msg << "\033[0m");
}

inline void __RCLCPP_INFO_STREAM_THROTTLE(const double rate, const std::string &msg)
{
    auto clock = *GET_STATIC_NODE_POINTER()->get_clock();
    RCLCPP_INFO_STREAM_THROTTLE(GET_STATIC_NODE_POINTER()->get_logger(), clock, rate, msg);
}

#define LOG_INFO_THROTTLE(rate, ...) __RCLCPP_INFO_STREAM_THROTTLE(rate, __VA_ARGS__)
#define LOG_WARN_THROTTLE(rate, ...) __RCLCPP_WARN_STREAM_THROTTLE(rate, __VA_ARGS__)
#define LOG_ERROR_THROTTLE(rate, ...) RCLCPP_ERROR_STREAM_THROTTLE(GET_STATIC_NODE_POINTER()->get_logger(), rate, __VA_ARGS__)
#define LOG_DEBUG_THROTTLE(rate, ...) RCLCPP_DEBUG_STREAM_THROTTLE(GET_STATIC_NODE_POINTER()->get_logger(), rate, __VA_ARGS__)

#else
// 纯 C++ 版本（无 ROS 依赖）
#include <iostream>
#include <chrono>
#include <map>
#include <sstream>

#define LOG_INFO(...) do { std::ostringstream oss; oss << __VA_ARGS__; std::cout << "[INFO] " << oss.str() << std::endl; } while(0)
#define LOG_WARN(...) do { std::ostringstream oss; oss << __VA_ARGS__; std::cerr << "\033[33m[WARN] " << oss.str() << "\033[0m" << std::endl; } while(0)
#define LOG_ERROR(...) do { std::ostringstream oss; oss << __VA_ARGS__; std::cerr << "\033[31m[ERROR] " << oss.str() << "\033[0m" << std::endl; } while(0)
#define LOG_DEBUG(...) do { std::ostringstream oss; oss << __VA_ARGS__; std::cout << "[DEBUG] " << oss.str() << std::endl; } while(0)
#define LOG_SUCCESS(...) do { std::ostringstream oss; oss << __VA_ARGS__; std::cout << "\033[32m[SUCCESS] " << oss.str() << "\033[0m" << std::endl; } while(0)

// 简单的节流实现
inline bool __should_log_throttle(const std::string& id, double rate_ms) {
    static std::map<std::string, std::chrono::steady_clock::time_point> last_log_times;
    auto now = std::chrono::steady_clock::now();
    auto it = last_log_times.find(id);

    if (it == last_log_times.end() ||
        std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count() >= rate_ms) {
        last_log_times[id] = now;
        return true;
    }
    return false;
}

#define LOG_INFO_THROTTLE(rate, ...) do { \
    static int __throttle_counter = 0; \
    std::string __throttle_id = std::string(__FILE__) + ":" + std::to_string(__LINE__) + ":" + std::to_string(__throttle_counter++); \
    if (__should_log_throttle(__throttle_id, rate)) { LOG_INFO(__VA_ARGS__); } \
} while(0)

#define LOG_WARN_THROTTLE(rate, ...) do { \
    static int __throttle_counter = 0; \
    std::string __throttle_id = std::string(__FILE__) + ":" + std::to_string(__LINE__) + ":" + std::to_string(__throttle_counter++); \
    if (__should_log_throttle(__throttle_id, rate)) { LOG_WARN(__VA_ARGS__); } \
} while(0)

#define LOG_ERROR_THROTTLE(rate, ...) do { \
    static int __throttle_counter = 0; \
    std::string __throttle_id = std::string(__FILE__) + ":" + std::to_string(__LINE__) + ":" + std::to_string(__throttle_counter++); \
    if (__should_log_throttle(__throttle_id, rate)) { LOG_ERROR(__VA_ARGS__); } \
} while(0)

#define LOG_DEBUG_THROTTLE(rate, ...) do { \
    static int __throttle_counter = 0; \
    std::string __throttle_id = std::string(__FILE__) + ":" + std::to_string(__LINE__) + ":" + std::to_string(__throttle_counter++); \
    if (__should_log_throttle(__throttle_id, rate)) { LOG_DEBUG(__VA_ARGS__); } \
} while(0)

#endif

#define LOG_VALUE(name, value) LOG_INFO("\033[1m" << name << ":\033[0m " << value)
#define LOG_VALUE_DEBUG(name, value) LOG_DEBUG("\033[1m" << name << ":\033[0m " << value)
#define LOG_DIVIDER() LOG_INFO("========================================")
#define LOG_HEADER(msg) LOG_INFO("=============== " << msg << " ================")

#define LOG_HOOK() LOG_INFO(__FILE__ << ":" << __LINE__);
#define LOG_HOOK_MSG(msg) LOG_INFO(__FILE__ << ":" << __LINE__ << " " << msg);

#define ROSTOOLS_ASSERT(Expr, Msg) __ROSTOOLS_ASSERT(#Expr, Expr, __FILE__, __LINE__, Msg)

// From https://stackoverflow.com/questions/3692954/add-custom-messages-in-assert
inline void __ROSTOOLS_ASSERT(const char *expr_str, bool expr, const char *file, int line, const char *msg)
{
    if (!expr)
    {
        LOG_ERROR("Assert failed:\t" << msg << "\n"
                                     << "Expected:\t" << expr_str << "\n"
                                     << "Source:\t\t" << file << ", line " << line << "\n");
        abort();
    }
}

class LogInitialize
{
public:
    LogInitialize(std::string &&name)
    {
        _log_output = "Initializing " + name;
    }

    ~LogInitialize()
    {
        if (!_printed)
            LOG_INFO(_log_output);
    }

    void print()
    {
        _printed = true;
        _log_output += "\033[32m -> Done\033[0m";

        LOG_INFO(_log_output);
    }

private:
    bool _printed{false};
    std::string _log_output{""};
};

#define LOG_INITIALIZE(name) LogInitialize log_initialize(name)
#define LOG_INITIALIZED() log_initialize.print()

#endif // ros_tools_H

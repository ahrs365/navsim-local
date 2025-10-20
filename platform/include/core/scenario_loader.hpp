#pragma once

#include "core/planning_context.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <memory>

namespace navsim {
namespace planning {

/**
 * @brief JSON 场景加载器
 * 
 * 从 JSON 文件加载场景数据，转换为 PlanningContext。
 * 支持与 navsim-online 保存的场景格式一致。
 * 
 * JSON 格式示例：
 * ```json
 * {
 *   "scenario_name": "simple_corridor",
 *   "timestamp": 0.0,
 *   "ego": {
 *     "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
 *     "twist": {"vx": 0.0, "vy": 0.0, "omega": 0.0},
 *     "chassis_model": "differential"
 *   },
 *   "task": {
 *     "goal_pose": {"x": 10.0, "y": 0.0, "yaw": 0.0},
 *     "type": "GOTO_GOAL"
 *   },
 *   "obstacles": [
 *     {
 *       "type": "circle",
 *       "center": {"x": 5.0, "y": 0.0},
 *       "radius": 1.0
 *     },
 *     {
 *       "type": "rectangle",
 *       "pose": {"x": 7.0, "y": 2.0, "yaw": 0.0},
 *       "width": 2.0,
 *       "height": 1.0
 *     }
 *   ],
 *   "dynamic_obstacles": [
 *     {
 *       "id": 1,
 *       "type": "vehicle",
 *       "shape_type": "rectangle",
 *       "current_pose": {"x": 8.0, "y": 0.0, "yaw": 0.0},
 *       "current_twist": {"vx": 2.0, "vy": 0.0, "omega": 0.0},
 *       "length": 4.5,
 *       "width": 2.0
 *     }
 *   ]
 * }
 * ```
 */
class ScenarioLoader {
public:
  /**
   * @brief 从 JSON 文件加载场景
   * 
   * @param file_path JSON 文件路径
   * @param context 输出的规划上下文
   * @return 是否成功加载
   */
  static bool loadFromFile(const std::string& file_path, PlanningContext& context);

  /**
   * @brief 从 JSON 字符串加载场景
   * 
   * @param json_str JSON 字符串
   * @param context 输出的规划上下文
   * @return 是否成功加载
   */
  static bool loadFromString(const std::string& json_str, PlanningContext& context);

  /**
   * @brief 从 JSON 对象加载场景
   * 
   * @param json JSON 对象
   * @param context 输出的规划上下文
   * @return 是否成功加载
   */
  static bool loadFromJson(const nlohmann::json& json, PlanningContext& context);

  /**
   * @brief 保存场景到 JSON 文件
   * 
   * @param file_path JSON 文件路径
   * @param context 规划上下文
   * @return 是否成功保存
   */
  static bool saveToFile(const std::string& file_path, const PlanningContext& context);

  /**
   * @brief 将场景转换为 JSON 对象
   * 
   * @param context 规划上下文
   * @return JSON 对象
   */
  static nlohmann::json toJson(const PlanningContext& context);

private:
  // ========== JSON 解析辅助函数 ==========

  static bool parseEgo(const nlohmann::json& json, EgoVehicle& ego);
  static bool parseTask(const nlohmann::json& json, PlanningTask& task);
  static bool parseObstacles(const nlohmann::json& json, PlanningContext& context);
  static bool parseDynamicObstacles(const nlohmann::json& json, PlanningContext& context);

  // ========== JSON 序列化辅助函数 ==========

  static nlohmann::json egoToJson(const EgoVehicle& ego);
  static nlohmann::json taskToJson(const PlanningTask& task);
  static nlohmann::json obstaclesToJson(const PlanningContext& context);
  static nlohmann::json dynamicObstaclesToJson(const PlanningContext& context);

  // ========== 基础类型转换 ==========

  static Point2d parsePoint2d(const nlohmann::json& json);
  static Pose2d parsePose2d(const nlohmann::json& json);
  static Twist2d parseTwist2d(const nlohmann::json& json);

  static nlohmann::json point2dToJson(const Point2d& point);
  static nlohmann::json pose2dToJson(const Pose2d& pose);
  static nlohmann::json twist2dToJson(const Twist2d& twist);

  // ========== Online 格式兼容性 ==========

  /**
   * @brief 检测是否为 online 模式保存的格式
   * @param json JSON 对象
   * @return 是否为 online 格式
   */
  static bool isOnlineFormat(const nlohmann::json& json);

  /**
   * @brief 将 online 格式转换为内部格式
   * @param online_json Online 格式的 JSON
   * @return 内部格式的 JSON
   */
  static nlohmann::json convertOnlineToInternal(const nlohmann::json& online_json);
};

} // namespace planning
} // namespace navsim


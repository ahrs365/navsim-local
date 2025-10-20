#include "core/scenario_loader.hpp"
#include <fstream>
#include <iostream>

namespace navsim {
namespace planning {

// ========== 公共接口 ==========

bool ScenarioLoader::loadFromFile(const std::string& file_path, PlanningContext& context) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "[ScenarioLoader] Failed to open file: " << file_path << std::endl;
    return false;
  }

  nlohmann::json json;
  try {
    file >> json;
  } catch (const nlohmann::json::exception& e) {
    std::cerr << "[ScenarioLoader] JSON parse error: " << e.what() << std::endl;
    return false;
  }

  return loadFromJson(json, context);
}

bool ScenarioLoader::loadFromString(const std::string& json_str, PlanningContext& context) {
  nlohmann::json json;
  try {
    json = nlohmann::json::parse(json_str);
  } catch (const nlohmann::json::exception& e) {
    std::cerr << "[ScenarioLoader] JSON parse error: " << e.what() << std::endl;
    return false;
  }

  return loadFromJson(json, context);
}

bool ScenarioLoader::loadFromJson(const nlohmann::json& json, PlanningContext& context) {
  try {
    // 检测并转换 online 格式
    nlohmann::json internal_json = json;
    if (isOnlineFormat(json)) {
      std::cout << "[ScenarioLoader] Detected online format, converting..." << std::endl;
      internal_json = convertOnlineToInternal(json);
    }

    // 解析时间戳
    if (internal_json.contains("timestamp")) {
      context.timestamp = internal_json["timestamp"].get<double>();
    }

    // 解析规划时域
    if (internal_json.contains("planning_horizon")) {
      context.planning_horizon = internal_json["planning_horizon"].get<double>();
    }

    // 解析自车状态
    if (internal_json.contains("ego")) {
      if (!parseEgo(internal_json["ego"], context.ego)) {
        std::cerr << "[ScenarioLoader] Failed to parse ego vehicle" << std::endl;
        return false;
      }
    }

    // 解析任务目标
    if (internal_json.contains("task")) {
      if (!parseTask(internal_json["task"], context.task)) {
        std::cerr << "[ScenarioLoader] Failed to parse task" << std::endl;
        return false;
      }
    }

    // 解析静态障碍物
    if (internal_json.contains("obstacles")) {
      if (!parseObstacles(internal_json["obstacles"], context)) {
        std::cerr << "[ScenarioLoader] Failed to parse obstacles" << std::endl;
        return false;
      }
    }

    // 解析动态障碍物
    if (internal_json.contains("dynamic_obstacles")) {
      if (!parseDynamicObstacles(internal_json["dynamic_obstacles"], context)) {
        std::cerr << "[ScenarioLoader] Failed to parse dynamic obstacles" << std::endl;
        return false;
      }
    }

    std::cout << "[ScenarioLoader] Successfully loaded scenario" << std::endl;
    return true;

  } catch (const nlohmann::json::exception& e) {
    std::cerr << "[ScenarioLoader] JSON error: " << e.what() << std::endl;
    return false;
  }
}

bool ScenarioLoader::saveToFile(const std::string& file_path, const PlanningContext& context) {
  nlohmann::json json = toJson(context);

  std::ofstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "[ScenarioLoader] Failed to open file for writing: " << file_path << std::endl;
    return false;
  }

  file << json.dump(2);  // 2-space indentation
  std::cout << "[ScenarioLoader] Successfully saved scenario to: " << file_path << std::endl;
  return true;
}

nlohmann::json ScenarioLoader::toJson(const PlanningContext& context) {
  nlohmann::json json;

  json["timestamp"] = context.timestamp;
  json["planning_horizon"] = context.planning_horizon;
  json["ego"] = egoToJson(context.ego);
  json["task"] = taskToJson(context.task);
  json["obstacles"] = obstaclesToJson(context);
  json["dynamic_obstacles"] = dynamicObstaclesToJson(context);

  return json;
}

// ========== 基础类型转换 ==========

Point2d ScenarioLoader::parsePoint2d(const nlohmann::json& json) {
  Point2d point;
  point.x = json.value("x", 0.0);
  point.y = json.value("y", 0.0);
  return point;
}

Pose2d ScenarioLoader::parsePose2d(const nlohmann::json& json) {
  Pose2d pose;
  pose.x = json.value("x", 0.0);
  pose.y = json.value("y", 0.0);
  pose.yaw = json.value("yaw", 0.0);
  return pose;
}

Twist2d ScenarioLoader::parseTwist2d(const nlohmann::json& json) {
  Twist2d twist;
  twist.vx = json.value("vx", 0.0);
  twist.vy = json.value("vy", 0.0);
  twist.omega = json.value("omega", 0.0);
  return twist;
}

nlohmann::json ScenarioLoader::point2dToJson(const Point2d& point) {
  return {{"x", point.x}, {"y", point.y}};
}

nlohmann::json ScenarioLoader::pose2dToJson(const Pose2d& pose) {
  return {{"x", pose.x}, {"y", pose.y}, {"yaw", pose.yaw}};
}

nlohmann::json ScenarioLoader::twist2dToJson(const Twist2d& twist) {
  return {{"vx", twist.vx}, {"vy", twist.vy}, {"omega", twist.omega}};
}

// ========== Ego 解析 ==========

bool ScenarioLoader::parseEgo(const nlohmann::json& json, EgoVehicle& ego) {
  if (json.contains("pose")) {
    ego.pose = parsePose2d(json["pose"]);
  }

  if (json.contains("twist")) {
    ego.twist = parseTwist2d(json["twist"]);
  }

  if (json.contains("timestamp")) {
    ego.timestamp = json["timestamp"].get<double>();
  }

  if (json.contains("chassis_model")) {
    ego.chassis_model = json["chassis_model"].get<std::string>();
  }

  // 解析运动学参数
  if (json.contains("kinematics")) {
    const auto& kin = json["kinematics"];
    ego.kinematics.wheelbase = kin.value("wheelbase", 2.8);
    ego.kinematics.track_width = kin.value("track_width", 2.0);
    ego.kinematics.front_overhang = kin.value("front_overhang", 1.0);
    ego.kinematics.rear_overhang = kin.value("rear_overhang", 1.0);
    ego.kinematics.width = kin.value("width", 2.0);
    ego.kinematics.height = kin.value("height", 1.8);
    ego.kinematics.body_length = kin.value("body_length", 4.8);
    ego.kinematics.body_width = kin.value("body_width", 2.0);
    ego.kinematics.wheel_radius = kin.value("wheel_radius", 0.3);
  }

  // 解析动力学约束
  if (json.contains("limits")) {
    const auto& lim = json["limits"];
    ego.limits.max_velocity = lim.value("max_velocity", 15.0);
    ego.limits.max_acceleration = lim.value("max_acceleration", 3.0);
    ego.limits.max_deceleration = lim.value("max_deceleration", 8.0);
    ego.limits.max_steer_angle = lim.value("max_steer_angle", 0.6);
    ego.limits.max_steer_rate = lim.value("max_steer_rate", 1.0);
    ego.limits.max_jerk = lim.value("max_jerk", 3.0);
    ego.limits.max_curvature = lim.value("max_curvature", 0.2);
  }

  return true;
}

nlohmann::json ScenarioLoader::egoToJson(const EgoVehicle& ego) {
  nlohmann::json json;
  json["pose"] = pose2dToJson(ego.pose);
  json["twist"] = twist2dToJson(ego.twist);
  json["timestamp"] = ego.timestamp;
  json["chassis_model"] = ego.chassis_model;

  json["kinematics"] = {
    {"wheelbase", ego.kinematics.wheelbase},
    {"track_width", ego.kinematics.track_width},
    {"front_overhang", ego.kinematics.front_overhang},
    {"rear_overhang", ego.kinematics.rear_overhang},
    {"width", ego.kinematics.width},
    {"height", ego.kinematics.height},
    {"body_length", ego.kinematics.body_length},
    {"body_width", ego.kinematics.body_width},
    {"wheel_radius", ego.kinematics.wheel_radius}
  };

  json["limits"] = {
    {"max_velocity", ego.limits.max_velocity},
    {"max_acceleration", ego.limits.max_acceleration},
    {"max_deceleration", ego.limits.max_deceleration},
    {"max_steer_angle", ego.limits.max_steer_angle},
    {"max_steer_rate", ego.limits.max_steer_rate},
    {"max_jerk", ego.limits.max_jerk},
    {"max_curvature", ego.limits.max_curvature}
  };

  return json;
}

// ========== Task 解析 ==========

bool ScenarioLoader::parseTask(const nlohmann::json& json, PlanningTask& task) {
  if (json.contains("goal_pose")) {
    task.goal_pose = parsePose2d(json["goal_pose"]);
  }

  if (json.contains("type")) {
    std::string type_str = json["type"].get<std::string>();
    if (type_str == "GOTO_GOAL") {
      task.type = PlanningTask::Type::GOTO_GOAL;
    } else if (type_str == "LANE_FOLLOWING") {
      task.type = PlanningTask::Type::LANE_FOLLOWING;
    } else if (type_str == "LANE_CHANGE") {
      task.type = PlanningTask::Type::LANE_CHANGE;
    } else if (type_str == "PARKING") {
      task.type = PlanningTask::Type::PARKING;
    } else if (type_str == "EMERGENCY_STOP") {
      task.type = PlanningTask::Type::EMERGENCY_STOP;
    }
  }

  if (json.contains("tolerance")) {
    const auto& tol = json["tolerance"];
    task.tolerance.position = tol.value("position", 0.5);
    task.tolerance.yaw = tol.value("yaw", 0.2);
  }

  return true;
}

nlohmann::json ScenarioLoader::taskToJson(const PlanningTask& task) {
  nlohmann::json json;
  json["goal_pose"] = pose2dToJson(task.goal_pose);

  // 转换任务类型
  std::string type_str;
  switch (task.type) {
    case PlanningTask::Type::GOTO_GOAL:
      type_str = "GOTO_GOAL";
      break;
    case PlanningTask::Type::LANE_FOLLOWING:
      type_str = "LANE_FOLLOWING";
      break;
    case PlanningTask::Type::LANE_CHANGE:
      type_str = "LANE_CHANGE";
      break;
    case PlanningTask::Type::PARKING:
      type_str = "PARKING";
      break;
    case PlanningTask::Type::EMERGENCY_STOP:
      type_str = "EMERGENCY_STOP";
      break;
  }
  json["type"] = type_str;

  json["tolerance"] = {
    {"position", task.tolerance.position},
    {"yaw", task.tolerance.yaw}
  };

  return json;
}

// ========== 静态障碍物解析 ==========

bool ScenarioLoader::parseObstacles(const nlohmann::json& json, PlanningContext& context) {
  if (!json.is_array()) {
    std::cerr << "[ScenarioLoader] Obstacles must be an array" << std::endl;
    return false;
  }

  // 创建 BEV 障碍物容器
  if (!context.bev_obstacles) {
    context.bev_obstacles = std::make_unique<BEVObstacles>();
  }

  for (const auto& obs : json) {
    std::string type = obs.value("type", "");

    if (type == "circle") {
      BEVObstacles::Circle circle;
      circle.center = parsePoint2d(obs["center"]);
      circle.radius = obs.value("radius", 1.0);
      circle.confidence = obs.value("confidence", 1.0);
      context.bev_obstacles->circles.push_back(circle);

    } else if (type == "rectangle") {
      BEVObstacles::Rectangle rect;
      rect.pose = parsePose2d(obs["pose"]);
      rect.width = obs.value("width", 2.0);
      rect.height = obs.value("height", 1.0);
      rect.confidence = obs.value("confidence", 1.0);
      context.bev_obstacles->rectangles.push_back(rect);

    } else if (type == "polygon") {
      BEVObstacles::Polygon poly;
      if (obs.contains("vertices") && obs["vertices"].is_array()) {
        for (const auto& vertex : obs["vertices"]) {
          poly.vertices.push_back(parsePoint2d(vertex));
        }
      }
      poly.confidence = obs.value("confidence", 1.0);
      context.bev_obstacles->polygons.push_back(poly);

    } else {
      std::cerr << "[ScenarioLoader] Unknown obstacle type: " << type << std::endl;
    }
  }

  return true;
}

nlohmann::json ScenarioLoader::obstaclesToJson(const PlanningContext& context) {
  nlohmann::json json = nlohmann::json::array();

  if (!context.bev_obstacles) {
    return json;
  }

  // 圆形障碍物
  for (const auto& circle : context.bev_obstacles->circles) {
    nlohmann::json obs;
    obs["type"] = "circle";
    obs["center"] = point2dToJson(circle.center);
    obs["radius"] = circle.radius;
    obs["confidence"] = circle.confidence;
    json.push_back(obs);
  }

  // 矩形障碍物
  for (const auto& rect : context.bev_obstacles->rectangles) {
    nlohmann::json obs;
    obs["type"] = "rectangle";
    obs["pose"] = pose2dToJson(rect.pose);
    obs["width"] = rect.width;
    obs["height"] = rect.height;
    obs["confidence"] = rect.confidence;
    json.push_back(obs);
  }

  // 多边形障碍物
  for (const auto& poly : context.bev_obstacles->polygons) {
    nlohmann::json obs;
    obs["type"] = "polygon";
    obs["vertices"] = nlohmann::json::array();
    for (const auto& vertex : poly.vertices) {
      obs["vertices"].push_back(point2dToJson(vertex));
    }
    obs["confidence"] = poly.confidence;
    json.push_back(obs);
  }

  return json;
}

// ========== 动态障碍物解析 ==========

bool ScenarioLoader::parseDynamicObstacles(const nlohmann::json& json, PlanningContext& context) {
  if (!json.is_array()) {
    std::cerr << "[ScenarioLoader] Dynamic obstacles must be an array" << std::endl;
    return false;
  }

  for (const auto& obs : json) {
    DynamicObstacle dyn_obs;

    dyn_obs.id = obs.value("id", 0);
    dyn_obs.type = obs.value("type", "vehicle");
    dyn_obs.shape_type = obs.value("shape_type", "rectangle");

    if (obs.contains("current_pose")) {
      dyn_obs.current_pose = parsePose2d(obs["current_pose"]);
    }

    if (obs.contains("current_twist")) {
      dyn_obs.current_twist = parseTwist2d(obs["current_twist"]);
    }

    dyn_obs.length = obs.value("length", 4.5);
    dyn_obs.width = obs.value("width", 2.0);
    dyn_obs.height = obs.value("height", 1.8);

    // 解析预测轨迹（如果有）
    if (obs.contains("predicted_trajectories") && obs["predicted_trajectories"].is_array()) {
      for (const auto& traj_json : obs["predicted_trajectories"]) {
        DynamicObstacle::Trajectory traj;
        traj.probability = traj_json.value("probability", 1.0);

        if (traj_json.contains("poses") && traj_json["poses"].is_array()) {
          for (const auto& pose_json : traj_json["poses"]) {
            traj.poses.push_back(parsePose2d(pose_json));
          }
        }

        if (traj_json.contains("timestamps") && traj_json["timestamps"].is_array()) {
          for (const auto& ts : traj_json["timestamps"]) {
            traj.timestamps.push_back(ts.get<double>());
          }
        }

        dyn_obs.predicted_trajectories.push_back(traj);
      }
    }

    context.dynamic_obstacles.push_back(dyn_obs);
  }

  return true;
}

nlohmann::json ScenarioLoader::dynamicObstaclesToJson(const PlanningContext& context) {
  nlohmann::json json = nlohmann::json::array();

  for (const auto& dyn_obs : context.dynamic_obstacles) {
    nlohmann::json obs;
    obs["id"] = dyn_obs.id;
    obs["type"] = dyn_obs.type;
    obs["shape_type"] = dyn_obs.shape_type;
    obs["current_pose"] = pose2dToJson(dyn_obs.current_pose);
    obs["current_twist"] = twist2dToJson(dyn_obs.current_twist);
    obs["length"] = dyn_obs.length;
    obs["width"] = dyn_obs.width;
    obs["height"] = dyn_obs.height;

    // 序列化预测轨迹
    obs["predicted_trajectories"] = nlohmann::json::array();
    for (const auto& traj : dyn_obs.predicted_trajectories) {
      nlohmann::json traj_json;
      traj_json["probability"] = traj.probability;

      traj_json["poses"] = nlohmann::json::array();
      for (const auto& pose : traj.poses) {
        traj_json["poses"].push_back(pose2dToJson(pose));
      }

      traj_json["timestamps"] = traj.timestamps;

      obs["predicted_trajectories"].push_back(traj_json);
    }

    json.push_back(obs);
  }

  return json;
}

// ========== Online 格式兼容性 ==========

bool ScenarioLoader::isOnlineFormat(const nlohmann::json& json) {
  // Online 格式的特征：
  // - 有 "startPose" 和 "goalPose" 而不是 "ego" 和 "task"
  // - 有 "obstacles" 对象包含 "circles", "polygons", "dynamic" 数组
  // - 有 "chassisType" 和 "chassisConfig"
  // - 有 "name" 而不是 "scenario_name"

  bool has_online_pose_fields = json.contains("startPose") && json.contains("goalPose");
  bool has_online_obstacles = json.contains("obstacles") && json["obstacles"].is_object() &&
                              (json["obstacles"].contains("circles") ||
                               json["obstacles"].contains("polygons") ||
                               json["obstacles"].contains("dynamic"));
  bool has_chassis_fields = json.contains("chassisType") || json.contains("chassisConfig");
  bool has_name_field = json.contains("name") && !json.contains("scenario_name");

  // 如果有多个 online 格式特征，则认为是 online 格式
  int online_features = 0;
  if (has_online_pose_fields) online_features++;
  if (has_online_obstacles) online_features++;
  if (has_chassis_fields) online_features++;
  if (has_name_field) online_features++;

  return online_features >= 2;
}

nlohmann::json ScenarioLoader::convertOnlineToInternal(const nlohmann::json& online_json) {
  nlohmann::json internal_json;

  // 设置默认值
  internal_json["scenario_name"] = online_json.value("name", "converted_online_scenario");
  internal_json["description"] = "Converted from online format";
  internal_json["timestamp"] = online_json.value("timestamp", 0.0) / 1000.0; // 转换毫秒到秒
  internal_json["planning_horizon"] = 5.0; // 默认规划时域

  // 转换自车信息
  nlohmann::json ego;
  if (online_json.contains("startPose")) {
    ego["pose"] = {
      {"x", online_json["startPose"].value("x", 0.0)},
      {"y", online_json["startPose"].value("y", 0.0)},
      {"yaw", online_json["startPose"].value("yaw", 0.0)}
    };
  } else {
    ego["pose"] = {{"x", 0.0}, {"y", 0.0}, {"yaw", 0.0}};
  }

  ego["twist"] = {{"vx", 0.0}, {"vy", 0.0}, {"omega", 0.0}};

  // 从 chassisConfig 提取车辆参数
  if (online_json.contains("chassisType")) {
    ego["chassis_model"] = online_json["chassisType"].get<std::string>();
  } else {
    ego["chassis_model"] = "differential";
  }

  if (online_json.contains("chassisConfig")) {
    const auto& chassis = online_json["chassisConfig"];

    // 运动学参数
    nlohmann::json kinematics;
    if (chassis.contains("geometry")) {
      const auto& geom = chassis["geometry"];
      kinematics["wheelbase"] = geom.value("wheelbase", 2.8);
      kinematics["track_width"] = geom.value("track_width", 2.0);
      kinematics["body_length"] = geom.value("body_length", 4.8);
      kinematics["body_width"] = geom.value("body_width", 2.0);
      kinematics["wheel_radius"] = geom.value("wheel_radius", 0.3);
      kinematics["front_overhang"] = geom.value("front_overhang", 1.0);
      kinematics["rear_overhang"] = geom.value("rear_overhang", 1.0);
      kinematics["width"] = geom.value("body_width", 2.0);
      kinematics["height"] = geom.value("body_height", 1.8);
    } else {
      // 默认值
      kinematics = {
        {"wheelbase", 2.8}, {"track_width", 2.0}, {"body_length", 4.8},
        {"body_width", 2.0}, {"wheel_radius", 0.3}, {"front_overhang", 1.0},
        {"rear_overhang", 1.0}, {"width", 2.0}, {"height", 1.8}
      };
    }
    ego["kinematics"] = kinematics;

    // 动力学约束
    nlohmann::json limits;
    if (chassis.contains("limits")) {
      const auto& lim = chassis["limits"];
      limits["max_velocity"] = lim.value("v_max", 15.0);
      limits["max_acceleration"] = lim.value("a_max", 3.0);
      limits["max_deceleration"] = 8.0;
      limits["max_steer_angle"] = lim.value("steer_max", 0.6);
      limits["max_steer_rate"] = 1.0;
      limits["max_jerk"] = 3.0;
      limits["max_curvature"] = 0.2;
    } else {
      limits = {
        {"max_velocity", 15.0}, {"max_acceleration", 3.0}, {"max_deceleration", 8.0},
        {"max_steer_angle", 0.6}, {"max_steer_rate", 1.0}, {"max_jerk", 3.0},
        {"max_curvature", 0.2}
      };
    }
    ego["limits"] = limits;
  } else {
    // 默认运动学和动力学参数
    ego["kinematics"] = {
      {"wheelbase", 2.8}, {"track_width", 2.0}, {"body_length", 4.8},
      {"body_width", 2.0}, {"wheel_radius", 0.3}, {"front_overhang", 1.0},
      {"rear_overhang", 1.0}, {"width", 2.0}, {"height", 1.8}
    };
    ego["limits"] = {
      {"max_velocity", 15.0}, {"max_acceleration", 3.0}, {"max_deceleration", 8.0},
      {"max_steer_angle", 0.6}, {"max_steer_rate", 1.0}, {"max_jerk", 3.0},
      {"max_curvature", 0.2}
    };
  }

  internal_json["ego"] = ego;

  // 转换任务目标
  nlohmann::json task;
  if (online_json.contains("goalPose")) {
    const auto& goal = online_json["goalPose"];
    task["goal_pose"] = {
      {"x", goal.value("x", 20.0)},
      {"y", goal.value("y", 0.0)},
      {"yaw", goal.value("yaw", 0.0)}
    };

    // 提取容差
    if (goal.contains("tol")) {
      task["tolerance"] = {
        {"position", goal["tol"].value("pos", 0.5)},
        {"yaw", goal["tol"].value("yaw", 0.2)}
      };
    } else {
      task["tolerance"] = {{"position", 0.5}, {"yaw", 0.2}};
    }
  } else {
    task["goal_pose"] = {{"x", 20.0}, {"y", 0.0}, {"yaw", 0.0}};
    task["tolerance"] = {{"position", 0.5}, {"yaw", 0.2}};
  }

  task["type"] = "GOTO_GOAL";
  internal_json["task"] = task;

  // 转换静态障碍物
  nlohmann::json obstacles = nlohmann::json::array();

  if (online_json.contains("obstacles") && online_json["obstacles"].is_object()) {
    const auto& online_obstacles = online_json["obstacles"];

    // 转换圆形障碍物
    if (online_obstacles.contains("circles") && online_obstacles["circles"].is_array()) {
      for (const auto& circle : online_obstacles["circles"]) {
        nlohmann::json obs;
        obs["type"] = "circle";
        obs["center"] = {
          {"x", circle.value("x", 0.0)},
          {"y", circle.value("y", 0.0)}
        };
        obs["radius"] = circle.value("radius", 1.0);
        obs["confidence"] = 1.0;
        obstacles.push_back(obs);
      }
    }

    // 转换多边形障碍物
    if (online_obstacles.contains("polygons") && online_obstacles["polygons"].is_array()) {
      for (const auto& polygon : online_obstacles["polygons"]) {
        nlohmann::json obs;
        obs["type"] = "polygon";
        obs["vertices"] = nlohmann::json::array();

        if (polygon.contains("points") && polygon["points"].is_array()) {
          for (const auto& point : polygon["points"]) {
            obs["vertices"].push_back({
              {"x", point.value("x", 0.0)},
              {"y", point.value("y", 0.0)}
            });
          }
        }
        obs["confidence"] = 1.0;
        obstacles.push_back(obs);
      }
    }
  }

  internal_json["obstacles"] = obstacles;

  // 转换动态障碍物
  nlohmann::json dynamic_obstacles = nlohmann::json::array();

  if (online_json.contains("obstacles") && online_json["obstacles"].contains("dynamic")) {
    const auto& online_dynamic = online_json["obstacles"]["dynamic"];

    for (size_t i = 0; i < online_dynamic.size(); ++i) {
      const auto& dyn_obs = online_dynamic[i];
      nlohmann::json obs;

      obs["id"] = static_cast<int>(i + 1);
      obs["type"] = "vehicle";

      if (dyn_obs.contains("kind")) {
        std::string kind = dyn_obs["kind"].get<std::string>();
        obs["shape_type"] = (kind == "circle") ? "circle" : "rectangle";
      } else {
        obs["shape_type"] = "rectangle";
      }

      if (dyn_obs.contains("state")) {
        const auto& state = dyn_obs["state"];
        obs["current_pose"] = {
          {"x", state.value("x", 0.0)},
          {"y", state.value("y", 0.0)},
          {"yaw", state.value("yaw", 0.0)}
        };
        obs["current_twist"] = {
          {"vx", state.value("vx", 0.0)},
          {"vy", state.value("vy", 0.0)},
          {"omega", 0.0}
        };
      }

      if (dyn_obs.contains("data")) {
        const auto& data = dyn_obs["data"];
        if (data.contains("w") && data.contains("h")) {
          obs["width"] = data.value("w", 2.0);
          obs["length"] = data.value("h", 4.5);
        } else if (data.contains("r")) {
          obs["width"] = data.value("r", 1.0) * 2.0;
          obs["length"] = data.value("r", 1.0) * 2.0;
        } else {
          obs["width"] = 2.0;
          obs["length"] = 4.5;
        }
      } else {
        obs["width"] = 2.0;
        obs["length"] = 4.5;
      }

      obs["height"] = 1.8;
      dynamic_obstacles.push_back(obs);
    }
  }

  internal_json["dynamic_obstacles"] = dynamic_obstacles;

  return internal_json;
}

} // namespace planning
} // namespace navsim


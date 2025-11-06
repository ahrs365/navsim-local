#ifndef MPC_PLANNER_MODEL_DETECTOR_H
#define MPC_PLANNER_MODEL_DETECTOR_H

#include <mpc_planner_util/load_yaml.hpp>
#include <ros_tools/logging.h>
#include <string>

namespace MPCPlanner
{

/**
 * @brief 动力学模型类型
 */
enum class ModelType
{
    UNKNOWN,
    UNICYCLE,              // 单轮模型（ContouringSecondOrderUnicycleModel）
    UNICYCLE_WITH_SLACK,   // 带松弛变量的单轮模型
    BICYCLE,               // 自行车模型（BicycleModel2ndOrder）
    BICYCLE_CURVATURE_AWARE // 曲率感知自行车模型
};

/**
 * @brief 模型检测器：自动识别当前使用的动力学模型
 */
class ModelDetector
{
public:
    ModelDetector()
    {
        // 使用 SYSTEM_CONFIG_PATH_INCLUDE 宏来定位配置文件
        loadConfigYaml(SYSTEM_CONFIG_PATH_INCLUDE(__FILE__, "model_map"), model_map_);
        loadConfigYaml(SYSTEM_CONFIG_PATH_INCLUDE(__FILE__, "solver_settings"), solver_settings_);
        detectModel();
    }

    /**
     * @brief 获取模型类型
     */
    ModelType getModelType() const { return model_type_; }

    /**
     * @brief 检查是否有指定的状态变量
     */
    bool hasState(const std::string &state_name) const
    {
        return model_map_[state_name].IsDefined() && 
               model_map_[state_name][0].as<std::string>() == "x";
    }

    /**
     * @brief 检查是否有指定的控制输入
     */
    bool hasInput(const std::string &input_name) const
    {
        return model_map_[input_name].IsDefined() && 
               model_map_[input_name][0].as<std::string>() == "u";
    }

    /**
     * @brief 获取状态数量
     */
    int getNumStates() const { return solver_settings_["nx"].as<int>(); }

    /**
     * @brief 获取控制输入数量
     */
    int getNumInputs() const { return solver_settings_["nu"].as<int>(); }

    /**
     * @brief 获取模型名称
     */
    std::string getModelName() const
    {
        switch (model_type_)
        {
        case ModelType::UNICYCLE:
            return "ContouringSecondOrderUnicycleModel";
        case ModelType::UNICYCLE_WITH_SLACK:
            return "ContouringSecondOrderUnicycleModelWithSlack";
        case ModelType::BICYCLE:
            return "BicycleModel2ndOrder";
        case ModelType::BICYCLE_CURVATURE_AWARE:
            return "BicycleModel2ndOrderCurvatureAware";
        default:
            return "Unknown";
        }
    }

    /**
     * @brief 打印模型信息
     */
    void printModelInfo() const
    {
        LOG_INFO("========================================");
        LOG_INFO("动力学模型信息");
        LOG_INFO("========================================");
        LOG_INFO("模型类型: " << getModelName());
        LOG_INFO("状态数量: " << getNumStates());
        LOG_INFO("控制输入数量: " << getNumInputs());
        LOG_INFO("----------------------------------------");
        LOG_INFO("状态变量:");
        
        std::vector<std::string> state_names = {"x", "y", "psi", "v", "delta", "spline", "slack"};
        for (const auto &name : state_names)
        {
            if (hasState(name))
                LOG_INFO("  ✓ " << name);
        }
        
        LOG_INFO("控制输入:");
        std::vector<std::string> input_names = {"a", "w", "slack"};
        for (const auto &name : input_names)
        {
            if (hasInput(name))
                LOG_INFO("  ✓ " << name);
        }
        LOG_INFO("========================================");
    }

    /**
     * @brief 获取轴距（仅自行车模型）
     */
    double getWheelBase() const
    {
        if (model_type_ == ModelType::BICYCLE || 
            model_type_ == ModelType::BICYCLE_CURVATURE_AWARE)
        {
            return 2.79;  // 与 solver_model.py 中的 BicycleModel2ndOrder 一致
        }
        return 0.0;
    }

    /**
     * @brief 获取最大转向角（仅自行车模型）
     */
    double getMaxSteeringAngle() const
    {
        if (hasState("delta") && model_map_["delta"].size() >= 4)
        {
            return model_map_["delta"][3].as<double>();  // 上界
        }
        return 0.55;  // 默认值
    }

private:
    void detectModel()
    {
        const int nx = getNumStates();
        const int nu = getNumInputs();
        const bool has_delta = hasState("delta");
        const bool has_slack_state = hasState("slack");
        const bool has_slack_input = hasInput("slack");

        // 根据状态和输入数量判断模型类型
        if (has_delta)
        {
            // 自行车模型
            if (nx == 6 && nu == 3)
            {
                model_type_ = ModelType::BICYCLE;
            }
            else
            {
                model_type_ = ModelType::BICYCLE_CURVATURE_AWARE;
            }
        }
        else
        {
            // 单轮模型
            if (has_slack_state)
            {
                model_type_ = ModelType::UNICYCLE_WITH_SLACK;
            }
            else if (nx == 5 && nu == 2)
            {
                model_type_ = ModelType::UNICYCLE;
            }
            else
            {
                model_type_ = ModelType::UNKNOWN;
            }
        }
    }

    YAML::Node model_map_;
    YAML::Node solver_settings_;
    ModelType model_type_{ModelType::UNKNOWN};
};

} // namespace MPCPlanner

#endif // MPC_PLANNER_MODEL_DETECTOR_H


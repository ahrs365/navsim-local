#include <guidance_planner/config.h>

#include <ros_tools/logging.h>

#include <yaml-cpp/yaml.h>
#include <fstream>

namespace GuidancePlanner
{
  // Need to be initialized outside of a member function
  bool Config::debug_output_ = false;
  bool Config::debug_visuals_ = false;
  double Config::DT = 0.05;
  double Config::CONTROL_DT = 0.0;
  int Config::N = 20;

  bool Config::use_non_passing_ = false;
  bool Config::use_dubins_path_ = false;
  double Config::reference_velocity_ = 2.0;
  double Config::turning_radius_ = 0.5;

  Config::Config()
  {
    // Load YAML configuration file
    std::string config_file = "config/params.yaml";

    // Try to find the config file in common locations
    std::vector<std::string> search_paths = {
      config_file,
      "guidance_planner/config/params.yaml",
      "../guidance_planner/config/params.yaml",
      "../../guidance_planner/config/params.yaml"
    };

    YAML::Node config;
    bool found = false;
    for (const auto& path : search_paths) {
      std::ifstream file(path);
      if (file.good()) {
        config = YAML::LoadFile(path);
        found = true;
        LOG_INFO("Loaded guidance_planner config from: " + path);
        break;
      }
    }

    if (!found) {
      LOG_WARN("Could not find guidance_planner config file, using default values");
      return;
    }

    YAML::Node gp = config["guidance_planner"];

    // Debug settings
    Config::debug_output_ = gp["debug"]["output"].as<bool>(false);
    Config::debug_visuals_ = gp["debug"]["visuals"].as<bool>(false);

    // High-level settings
    T_ = gp["T"].as<double>(4.0);
    Config::N = gp["N"].as<int>(20);
    Config::DT = T_ / (double)Config::N;

    // Control frequency (default 10 Hz)
    Config::CONTROL_DT = 1.0 / 10.0;

    // Sampling settings
    seed_ = gp["seed"].as<int>(1);
    n_samples_ = gp["sampling"]["n_samples"].as<int>(50);
    timeout_ = gp["sampling"]["timeout"].as<double>(10.0);
    sample_margin_ = gp["sampling"]["margin"].as<double>(0.0);

    // Homotopy settings
    n_paths_ = gp["homotopy"]["n_paths"].as<int>(4);
    track_selected_homology_only_ = gp["homotopy"]["track_selected_homology_only"].as<bool>(false);
    topology_comparison_function_ = gp["homotopy"]["comparison_function"].as<std::string>("Homology");
    winding_pass_threshold_ = gp["homotopy"]["winding"]["pass_threshold"].as<double>(0.25);
    Config::use_non_passing_ = gp["homotopy"]["winding"]["use_non_passing"].as<bool>(false);
    use_learning = gp["homotopy"]["use_learning"].as<bool>(false);

    // Predictions
    assume_constant_velocity_ = gp["predictions_are_constant_velocity"].as<bool>(true);

    // Dynamics
    connection_type_ = gp["dynamics"]["connections"].as<std::string>("Straight");
    Config::use_dubins_path_ = (connection_type_ == "Dubins");
    Config::turning_radius_ = gp["dynamics"]["turning_radius"].as<double>(0.5);

    // Goals
    longitudinal_goals_ = gp["goals"]["longitudinal"].as<int>(3);
    vertical_goals_ = gp["goals"]["vertical"].as<int>(3);

    // Velocity and acceleration limits
    max_velocity_ = gp["max_velocity"].as<double>(3.0);
    max_acceleration_ = gp["max_acceleration"].as<double>(3.0);

    // Connection filters
    enable_forward_filter_ = gp["connection_filters"]["forward"].as<bool>(false);
    enable_velocity_filter_ = gp["connection_filters"]["velocity"].as<bool>(true);
    enable_acceleration_filter_ = gp["connection_filters"]["acceleration"].as<bool>(true);

    // Spline optimization
    optimize_splines_ = gp["spline_optimization"]["enable"].as<bool>(true);
    geometric_weight_ = gp["spline_optimization"]["geometric"].as<double>(25.0);
    smoothness_weight_ = gp["spline_optimization"]["smoothness"].as<double>(10.0);
    collision_weight_ = gp["spline_optimization"]["collision"].as<double>(0.5);
    velocity_tracking_ = gp["spline_optimization"]["velocity_tracking"].as<double>(0.01);

    // Selection weights
    selection_weight_length_ = gp["selection_weights"]["length"].as<double>(1.0);
    selection_weight_velocity_ = gp["selection_weights"]["velocity"].as<double>(0.0);
    selection_weight_acceleration_ = gp["selection_weights"]["acceleration"].as<double>(0.0);
    selection_weight_consistency_ = gp["selection_weights"]["consistency"].as<double>(0.0);

    // Number of points for spline optimization
    num_points_ = gp["spline_optimization"]["num_points"].as<int>(-1);
    if (num_points_ == -1)
      num_points_ = N;

    // Visualization settings
    visuals_transparency_ = gp["visuals"]["transparency"].as<double>(0.97);
    visualize_all_samples_ = gp["visuals"]["visualize_all_samples"].as<bool>(false);
    visualize_homology_ = gp["visuals"]["visualize_homology"].as<bool>(false);
    show_trajectory_indices_ = gp["visuals"]["show_indices"].as<bool>(false);

    // Enable flags
    dynamically_propagate_nodes_ = gp["enable"]["dynamically_propagate_nodes"].as<bool>(true);
    project_from_obstacles_ = gp["enable"]["project_from_obstacles"].as<bool>(false);

    // Test node settings
    debug_continuous_replanning_ = gp["test_node"]["continuous_replanning"].as<bool>(true);
  }
}

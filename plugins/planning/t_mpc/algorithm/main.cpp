#include <ros_tools/profiling.h>
#include <ros_tools/logging.h>

#include <mpc_planner/planner.h>
#include <mpc_planner/data_preparation.h>
#include <mpc_planner_solver/state.h>
#include <mpc_planner_solver/model_detector.h>
#include <mpc_planner_types/realtime_data.h>
#include <mpc_planner_util/parameters.h>
#include <guidance_planner/global_guidance.h>

#include <mpc_planner_types/data_types.h>

#define WITHOUT_NUMPY
#include "matplotlibcpp.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ros_tools/spline.h>

namespace plt = matplotlibcpp;
namespace fs = std::filesystem;
using namespace MPCPlanner;

namespace
{

volatile std::sig_atomic_t g_running = 1;
constexpr double kPi = 3.14159265358979323846;

void handleSignal(int signal)
{
    (void)signal;
    g_running = 0;
}

std::pair<std::vector<double>, std::vector<double>> makeCircle(double cx, double cy, double radius, int samples = 48)
{
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(samples + 1);
    ys.reserve(samples + 1);

    for (int i = 0; i <= samples; ++i)
    {
        double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(samples);
        xs.push_back(cx + radius * std::cos(theta));
        ys.push_back(cy + radius * std::sin(theta));
    }

    return {xs, ys};
}

struct SimObstacle
{
    int id;
    Eigen::Vector2d position;
    Eigen::Vector2d velocity;
    double radius;
    bool bounce_x{false};
    bool bounce_y{true};
    double min_x{0.0};
    double max_x{0.0};
    double min_y{-1.5};
    double max_y{1.5};
    bool is_static{false};

    void update(double dt)
    {
        if (is_static)
            return;

        position += velocity * dt;

        if (bounce_x)
        {
            if (position.x() < min_x)
            {
                position.x() = min_x;
                velocity.x() = std::abs(velocity.x());
            }
            else if (position.x() > max_x)
            {
                position.x() = max_x;
                velocity.x() = -std::abs(velocity.x());
            }
        }

        if (bounce_y)
        {
            if (position.y() < min_y)
            {
                position.y() = min_y;
                velocity.y() = std::abs(velocity.y());
            }
            else if (position.y() > max_y)
            {
                position.y() = max_y;
                velocity.y() = -std::abs(velocity.y());
            }
        }
    }
};

struct GuidancePath
{
    std::vector<Eigen::Vector2d> points;
    bool selected{false};
    int color_index{-1};
};

std::string colorFromIndex(int idx)
{
    static const std::vector<std::string> palette = {
        "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
        "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
        "#bcbd22", "#17becf", "#e6550d", "#31a354"
    };

    if (idx < 0)
        return "#4c566a";
    return palette[static_cast<size_t>(idx) % palette.size()];
}

class MatplotlibVisualizer
{
public:
    MatplotlibVisualizer() = default;

    void initialize()
    {
        if (_initialized)
            return;

        matplotlibcpp::detail::_interpreter::get();
        const char *script = R"PYTHON(
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons

plt.ion()
plt.close('all')
fig = plt.figure(figsize=(9.6, 6.4))
main_ax = fig.add_subplot(2, 1, 1)
speed_ax = fig.add_subplot(2, 1, 2)
fig.subplots_adjust(top=0.85)

if not hasattr(plt, '_mpc_pause_state'):
    plt._mpc_pause_state = {'paused': False}
else:
    plt._mpc_pause_state['paused'] = bool(plt._mpc_pause_state.get('paused', False))

# Initialize visibility state for legend items
if not hasattr(plt, '_mpc_visibility'):
    plt._mpc_visibility = {}

def _mpc_pause(event):
    plt._mpc_pause_state['paused'] = True

def _mpc_start(event):
    plt._mpc_pause_state['paused'] = False

def _mpc_on_legend_pick(event):
    legline = event.artist
    label = legline.get_label()
    if label not in plt._mpc_visibility:
        plt._mpc_visibility[label] = True
    plt._mpc_visibility[label] = not plt._mpc_visibility[label]

    # Update visibility of all lines with this label
    ax = event.inaxes
    if ax is None:
        return
    for line in ax.get_lines():
        if line.get_label() == label:
            line.set_visible(plt._mpc_visibility[label])
    for patch in ax.patches:
        if patch.get_label() == label:
            patch.set_visible(plt._mpc_visibility[label])
    fig.canvas.draw_idle()

pause_ax = fig.add_axes([0.73, 0.9, 0.1, 0.05])
start_ax = fig.add_axes([0.85, 0.9, 0.1, 0.05])

pause_btn = Button(pause_ax, 'Pause')
start_btn = Button(start_ax, 'Start')
pause_btn.on_clicked(_mpc_pause)
start_btn.on_clicked(_mpc_start)

fig.canvas.mpl_connect('pick_event', _mpc_on_legend_pick)

plt._mpc_axes = {'main': main_ax, 'speed': speed_ax}

# Visibility toggles via CheckButtons (checkboxes)
try:
    labels = ['Reference Path','Ego History','Candidate Guidance Path','Selected Guidance Path',
              'MPC Candidates','MPC Best Trajectory','MPC Selected','Static Obstacle',
              'Dynamic Obstacle','Obstacle Prediction','Robot Footprint','Goal',
              'Planned Speed','History Speed']
    if not hasattr(plt, '_mpc_visibility'):
        plt._mpc_visibility = {}
    for key in labels:
        if key not in plt._mpc_visibility:
            plt._mpc_visibility[key] = True

    cb_ax = fig.add_axes([0.86, 0.58, 0.13, 0.30])
    cb_ax.set_title('显示项', fontsize=9)
    plt._mpc_checks = CheckButtons(cb_ax, labels, [plt._mpc_visibility[k] for k in labels])

    def _mpc_apply_visibility_main():
        ax = plt._mpc_axes['main']
        for line in ax.get_lines():
            lab = line.get_label()
            if lab in plt._mpc_visibility:
                line.set_visible(plt._mpc_visibility[lab])
        for patch in ax.patches:
            lab = patch.get_label()
            if lab in plt._mpc_visibility:
                patch.set_visible(plt._mpc_visibility[lab])

    def _mpc_apply_visibility_speed():
        ax = plt._mpc_axes['speed']
        for line in ax.get_lines():
            lab = line.get_label()
            if lab in plt._mpc_visibility:
                line.set_visible(plt._mpc_visibility[lab])

    def _mpc_on_check(label):
        plt._mpc_visibility[label] = not plt._mpc_visibility.get(label, True)
        _mpc_apply_visibility_main()
        _mpc_apply_visibility_speed()
        fig.canvas.draw_idle()

    plt._mpc_checks.on_clicked(_mpc_on_check)
except Exception as _e:
    pass
)PYTHON";
        PyRun_SimpleString(script);
        _initialized = true;
    }

    void resetLayout()
    {
        if (!_initialized)
            return;
        PyRun_SimpleString("import matplotlib.pyplot as plt\nplt.close('all')\n");
        _initialized = false;
    }

    void update(const State &state,
                const RealTimeData &data,
                const PlannerOutput &output,
                const std::vector<CandidateTrajectory> &candidates,
                const std::vector<double> &history_x,
                const std::vector<double> &history_y,
                const std::vector<double> &history_speed,
                const std::vector<double> &history_time,
                const std::vector<GuidancePath> &guidance_paths,
                double sim_time,
                int iteration)
    {
        if (!_initialized)
            initialize();

        if (!_reference_cache_ready && !data.reference_path.x.empty())
        {
            _ref_x = data.reference_path.x;
            _ref_y = data.reference_path.y;
            _reference_cache_ready = true;
        }

        setActiveAxis("main");
        plt::cla();


        if (!_ref_x.empty())
        {
            std::map<std::string, std::string> ref_opts;
            ref_opts["color"] = "k";
            ref_opts["linestyle"] = "--";
            ref_opts["label"] = "Reference Path";
            plt::plot(_ref_x, _ref_y, ref_opts);
        }

        if (!history_x.empty())
        {
            std::map<std::string, std::string> history_opts;
            history_opts["color"] = "b";
            history_opts["linestyle"] = "-";
            history_opts["label"] = "Ego History";

        // MPC Candidate Trajectories (from T-MPC parallel planners)
        if (!candidates.empty())
        {
            for (const auto &c : candidates)
            {
                if (!c.success || c.traj.positions.empty())
                    continue;

                std::vector<double> cx;
                std::vector<double> cy;
                cx.reserve(c.traj.positions.size());
                cy.reserve(c.traj.positions.size());
                for (const auto &p : c.traj.positions)
                {
                    cx.push_back(p.x());
                    cy.push_back(p.y());
                }

                std::map<std::string, std::string> opts;
                opts["color"] = colorFromIndex(c.color);

                if (c.is_best)
                {
                    // Best trajectory: solid, thick, bright
                    opts["linestyle"] = "-";
                    opts["linewidth"] = "3.5";
                    opts["label"] = "MPC Best Trajectory";
                }
                else
                {
                    // Other candidates: solid, bold
                    opts["linestyle"] = "-";
                    opts["linewidth"] = "2.2";
                    opts["label"] = "MPC Candidates";
                }
                plt::plot(cx, cy, opts);
            }
        }

            plt::plot(history_x, history_y, history_opts);
        }

        if (output.success && !output.trajectory.positions.empty())
        {
            std::vector<double> traj_x;
            std::vector<double> traj_y;
            traj_x.reserve(output.trajectory.positions.size());
            traj_y.reserve(output.trajectory.positions.size());

            for (const auto &p : output.trajectory.positions)
            {
                traj_x.push_back(p.x());
                traj_y.push_back(p.y());
            }

            std::map<std::string, std::string> traj_opts;
            traj_opts["color"] = "lime";
            traj_opts["linestyle"] = "-";
            traj_opts["linewidth"] = "4.0";
            traj_opts["label"] = "MPC Selected";
            plt::plot(traj_x, traj_y, traj_opts);
        }

        // Obstacles and predictions
        double min_x = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();

        auto update_bounds = [&](double x, double y) {
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
        };

        if (!_ref_x.empty())
        {
            for (size_t i = 0; i < _ref_x.size(); ++i)
                update_bounds(_ref_x[i], _ref_y[i]);
        }

        for (size_t i = 0; i < history_x.size(); ++i)
            update_bounds(history_x[i], history_y[i]);

        for (const auto &obs : data.dynamic_obstacles)
        {
            if (obs.index < 0)
                continue; // Skip dummy obstacles added for padding

            bool is_static_obstacle = (obs.type == ObstacleType::STATIC);
            auto circle = makeCircle(obs.position.x(), obs.position.y(), obs.radius);
            std::map<std::string, std::string> circle_opts;
            circle_opts["color"] = is_static_obstacle ? "#ff8c00" : "#d62728";
            circle_opts["linewidth"] = is_static_obstacle ? "1.8" : "1.4";
            if (is_static_obstacle)
            {
                circle_opts["label"] = "Static Obstacle";
            }
            else
            {
                circle_opts["label"] = "Dynamic Obstacle";
            }
            plt::plot(circle.first, circle.second, circle_opts);

            if (!is_static_obstacle && !obs.prediction.modes.empty() && !obs.prediction.modes.front().empty())
            {
                std::vector<double> pred_x;
                std::vector<double> pred_y;
                pred_x.reserve(obs.prediction.modes.front().size());
                pred_y.reserve(obs.prediction.modes.front().size());

                for (const auto &step : obs.prediction.modes.front())
                {
                    pred_x.push_back(step.position.x());
                    pred_y.push_back(step.position.y());
                }
                std::map<std::string, std::string> pred_opts;
                pred_opts["color"] = "#d62728";
                pred_opts["linestyle"] = ":";
                pred_opts["label"] = "Obstacle Prediction";
                plt::plot(pred_x, pred_y, pred_opts);

                for (size_t i = 0; i < pred_x.size(); ++i)
                    update_bounds(pred_x[i], pred_y[i]);
            }

            update_bounds(obs.position.x(), obs.position.y());
        }

        for (const auto &path : guidance_paths)
        {
            for (const auto &pt : path.points)
                update_bounds(pt.x(), pt.y());
        }

        const double state_x = state.get("x");
        const double state_y = state.get("y");
        const double psi = state.get("psi");
        const double cos_psi = std::cos(psi);
        const double sin_psi = std::sin(psi);

        for (const auto &disc : data.robot_area)
        {
            const double cx = state_x + disc.offset * cos_psi;
            const double cy = state_y + disc.offset * sin_psi;
            auto circle = makeCircle(cx, cy, disc.radius);

            std::map<std::string, std::string> disc_opts;
            disc_opts["color"] = "#4169e1";
            disc_opts["linewidth"] = "2.2";
            disc_opts["linestyle"] = "-";
            disc_opts["label"] = "Robot Footprint";
            plt::plot(circle.first, circle.second, disc_opts);

            for (size_t i = 0; i < circle.first.size(); ++i)
                update_bounds(circle.first[i], circle.second[i]);
        }

        std::vector<double> ego_x = {state_x};
        std::vector<double> ego_y = {state_y};
        std::map<std::string, std::string> ego_opts;
        ego_opts["color"] = "#0c2c84";
        ego_opts["label"] = "Ego Center";
        plt::scatter(ego_x, ego_y, 160.0, ego_opts);
        update_bounds(ego_x.front(), ego_y.front());

        if (data.goal_received)
        {
            std::vector<double> goal_x = {data.goal.x()};
            std::vector<double> goal_y = {data.goal.y()};
            std::map<std::string, std::string> goal_opts;
            goal_opts["color"] = "gold";
            goal_opts["label"] = "Goal";
            plt::scatter(goal_x, goal_y, 220.0, goal_opts);
            update_bounds(goal_x.front(), goal_y.front());
        }

        for (const auto &path : guidance_paths)
        {
            if (path.points.size() < 2)
                continue;

            std::vector<double> gx;
            std::vector<double> gy;
            gx.reserve(path.points.size());
            gy.reserve(path.points.size());

            for (const auto &pt : path.points)
            {
                gx.push_back(pt.x());
                gy.push_back(pt.y());
            }

            std::map<std::string, std::string> opts;
            opts["color"] = colorFromIndex(path.color_index);
            // Guidance paths: always dashed, thinner
            opts["linewidth"] = "1.2";
            opts["linestyle"] = "--";
            if (path.selected)
            {
                opts["label"] = "Selected Guidance Path";
            }
            else
            {
                opts["label"] = "Candidate Guidance Path";
            }

            plt::plot(gx, gy, opts);
        }

        plt::xlabel("X [m]");
        plt::ylabel("Y [m]");

        std::ostringstream title_stream;
        title_stream << "MPC Jackal-like Simulation  |  Iter: " << iteration
                     << "  Time: " << std::fixed << std::setprecision(1) << sim_time << " s";
        plt::title(title_stream.str());

        if (min_x == std::numeric_limits<double>::infinity())
        {
            min_x = -2.0;
            max_x = 2.0;
            min_y = -2.0;
            max_y = 2.0;
        }

        const double dx = std::max(1e-3, max_x - min_x);
        const double dy = std::max(1e-3, max_y - min_y);

        const double margin_x = std::max(1.0, 0.15 * dx);
        const double margin_y = std::max(1.0, 0.15 * dy);

        plt::xlim(min_x - margin_x, max_x + margin_x);
        plt::ylim(min_y - margin_y, max_y + margin_y);
        plt::axis("equal");
        plt::grid(true);

        // Apply checkbox visibility and create de-duplicated legend on main axis
        PyRun_SimpleString(R"PYTHON(
import matplotlib.pyplot as plt
ax = plt._mpc_axes['main']
if hasattr(plt, '_mpc_apply_visibility_main'):
    plt._mpc_apply_visibility_main()
handles, labels = ax.get_legend_handles_labels()
uniq_h, uniq_l = [], []
for h, l in zip(handles, labels):
    if l and l not in uniq_l:
        uniq_l.append(l); uniq_h.append(h)
ax.legend(uniq_h, uniq_l, loc='upper right')
)PYTHON");

        setActiveAxis("speed");
        plt::cla();

        _planned_speed_time.clear();
        _planned_speed_values.clear();
        if (output.success && output.trajectory.dt > 1e-6 && !output.trajectory.positions.empty())
        {
            const auto &positions = output.trajectory.positions;
            _planned_speed_time.reserve(positions.size());
            _planned_speed_values.reserve(positions.size());

            for (size_t i = 0; i < positions.size(); ++i)
            {
                _planned_speed_time.push_back(sim_time + output.trajectory.dt * static_cast<double>(i));
                if (i == 0)
                {
                    _planned_speed_values.push_back(state.get("v"));
                }
                else
                {
                    const double dx = positions[i].x() - positions[i - 1].x();
                    const double dy = positions[i].y() - positions[i - 1].y();
                    const double speed = std::sqrt(dx * dx + dy * dy) / output.trajectory.dt;
                    _planned_speed_values.push_back(speed);
                }
            }
        }

        if (!_planned_speed_time.empty())
        {
            std::map<std::string, std::string> traj_speed_opts;
            traj_speed_opts["color"] = "#2ca02c";
            traj_speed_opts["label"] = "Planned Speed";
            traj_speed_opts["linewidth"] = "1.8";
            plt::plot(_planned_speed_time, _planned_speed_values, traj_speed_opts);
        }

        if (!history_time.empty() && history_time.size() == history_speed.size())
        {
            std::map<std::string, std::string> hist_speed_opts;
            hist_speed_opts["color"] = "#1f77b4";
            hist_speed_opts["label"] = "History Speed";
            hist_speed_opts["linewidth"] = "1.6";
            plt::plot(history_time, history_speed, hist_speed_opts);
        }

        plt::xlabel("Time [s]");
        plt::ylabel("Speed [m/s]");
        plt::grid(true);

        // Apply checkbox visibility and create de-duplicated legend on speed axis
        PyRun_SimpleString(R"PYTHON(
import matplotlib.pyplot as plt
ax = plt._mpc_axes['speed']
if hasattr(plt, '_mpc_apply_visibility_speed'):
    plt._mpc_apply_visibility_speed()
handles, labels = ax.get_legend_handles_labels()
uniq_h, uniq_l = [], []
for h, l in zip(handles, labels):
    if l and l not in uniq_l:
        uniq_l.append(l); uniq_h.append(h)
ax.legend(uniq_h, uniq_l, loc='upper right')
)PYTHON");
        plt::pause(0.001);
    }

    void waitWhilePaused()
    {
        if (!_initialized)
            return;

        while (isPaused() && g_running)
        {
            plt::pause(0.05);
        }
    }

    bool isPaused() const
    {
        PyObject *plt_mod = PyImport_AddModule("matplotlib.pyplot");
        if (!plt_mod)
            return false;

        PyObject *state = PyObject_GetAttrString(plt_mod, "_mpc_pause_state");
        if (!state || !PyDict_Check(state))
        {
            Py_XDECREF(state);
            return false;
        }

        PyObject *paused_obj = PyDict_GetItemString(state, "paused");
        bool paused = paused_obj && PyObject_IsTrue(paused_obj);
        Py_DECREF(state);
        return paused;
    }

    void setActiveAxis(const std::string &axis_key)
    {
        std::string script =
            "import matplotlib.pyplot as plt\n"
            "plt.sca(plt._mpc_axes['" + axis_key + "'])\n";
        PyRun_SimpleString(script.c_str());
    }

private:
    bool _initialized{false};
    bool _reference_cache_ready{false};
    std::vector<double> _ref_x;
    std::vector<double> _ref_y;
    std::vector<double> _planned_speed_time;
    std::vector<double> _planned_speed_values;
};

class JackalLikeSimulation
{
public:
    explicit JackalLikeSimulation(const fs::path &config_dir)
    {
        loadConfiguration(config_dir);

        // 初始化模型检测器
        model_detector_ = std::make_unique<ModelDetector>();
        model_detector_->printModelInfo();

        control_frequency_ = CONFIG["control_frequency"].as<double>();
        dt_ = 1.0 / control_frequency_;
        horizon_steps_ = CONFIG["N"].as<int>();
        enable_output_ = CONFIG["enable_output"].as<bool>();
        deceleration_ = CONFIG["deceleration_at_infeasible"].as<double>();

        planner_ = std::make_unique<Planner>();
        global_guidance_ = std::make_shared<GuidancePlanner::GlobalGuidance>();
        global_guidance_->SetPlanningFrequency(control_frequency_);
        global_guidance_->DoNotPropagateNodes();

        data_.robot_area = defineRobotArea(CONFIG["robot"]["length"].as<double>(),
                                           CONFIG["robot"]["width"].as<double>(),
                                           CONFIG["n_discs"].as<int>());
        data_.past_trajectory = FixedSizeTrajectory(200);
    }

    void run()
    {
        LOG_INFO("Preparing simulation environment");
        initializeState();
        buildReferencePath();
        initializeObstacles();

        planner_->onDataReceived(data_, "reference_path");
        planner_->onDataReceived(data_, "dynamic obstacles");

        state_.set("spline", computeReferenceProgress());
        updateGuidanceTrajectories();

        history_x_.push_back(state_.get("x"));
        history_y_.push_back(state_.get("y"));
        history_speed_.push_back(state_.get("v"));
        history_time_.push_back(sim_time_);

        visualizer_.initialize();
        visualizer_.resetLayout();

        RosTools::Instrumentor::Get().BeginSession("mpc_planner_pure_cpp_demo");

        const int max_iterations = static_cast<int>(control_frequency_ * max_sim_time_);
        int iteration = 0;

        while (g_running && iteration < max_iterations)
        {
            visualizer_.waitWhilePaused();
            const auto loop_start = std::chrono::steady_clock::now();


            data_.planning_start_time = std::chrono::system_clock::now();

            state_.set("spline", computeReferenceProgress());

            updateObstacles(dt_);
            updateGuidanceTrajectories();

            auto output = planner_->solveMPC(state_, data_);

            double v_cmd{0.0};
            double w_cmd{0.0};
            auto candidates = planner_->getTMPCandidates();


            if (enable_output_ && output.success)
            {
                v_cmd = planner_->getSolution(1, "v");
                w_cmd = planner_->getSolution(0, "w");
            }
            else
            {
                const double velocity = state_.get("v");
                const double velocity_after_brake = std::max(velocity - deceleration_ * dt_, 0.0);
                v_cmd = velocity_after_brake;
                w_cmd = 0.0;
            }

            integrateState(v_cmd, w_cmd, dt_);
            state_.set("spline", computeReferenceProgress());

            sim_time_ += dt_;
            history_x_.push_back(state_.get("x"));
            history_y_.push_back(state_.get("y"));
            history_speed_.push_back(state_.get("v"));
            history_time_.push_back(sim_time_);
            data_.past_trajectory.add(state_.getPos());

            visualizer_.update(state_, data_, output, candidates, history_x_, history_y_, history_speed_, history_time_, guidance_paths_, sim_time_, iteration);

            if (objectiveReached())
            {
                LOG_INFO("Objective reached, stopping simulation");
                break;
            }

            ++iteration;

            auto loop_end = std::chrono::steady_clock::now();
            const double loop_duration = std::chrono::duration<double>(loop_end - loop_start).count();
            const double sleep_time = dt_ - loop_duration;

            if (sleep_time > 0.0)
            {
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
            }
            else
            {
                LOG_WARN_THROTTLE(200, "Control loop overrun: " << loop_duration << " s (target " << dt_ << " s)");
            }
        }

        RosTools::Instrumentor::Get().EndSession();
        LOG_INFO("Simulation finished after " << iteration << " iterations and " << sim_time_ << " seconds");
    }

private:
    void loadConfiguration(const fs::path &config_dir)
    {
        fs::path config_file = config_dir;

        if (fs::is_directory(config_dir))
        {
            config_file = config_dir / "settings.yaml";
        }

        if (!fs::exists(config_file))
        {
            std::ostringstream err;
            err << "Config file not found: " << config_file;
            throw std::runtime_error(err.str());
        }

        LOG_INFO("Loading configuration from " << config_file);
        Configuration::getInstance().initialize(config_file.string());
    }

    void initializeState()
    {
        // 基础状态（所有模型都有）
        state_.set("x", 0.0);
        state_.set("y", 0.0);
        state_.set("psi", 0.0);
        state_.set("v", 0.5);

        // 自适应初始化：根据模型检测器初始化额外的状态
        if (model_detector_->hasState("delta"))
        {
            state_.set("delta", 0.0);
            LOG_INFO("✓ 初始化 delta 状态（自行车模型）");
        }

        if (model_detector_->hasState("slack"))
        {
            state_.set("slack", 0.0);
            LOG_INFO("✓ 初始化 slack 状态");
        }

        // 控制输入初始化
        state_.set("w", 0.0);
        state_.set("a", 0.0);

        // 路径跟踪状态
        state_.set("spline", 0.0);

        reference_progress_initialized_ = false;
        reference_segment_ = -1;
        reference_parameter_ = 0.0;

        LOG_INFO("状态初始化完成");
    }

    void buildReferencePath()
    {
        const double segment_length = 20.0;
        const int points = 200;

        data_.reference_path.clear();
        data_.reference_path.x.reserve(points);
        data_.reference_path.y.reserve(points);
        data_.reference_path.psi.reserve(points);
        data_.reference_path.v.reserve(points);
        data_.reference_path.s.clear();
        data_.reference_path.s.reserve(points);

        Eigen::Vector2d prev_point(0.0, 0.0);
        for (int i = 0; i < points; ++i)
        {
            const double s = segment_length * static_cast<double>(i) / static_cast<double>(points - 1);
            const double curvature = 0.15;
            const double y = 1.5 * std::sin(curvature * 0.5 * s);

            Eigen::Vector2d current_point(s, y);

            data_.reference_path.x.push_back(s);
            data_.reference_path.y.push_back(y);
            data_.reference_path.psi.push_back(std::atan2(curvature * 0.75 * std::cos(curvature * 0.5 * s), 1.0));
            data_.reference_path.v.push_back(std::max(0.1, 1.0 + 0.3 * std::cos(0.2 * s)));

            if (i == 0)
                data_.reference_path.s.push_back(0.0);
            else
                data_.reference_path.s.push_back(data_.reference_path.s.back() + (current_point - prev_point).norm());

            prev_point = current_point;
        }

        const auto goal_index = data_.reference_path.x.size() - 1;
        data_.goal = Eigen::Vector2d(data_.reference_path.x[goal_index], data_.reference_path.y[goal_index]);
        data_.goal_received = true;

        reference_spline_ = std::make_shared<RosTools::Spline2D>(data_.reference_path.x, data_.reference_path.y);
        reference_progress_initialized_ = false;
        reference_segment_ = -1;
        reference_parameter_ = 0.0;
    }

    void initializeObstacles()
    {
        sim_obstacles_.clear();

        SimObstacle o1;
        o1.id = 0;
        o1.position = Eigen::Vector2d(6.0, 0.8);
        o1.velocity = Eigen::Vector2d(-0.3, 0.0);
        o1.radius = 0.45;
        o1.bounce_x = true;
        o1.min_x = 2.0;
        o1.max_x = 12.0;

        SimObstacle o2;
        o2.id = 1;
        o2.position = Eigen::Vector2d(10.0, -1.0);
        o2.velocity = Eigen::Vector2d(0.0, 0.4);
        o2.radius = 0.35;
        o2.bounce_y = true;
        o2.min_y = -1.5;
        o2.max_y = 1.5;

        SimObstacle o3;
        o3.id = 2;
        o3.position = Eigen::Vector2d(14.0, 0.0);
        o3.velocity = Eigen::Vector2d(-0.2, -0.35);
        o3.radius = 0.4;
        o3.bounce_x = true;
        o3.bounce_y = true;
        o3.min_x = 8.0;
        o3.max_x = 18.0;
        o3.min_y = -2.0;
        o3.max_y = 2.0;

        SimObstacle s1;
        s1.id = 3;
        s1.position = Eigen::Vector2d(4.0, 1.5);
        s1.velocity = Eigen::Vector2d::Zero();
        s1.radius = 0.35;
        s1.is_static = true;

        SimObstacle s2;
        s2.id = 4;
        s2.position = Eigen::Vector2d(8.5, -1.2);
        s2.velocity = Eigen::Vector2d::Zero();
        s2.radius = 0.4;
        s2.is_static = true;

        SimObstacle s3;
        s3.id = 5;
        s3.position = Eigen::Vector2d(11.5, 1.1);
        s3.velocity = Eigen::Vector2d::Zero();
        s3.radius = 0.35;
        s3.is_static = true;

        SimObstacle s4;
        s4.id = 6;
        s4.position = Eigen::Vector2d(16.0, -0.6);
        s4.velocity = Eigen::Vector2d::Zero();
        s4.radius = 0.45;
        s4.is_static = true;

        sim_obstacles_ = {o1, o2, o3, s1, s2, s3, s4};

        data_.dynamic_obstacles.clear();

        for (const auto &sim_obs : sim_obstacles_)
        {
            DynamicObstacle dyn(sim_obs.id, sim_obs.position, 0.0, sim_obs.radius);
            dyn.prediction = getConstantVelocityPrediction(sim_obs.position,
                                                           sim_obs.velocity,
                                                           CONFIG["integrator_step"].as<double>(),
                                                           horizon_steps_);
            if (sim_obs.is_static)
                dyn.type = ObstacleType::STATIC;
            data_.dynamic_obstacles.push_back(dyn);
        }

        ensureObstacleSize(data_.dynamic_obstacles, state_);
    }

    void updateObstacles(double dt)
    {
        const int steps = horizon_steps_;
        const double integrator_step = CONFIG["integrator_step"].as<double>();

        for (size_t i = 0; i < sim_obstacles_.size(); ++i)
        {
            auto &sim_obs = sim_obstacles_[i];
            sim_obs.update(dt);

            auto &dyn = data_.dynamic_obstacles[i];
            dyn.position = sim_obs.position;
            dyn.prediction = getConstantVelocityPrediction(sim_obs.position,
                                                           sim_obs.velocity,
                                                           integrator_step,
                                                           steps);
        }

        planner_->onDataReceived(data_, "dynamic obstacles");
    }

    void integrateState(double v, double w, double dt)
    {
        const double prev_v = state_.get("v");

        double x = state_.get("x");
        double y = state_.get("y");
        double psi = state_.get("psi");

        // 自适应模型：根据模型检测器选择不同的运动学模型
        if (model_detector_->hasState("delta"))
        {
            // ========================================
            // 自行车模型（Bicycle Model）
            // ========================================
            const double prev_delta = state_.get("delta");
            double delta = prev_delta;

            // 从模型检测器获取参数
            const double wheel_base = model_detector_->getWheelBase();
            const double max_delta = model_detector_->getMaxSteeringAngle();

            const double lr = wheel_base / 2.0;
            const double lf = wheel_base / 2.0;
            const double ratio = lr / (lr + lf);

            // 计算 beta（侧偏角）
            const double beta = std::atan(ratio * std::tan(delta));

            // 自行车模型运动学方程
            x += v * std::cos(psi + beta) * dt;
            y += v * std::sin(psi + beta) * dt;
            psi += (v / lr) * std::sin(beta) * dt;
            delta += w * dt;  // w 是转向角速度

            // 限制转向角
            if (delta > max_delta)
                delta = max_delta;
            else if (delta < -max_delta)
                delta = -max_delta;

            state_.set("delta", delta);
        }
        else
        {
            // ========================================
            // 单轮模型（Unicycle Model）
            // ========================================
            // 单轮模型运动学方程
            x += v * std::cos(psi) * dt;
            y += v * std::sin(psi) * dt;
            psi += w * dt;  // w 是角速度
        }

        // 限制航向角在 [-pi, pi]
        if (psi > kPi)
            psi -= 2.0 * kPi;
        else if (psi < -kPi)
            psi += 2.0 * kPi;

        // 更新状态
        state_.set("x", x);
        state_.set("y", y);
        state_.set("psi", psi);
        state_.set("v", v);
        state_.set("w", w);
        state_.set("a", (v - prev_v) / dt);
    }

    bool objectiveReached() const
    {
        const Eigen::Vector2d diff = state_.getPos() - data_.goal;
        return diff.norm() < 0.5;
    }

    double computeReferenceProgress()
    {
        if (!reference_spline_)
            return 0.0;

        if (!reference_progress_initialized_)
        {
            reference_spline_->initializeClosestPoint(state_.getPos(), reference_segment_, reference_parameter_);
            reference_progress_initialized_ = true;
        }
        else
        {
            reference_spline_->findClosestPoint(state_.getPos(), reference_segment_, reference_parameter_);
        }

        return reference_parameter_;
    }

    void updateGuidanceTrajectories()
    {
        guidance_paths_.clear();

        if (!global_guidance_ || !reference_spline_)
            return;

        double spline_position = computeReferenceProgress();
        double robot_radius = data_.robot_area.empty() ? 0.0 : data_.robot_area.front().radius;

        std::vector<GuidancePlanner::Obstacle> obstacles;
        obstacles.reserve(data_.dynamic_obstacles.size());

        for (const auto &obs : data_.dynamic_obstacles)
        {
            if (obs.index < 0)
                continue;

            std::vector<Eigen::Vector2d> positions;
            positions.reserve(1 + (obs.prediction.modes.empty() ? 0 : obs.prediction.modes.front().size()));
            positions.push_back(obs.position);
            if (!obs.prediction.modes.empty())
            {
                for (const auto &step : obs.prediction.modes.front())
                    positions.push_back(step.position);
            }

            obstacles.emplace_back(obs.index, positions, obs.radius + robot_radius);
        }

        global_guidance_->LoadObstacles(obstacles, {});
        global_guidance_->SetStart(state_.getPos(), state_.get("psi"), state_.get("v"));

        double reference_velocity = std::max(0.5, state_.get("v"));
        if (!data_.reference_path.x.empty())
        {
            double best_dist = std::numeric_limits<double>::infinity();
            size_t best_idx = 0;
            const double robot_x = state_.get("x");
            const double robot_y = state_.get("y");

            for (size_t i = 0; i < data_.reference_path.x.size(); ++i)
            {
                double dx = data_.reference_path.x[i] - robot_x;
                double dy = data_.reference_path.y[i] - robot_y;
                double dist = std::hypot(dx, dy);
                if (dist < best_dist)
                {
                    best_dist = dist;
                    best_idx = i;
                }
            }

            reference_velocity = std::max(0.3, data_.reference_path.v[best_idx]);
        }
        global_guidance_->SetReferenceVelocity(std::max(0.3, reference_velocity));

        double road_half_width = CONFIG["road"]["width"].as<double>() / 2.0 - robot_radius - 0.1;
        road_half_width = std::max(0.5, road_half_width);

        global_guidance_->LoadReferencePath(std::max(0.0, spline_position), reference_spline_, road_half_width, road_half_width);

        bool success = false;
        try
        {
            success = global_guidance_->Update();
        }
        catch (const std::exception &e)
        {
            LOG_WARN_THROTTLE(5000, "Guidance planner update failed: " << e.what());
            return;
        }

        if (!success || global_guidance_->NumberOfGuidanceTrajectories() == 0)
            return;

        int selected_topology = -1;
        try
        {
            auto &selected = global_guidance_->GetUsedTrajectory();
            selected_topology = selected.topology_class;
        }
        catch (...)
        {
            selected_topology = -1;
        }

        for (int i = 0; i < global_guidance_->NumberOfGuidanceTrajectories(); ++i)
        {
            auto &traj = global_guidance_->GetGuidanceTrajectory(i);
            RosTools::Spline2D &trajectory_spline = traj.spline.GetTrajectory();

            int samples = 80;
            std::vector<Eigen::Vector2d> points;
            points.reserve(samples + 1);
            double end_param = trajectory_spline.parameterLength();
            for (int k = 0; k <= samples; ++k)
            {
                double t = (samples == 0) ? 0.0 : end_param * static_cast<double>(k) / static_cast<double>(samples);
                points.push_back(trajectory_spline.getPoint(t));
            }

            GuidancePath path;
            path.points = std::move(points);
            path.color_index = traj.color_;
            path.selected = (traj.topology_class == selected_topology);
            guidance_paths_.push_back(std::move(path));
        }
    }

    std::unique_ptr<Planner> planner_;
    State state_;
    RealTimeData data_;
    MatplotlibVisualizer visualizer_;
    std::vector<SimObstacle> sim_obstacles_;

    std::vector<double> history_x_;
    std::vector<double> history_y_;
    std::vector<double> history_speed_;
    std::vector<double> history_time_;
    std::vector<GuidancePath> guidance_paths_;

    std::shared_ptr<GuidancePlanner::GlobalGuidance> global_guidance_;
    std::shared_ptr<RosTools::Spline2D> reference_spline_;
    bool reference_progress_initialized_{false};
    int reference_segment_{-1};
    double reference_parameter_{0.0};

    double control_frequency_{20.0};
    double dt_{0.05};
    int horizon_steps_{30};
    double deceleration_{3.0};
    bool enable_output_{true};

    double sim_time_{0.0};
    const double max_sim_time_{25.0};

    // 模型检测器
    std::unique_ptr<ModelDetector> model_detector_;
};

} // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::cout << "========================================\n";
    std::cout << "MPC Planner - Pure C++ Jackal Simulator\n";
    std::cout << "========================================\n";

    // 智能查找配置文件路径
    fs::path config_path;

    if (argc > 1)
    {
        // 用户指定了配置路径
        config_path = argv[1];
    }
    else
    {
        // 自动查找配置文件
        std::vector<fs::path> search_paths = {
            "mpc_planner_jackalsimulator/config",           // 从项目根目录运行
            "../mpc_planner_jackalsimulator/config",        // 从 build 目录运行
            "../../mpc_planner_jackalsimulator/config",     // 从 build/pure_cpp 运行
        };

        for (const auto &path : search_paths)
        {
            if (fs::exists(path / "settings.yaml"))
            {
                config_path = path;
                break;
            }
        }

        if (config_path.empty())
        {
            std::cerr << "\n[ERROR] Could not find configuration file!\n";
            std::cerr << "Searched in:\n";
            for (const auto &path : search_paths)
            {
                std::cerr << "  - " << fs::absolute(path) << "\n";
            }
            std::cerr << "\nPlease specify config path: " << argv[0] << " <config_path>\n";
            return 1;
        }
    }

    try
    {
        JackalLikeSimulation simulation(config_path);
        simulation.run();

        std::cout << "\nSimulation completed successfully.\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "\nFatal error: " << e.what() << '\n';
        return 1;
    }
}

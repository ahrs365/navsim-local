#ifndef _OPTIMIZER_H_
#define _OPTIMIZER_H_

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

#include "esdf_map.hpp"
#include "../jps_data_structures.hpp"
#include "trajectory.hpp"
#include "minco.hpp"
#include "lbfgs.hpp"

namespace JPS {

// Forward declaration
struct FlatTrajData;
struct OptimizerConfig;

struct PenaltyWeights{
    double time_weight;
    double time_weight_backup_for_replan;
    double acc_weight;
    double domega_weight;
    double collision_weight;
    double moment_weight;
    double mean_time_weight;
    double cen_acc_weight;
};

// For trajectory pre-processing
struct PathpenaltyWeights{
    double time_weight;
    double bigpath_sdf_weight;
    double mean_time_weight;
    double moment_weight;
    double acc_weight;
    double domega_weight;
};

// For trajectory pre-processing
struct PathLbfgsParams{
    lbfgs::lbfgs_parameter_t path_lbfgs_params;
    double normal_past;
    double shot_path_past;
    double shot_path_horizon;
};


class MSPlanner
{
private:
    OptimizerConfig config_;
    std::shared_ptr<navsim::perception::ESDFMap> map_;

    // optimizer parameters
    double mean_time_lowBound_;
    double mean_time_uppBound_;
    double smoothEps;// for smoothL1
    PenaltyWeights penaltyWt;
    Eigen::Vector2d energyWeights;
    lbfgs::lbfgs_parameter_t lbfgs_params_;
    
    double finalMinSafeDis;
    int finalSafeDisCheckNum;
    int safeReplanMaxTime;

    Eigen::Vector3d iniStateXYTheta;
    Eigen::Vector3d finStateXYTheta;

    Eigen::Vector3d final_initStateXYTheta_;
    Eigen::Vector3d final_finStateXYTheta_;

    Eigen::VectorXd pieceTime;
    Eigen::MatrixXd Innerpoints;
    Eigen::MatrixXd iniState;
    Eigen::MatrixXd finState;
    // trajectory segments number
    int TrajNum;
    // if the traj is cutted
    bool ifCutTraj_;
    int unOccupied_traj_num_;

    std::vector<Eigen::Vector3d> inner_init_positions;

    Eigen::MatrixXd finalInnerpoints;
    Eigen::VectorXd finalpieceTime;

    minco::MINCO_S3NU Minco;
    std::vector<Eigen::Vector3d> statelist;

    PathLbfgsParams path_lbfgs_params_;
    PathpenaltyWeights PathpenaltyWt;
 

    // sampling parameters
    int sparseResolution_;
    int sparseResolution_6_;
    double timeResolution_;
    int mintrajNum_;

    int iter_num_;
    // store the gradient of the cost function
    Eigen::Matrix2Xd gradByPoints;
    Eigen::VectorXd gradByTimes;
    Eigen::MatrixX2d partialGradByCoeffs;
    Eigen::VectorXd partialGradByTimes;
    Eigen::Vector2d gradByTailStateS;
    Eigen::Vector2d FinalIntegralXYError;
    // for ALM
    Eigen::Vector2d FinalIntegralXYError_;
    // for debug, record the collision points
    std::vector<Eigen::Vector2d> collision_point;
    std::vector<Eigen::Vector2d> collision_point_;

    // unchanged auxiliary parameters in the loop
    int SamNumEachPart;
    // Simpson integration coefficients for each sampling point
    Eigen::VectorXd IntegralChainCoeff;

    // checkpoints for collision check
    std::vector<Eigen::Vector2d> check_point;
    double safeDis_, safeDis;

    // Whether to perform visualization
    bool ifprint = false;

    // Augmented Lagrangian
    Eigen::VectorXd init_EqualLambda_, init_EqualRho_, EqualRhoMax_, EqualGamma_;
    Eigen::VectorXd EqualLambda, EqualRho;
    Eigen::VectorXd EqualTolerance_;

    Eigen::VectorXd Cut_init_EqualLambda_, Cut_init_EqualRho_, Cut_EqualRhoMax_, Cut_EqualGamma_;
    Eigen::VectorXd Cut_EqualLambda, Cut_EqualRho;
    Eigen::VectorXd Cut_EqualTolerance_;

    bool hrz_limited_;
    double hrz_laser_range_dgr_;

    // Trajectory prediction resolution for get_the_predicted_state
    double trajPredictResolution_;

    bool if_visual_optimization_ = false;

public:

    // Results
    Trajectory<5, 2> final_traj_;
    // Results before collision check
    Trajectory<5, 2> optimizer_traj_;
    // Results before trajectory pre-processing
    Trajectory<5, 2> init_final_traj_;

    Eigen::Vector3d ICR_;
    bool if_standard_diff_;

    MSPlanner(const OptimizerConfig &conf, std::shared_ptr<navsim::perception::ESDFMap> map);

    // Main function of the optimizer
    bool minco_plan(const FlatTrajData &flat_traj);
    // Obtain the initial state for planning
    bool get_state(const FlatTrajData &flat_traj);
    // Optimization
    bool optimizer();
    // Result check: whether a collision occurred
    bool check_final_collision(const Trajectory<5, 2> &final_traj, const Eigen::Vector3d &start_state_XYTheta);

    template <typename EIGENVEC>
    inline void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    inline void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    static inline int earlyExit(void *instance,
                                const Eigen::VectorXd &x,
                                const Eigen::VectorXd &g,
                                const double fx,
                                const double step,
                                const int k,
                                const int ls);

    static double costFunctionCallback(void *ptr,
                                       const Eigen::VectorXd &x,
                                       Eigen::VectorXd &g);

    // Gradient for partialGradByCoeffs and partialGradByTimes
    void attachPenaltyFunctional(double &cost);

    inline void positiveSmoothedL1(const double &x, double &f, double &df);

    template <typename EIGENVEC>
    static inline void backwardGradT(const Eigen::VectorXd &tau,
                                    const Eigen::VectorXd &gradT,
                                    EIGENVEC &gradTau);
    
    static double costFunctionCallbackPath(void *ptr,
                                           const Eigen::VectorXd &x,
                                           Eigen::VectorXd &g);

    void attachPenaltyFunctionalPath(double &cost);

    Eigen::MatrixXd get_current_iniState(){
        return iniState;
    }
    Eigen::MatrixXd get_current_finState(){
        return finState;
    }
    Eigen::MatrixXd get_current_Innerpoints(){
        return finalInnerpoints;
    }
    Eigen::VectorXd get_current_finalpieceTime(){
        return finalpieceTime;
    }

    Eigen::Vector3d get_current_iniStateXYTheta(){
        return iniStateXYTheta;
    }

    void get_the_predicted_state(const double& time, Eigen::Vector3d& XYTheta, Eigen::Vector3d& VAJ, Eigen::Vector3d& OAJ);

    std::vector<Eigen::Vector3d> get_the_predicted_state_and_path(const double &start_time, const double &time,
                                                                  const Eigen::Vector3d &start_XYTheta,
                                                                  Eigen::Vector3d &XYTheta, bool &if_forward);

    inline double normlize_angle(double angle);
};

}  // namespace JPS

#endif
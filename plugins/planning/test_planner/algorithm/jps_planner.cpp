/**
 * @file jps_planner.cpp
 * @brief Implementation of JPSPlanner class
 * 
 * Migrated from ROS-based implementation to NavSim plugin system.
 * Core algorithm remains unchanged.
 */

#include "jps_planner.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace JPS;

// ============================================================================
// Constructor
// ============================================================================

JPSPlanner::JPSPlanner(std::shared_ptr<navsim::perception::ESDFMap> map)
    : map_util_(map), status_(0), if_first_point_cut_(false) {
  // Configuration will be set via setConfig()
}

// ============================================================================
// Configuration
// ============================================================================

void JPSPlanner::setConfig(const JPSConfig& config) {
  config_ = config;

  // Create GraphSearch (if not already created)
  if (!graph_search_) {
    graph_search_ = std::make_shared<GraphSearch>(map_util_, config_.safe_dis);
  } else {
    graph_search_->SetSafeDis(config_.safe_dis);
  }
}

// ============================================================================
// Main Planning Function
// ============================================================================

bool JPSPlanner::plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
  start_state_ = start;
  end_state_ = goal;

  Eigen::Vector2i start_idx = map_util_->coord2gridIndex(start.head(2));
  Eigen::Vector2i goal_idx = map_util_->coord2gridIndex(goal.head(2));

  double start_dis = map_util_->getDistanceReal(map_util_->gridIndex2coordd(start_idx)) * 0.8;
  double goal_dis = map_util_->getDistanceReal(map_util_->gridIndex2coordd(goal_idx)) * 0.8;
  double safe_dis = std::max(std::min(config_.safe_dis, start_dis), 0.0);
  safe_dis = std::max(std::min(safe_dis, goal_dis), 0.0);

  graph_search_->plan(start_idx(0), start_idx(1), goal_idx(0), goal_idx(1), true, 1e10);

  const auto path = graph_search_->getPath();
  if (path.size() < 1) {
    std::cout << "Cannot find a path from " << start.transpose() << " to " << goal.transpose()
              << " Abort!" << std::endl;
    status_ = -1;
    return false;
  }

  std::vector<Eigen::Vector2d> ps;
  for (const auto& it : path) {
    ps.push_back(map_util_->gridIndex2coordd(Eigen::Vector2i(it->x, it->y)));
  }

  raw_path_ = ps;
  std::reverse(std::begin(raw_path_), std::end(raw_path_));

  raw_path_.front() = start.head(2);
  raw_path_.back() = goal.head(2);

  // Optimize path by removing corner points
  path_ = removeCornerPts(raw_path_);
  Unoccupied_path_ = path_;

  return true;
}

// ============================================================================
// Path Optimization
// ============================================================================

void JPSPlanner::get_small_resolution_path_() {
  small_resolution_path_.clear();

  int path_size = path_.size();
  for (int i = 0; i < path_size - 1; i++) {
    Eigen::Vector2i start = map_util_->coord2gridIndex(path_[i]);
    Eigen::Vector2i end = map_util_->coord2gridIndex(path_[i + 1]);
    std::vector<Eigen::Vector2i> line = getGridsBetweenPoints2D(start, end);
    small_resolution_path_.insert(small_resolution_path_.end(), line.begin(), line.end());
  }
}

std::vector<Eigen::Vector2d> JPSPlanner::removeCornerPts(const std::vector<Eigen::Vector2d>& path) {
  if (path.size() < 2)
    return path;

  // cut zigzag segment
  std::vector<Eigen::Vector2d> optimized_path;
  Eigen::Vector2d pose1 = path[0];
  Eigen::Vector2d pose2 = path[1];
  Eigen::Vector2d prev_pose = pose1;
  optimized_path.push_back(pose1);
  double cost1, cost2, cost3;

  if (!checkLineCollision(pose1, pose2))
    cost1 = (pose1 - pose2).norm();
  else
    cost1 = std::numeric_limits<double>::infinity();

  for (unsigned int i = 1; i < path.size() - 1; i++) {
    pose1 = path[i];
    pose2 = path[i + 1];
    if (!checkLineCollision(pose1, pose2))
      cost2 = (pose1 - pose2).norm();
    else
      cost2 = std::numeric_limits<double>::infinity();

    if (!checkLineCollision(prev_pose, pose2))
      cost3 = (prev_pose - pose2).norm();
    else
      cost3 = std::numeric_limits<double>::infinity();

    if (cost3 < cost1 + cost2)
      cost1 = cost3;
    else {
      optimized_path.push_back(path[i]);
      cost1 = (pose1 - pose2).norm();
      prev_pose = pose1;
    }
  }

  optimized_path.push_back(path.back());
  return optimized_path;
}

bool JPSPlanner::checkLineCollision(const Eigen::Vector2d& start, const Eigen::Vector2d& end) {
  std::vector<Eigen::Vector2i> line =
      getGridsBetweenPoints2D(map_util_->coord2gridIndex(start), map_util_->coord2gridIndex(end));
  for (auto line_pt : line) {
    if (map_util_->isOccWithSafeDis(line_pt, graph_search_->GetSafeDis())) {
      return true;
    }
  }
  return false;
}

std::vector<Eigen::Vector2i> JPSPlanner::getGridsBetweenPoints2D(const Eigen::Vector2i& start,
                                                                  const Eigen::Vector2i& end) {
  std::vector<Eigen::Vector2i> line;

  int dx = abs(end.x() - start.x());
  int dy = abs(end.y() - start.y());
  int sx = (start.x() < end.x()) ? 1 : -1;
  int sy = (start.y() < end.y()) ? 1 : -1;
  int err = dx - dy;

  double x0 = start.x();
  double y0 = start.y();

  while (true) {
    line.emplace_back(x0, y0);
    if (x0 == end.x() && y0 == end.y()) break;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  return line;
}

// ============================================================================
// Kinodynamic Planning
// ============================================================================

void JPSPlanner::getKinoNodeWithStartPath(const std::vector<Eigen::Vector3d>& start_path,
                                          const bool if_forward,
                                          const Eigen::Vector3d& current_state_VAJ,
                                          const Eigen::Vector3d& current_state_OAJ) {
  current_state_VAJ_ = current_state_VAJ;
  current_state_OAJ_ = current_state_OAJ;

  if (start_path.size() > 0) {
    std::vector<Eigen::Vector2d> start_path_2d;
    for (auto pt : start_path) {
      start_path_2d.push_back(pt.head(2));
      // Note: Original code had a bug here (pop_back after push_back)
      // Removed the pop_back line
    }
    raw_path_.insert(raw_path_.begin(), start_path_2d.begin(), start_path_2d.end());
    start_state_ = start_path.front();
  }

  path_ = removeCornerPts(raw_path_);
  Unoccupied_path_ = path_;

  getSampleTraj();
  getTrajsWithTime();
}

void JPSPlanner::getSampleTraj() {
  Unoccupied_sample_trajs_.clear();
  double cur_theta;

  Eigen::VectorXd state5d;  // x y theta dtheta ds
  state5d.resize(5);
  state5d << start_state_.x(), start_state_.y(), start_state_.z(), 0, 0;
  Unoccupied_sample_trajs_.push_back(state5d);

  cur_theta = atan2(Unoccupied_path_[1].y() - Unoccupied_path_[0].y(),
                    Unoccupied_path_[1].x() - Unoccupied_path_[0].x());
  normalizeAngle(start_state_.z(), cur_theta);
  state5d << start_state_.x(), start_state_.y(), cur_theta, cur_theta - start_state_.z(), 0;
  Unoccupied_sample_trajs_.push_back(state5d);
  
  cur_theta = atan2(Unoccupied_path_[0].y() - Unoccupied_path_[1].y(),
                    Unoccupied_path_[0].x() - Unoccupied_path_[1].x()) + M_PI;
  normalizeAngle(start_state_.z(), cur_theta);
  state5d << start_state_.x(), start_state_.y(), cur_theta, cur_theta - start_state_.z(), 0;
  Unoccupied_sample_trajs_.push_back(state5d);  // 2

  int path_size = Unoccupied_path_.size();
  Eigen::VectorXd pt;
  for (int i = 1; i < path_size - 1; i++) {
    pt = Unoccupied_path_[i];

    state5d << pt.x(), pt.y(), Unoccupied_sample_trajs_.back()[2], 0,
        sqrt(pow(pt.x() - Unoccupied_sample_trajs_.back()[0], 2) +
             pow(pt.y() - Unoccupied_sample_trajs_.back()[1], 2));
    Unoccupied_sample_trajs_.push_back(state5d);
    
    cur_theta = atan2(Unoccupied_path_[i + 1].y() - Unoccupied_path_[i].y(),
                      Unoccupied_path_[i + 1].x() - Unoccupied_path_[i].x());
    normalizeAngle(Unoccupied_sample_trajs_.back()[2], cur_theta);
    state5d << pt.x(), pt.y(), cur_theta, cur_theta - Unoccupied_sample_trajs_.back()[2], 0;
    Unoccupied_sample_trajs_.push_back(state5d);
  }
  
  pt = Unoccupied_path_.back();
  state5d << pt.x(), pt.y(), Unoccupied_sample_trajs_.back()[2], 0,
      sqrt(pow(pt.x() - Unoccupied_sample_trajs_.back()[0], 2) +
           pow(pt.y() - Unoccupied_sample_trajs_.back()[1], 2));
  Unoccupied_sample_trajs_.push_back(state5d);

  cur_theta = end_state_.z();
  normalizeAngle(Unoccupied_sample_trajs_.back()[2], cur_theta);
  state5d << pt.x(), pt.y(), cur_theta, cur_theta - Unoccupied_sample_trajs_.back()[2], 0;
  Unoccupied_sample_trajs_.push_back(state5d);
}

void JPSPlanner::getTrajsWithTime() {
  cut_Unoccupied_sample_trajs_.clear();
  flat_traj_.UnOccupied_traj_pts.clear();
  flat_traj_.UnOccupied_positions.clear();

  double cur_t = 0;
  double cur_s = 0;
  double cur_v = current_state_VAJ_.x();
  double cur_omega = current_state_OAJ_.x();

  int traj_size = Unoccupied_sample_trajs_.size();
  for (int i = 0; i < traj_size; i++) {
    Eigen::VectorXd pt = Unoccupied_sample_trajs_[i];
    double ds = pt[4];
    double dtheta = pt[3];

    double local_time_s, local_time_theta;
    if (i == 0) {
      local_time_s = 0;
      local_time_theta = 0;
    } else {
      local_time_s = evaluateDuration(ds, cur_v, 0, config_.max_vel, config_.max_acc);
      local_time_theta = evaluateDuration(std::abs(dtheta), cur_omega, 0, config_.max_omega, config_.max_domega);
    }

    double local_time = std::max(local_time_s, local_time_theta);
    cur_t += local_time;
    cur_s += ds;

    if (cur_t > config_.traj_cut_length) {
      if_first_point_cut_ = true;
      double cut_time = config_.traj_cut_length;
      double cut_s = evaluateLength(cut_time, cur_s - ds, local_time, cur_v, 0, config_.max_vel, config_.max_acc);
      double cut_v = evaluateVel(cut_time, ds, local_time, cur_v, 0, config_.max_vel, config_.max_acc);

      double cut_theta_time = evaluteTimeOfPos(cut_s - (cur_s - ds), ds, cur_v, 0, config_.max_vel, config_.max_acc);
      double cut_theta = evaluateLength(cut_theta_time, 0, local_time, cur_omega, 0, config_.max_omega, config_.max_domega);
      if (dtheta < 0) cut_theta = -cut_theta;
      cut_theta += Unoccupied_sample_trajs_[i - 1][2];

      double cut_omega = evaluateVel(cut_theta_time, std::abs(dtheta), local_time, cur_omega, 0, config_.max_omega, config_.max_domega);
      if (dtheta < 0) cut_omega = -cut_omega;

      double cut_x = Unoccupied_sample_trajs_[i - 1][0] + cut_s * cos(cut_theta);
      double cut_y = Unoccupied_sample_trajs_[i - 1][1] + cut_s * sin(cut_theta);

      Eigen::VectorXd cut_state5d;
      cut_state5d.resize(5);
      cut_state5d << cut_x, cut_y, cut_theta, cut_theta - Unoccupied_sample_trajs_[i - 1][2], cut_s - (cur_s - ds);
      cut_Unoccupied_sample_trajs_.push_back(cut_state5d);

      Eigen::Vector3d cut_traj_pt;
      cut_traj_pt << cut_theta, cut_s, cut_time;
      flat_traj_.UnOccupied_traj_pts.push_back(cut_traj_pt);

      Eigen::Vector3d cut_position;
      cut_position << cut_x, cut_y, cut_theta;
      flat_traj_.UnOccupied_positions.push_back(cut_position);

      flat_traj_.start_state.resize(2, 3);
      flat_traj_.start_state << start_state_.x(), cur_v * cos(start_state_.z()), 0,
          start_state_.y(), cur_v * sin(start_state_.z()), 0;

      flat_traj_.final_state.resize(2, 3);
      flat_traj_.final_state << cut_x, cut_v * cos(cut_theta), 0, cut_y, cut_v * sin(cut_theta), 0;

      flat_traj_.start_state_XYTheta = start_state_;
      flat_traj_.final_state_XYTheta << cut_x, cut_y, cut_theta;
      flat_traj_.if_cut = true;
      flat_traj_.UnOccupied_initT = 0;

      break;
    } else {
      Eigen::Vector3d traj_pt;
      traj_pt << pt[2], cur_s, cur_t;
      flat_traj_.UnOccupied_traj_pts.push_back(traj_pt);

      Eigen::Vector3d position;
      position << pt[0], pt[1], pt[2];
      flat_traj_.UnOccupied_positions.push_back(position);

      if (i == traj_size - 1) {
        flat_traj_.start_state.resize(2, 3);
        flat_traj_.start_state << start_state_.x(), cur_v * cos(start_state_.z()), 0,
            start_state_.y(), cur_v * sin(start_state_.z()), 0;

        flat_traj_.final_state.resize(2, 3);
        flat_traj_.final_state << end_state_.x(), 0, 0, end_state_.y(), 0, 0;

        flat_traj_.start_state_XYTheta = start_state_;
        flat_traj_.final_state_XYTheta = end_state_;
        flat_traj_.if_cut = false;
        flat_traj_.UnOccupied_initT = 0;
      }
    }

    if (i % 2 == 0) {
      cur_v = evaluateVel(local_time, ds, local_time, cur_v, 0, config_.max_vel, config_.max_acc);
    } else {
      cur_omega = evaluateVel(local_time, std::abs(dtheta), local_time, cur_omega, 0, config_.max_omega, config_.max_domega);
    }
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

void JPSPlanner::normalizeAngle(const double& ref_angle, double& angle) {
  while (angle - ref_angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle - ref_angle < -M_PI) {
    angle += 2 * M_PI;
  }
}

// ============================================================================
// Trapezoidal Velocity Profile Functions
// ============================================================================

double JPSPlanner::evaluateDuration(const double& length, const double& startV, const double& endV,
                                     const double& maxV, const double& maxA) {
  if (length < 1e-4) return 0.0;

  double acc_time = (maxV - startV) / maxA;
  double acc_dist = (maxV + startV) * acc_time / 2;

  double dec_time = (maxV - endV) / maxA;
  double dec_dist = (maxV + endV) * dec_time / 2;

  if (acc_dist + dec_dist > length) {
    double tmp_maxV = sqrt(maxA * length + (startV * startV + endV * endV) / 2);
    acc_time = (tmp_maxV - startV) / maxA;
    dec_time = (tmp_maxV - endV) / maxA;
    return acc_time + dec_time;
  } else {
    double uniform_dist = length - acc_dist - dec_dist;
    double uniform_time = uniform_dist / maxV;
    return acc_time + uniform_time + dec_time;
  }
}

double JPSPlanner::evaluateLength(const double& curt, const double& locallength, const double& localtime,
                                   const double& startV, const double& endV, const double& maxV,
                                   const double& maxA) {
  if (locallength < 1e-4) return 0.0;

  double acc_time = (maxV - startV) / maxA;
  double acc_dist = (maxV + startV) * acc_time / 2;

  double dec_time = (maxV - endV) / maxA;
  double dec_dist = (maxV + endV) * dec_time / 2;

  if (acc_dist + dec_dist > locallength) {
    double tmp_maxV = sqrt(maxA * locallength + (startV * startV + endV * endV) / 2);
    acc_time = (tmp_maxV - startV) / maxA;
    dec_time = (tmp_maxV - endV) / maxA;

    if (curt < acc_time) {
      return (startV + maxA * curt) * curt / 2;
    } else {
      return acc_dist + (tmp_maxV - maxA * (curt - acc_time)) * (curt - acc_time) / 2;
    }
  } else {
    double uniform_dist = locallength - acc_dist - dec_dist;
    double uniform_time = uniform_dist / maxV;

    if (curt < acc_time) {
      return (startV + maxA * curt) * curt / 2;
    } else if (curt < acc_time + uniform_time) {
      return acc_dist + maxV * (curt - acc_time);
    } else {
      return acc_dist + uniform_dist + (maxV - maxA * (curt - acc_time - uniform_time)) *
                                            (curt - acc_time - uniform_time) / 2;
    }
  }
}

double JPSPlanner::evaluateVel(const double& curt, const double& locallength, const double& localtime,
                                const double& startV, const double& endV, const double& maxV,
                                const double& maxA) {
  if (locallength < 1e-4) return 0.0;

  double acc_time = (maxV - startV) / maxA;
  double acc_dist = (maxV + startV) * acc_time / 2;

  double dec_time = (maxV - endV) / maxA;
  double dec_dist = (maxV + endV) * dec_time / 2;

  if (acc_dist + dec_dist > locallength) {
    double tmp_maxV = sqrt(maxA * locallength + (startV * startV + endV * endV) / 2);
    acc_time = (tmp_maxV - startV) / maxA;
    dec_time = (tmp_maxV - endV) / maxA;

    if (curt < acc_time) {
      return startV + maxA * curt;
    } else {
      return tmp_maxV - maxA * (curt - acc_time);
    }
  } else {
    double uniform_dist = locallength - acc_dist - dec_dist;
    double uniform_time = uniform_dist / maxV;

    if (curt < acc_time) {
      return startV + maxA * curt;
    } else if (curt < acc_time + uniform_time) {
      return maxV;
    } else {
      return maxV - maxA * (curt - acc_time - uniform_time);
    }
  }
}

double JPSPlanner::evaluteTimeOfPos(const double& pos, const double& locallength, const double& startV,
                                     const double& endV, const double& maxV, const double& maxA) {
  if (locallength < 1e-4) return 0.0;

  double acc_time = (maxV - startV) / maxA;
  double acc_dist = (maxV + startV) * acc_time / 2;

  double dec_time = (maxV - endV) / maxA;
  double dec_dist = (maxV + endV) * dec_time / 2;

  if (acc_dist + dec_dist > locallength) {
    double tmp_maxV = sqrt(maxA * locallength + (startV * startV + endV * endV) / 2);
    acc_time = (tmp_maxV - startV) / maxA;
    dec_time = (tmp_maxV - endV) / maxA;
    acc_dist = (tmp_maxV + startV) * acc_time / 2;

    if (pos < acc_dist) {
      return (sqrt(startV * startV + 2 * maxA * pos) - startV) / maxA;
    } else {
      return acc_time + (tmp_maxV - sqrt(tmp_maxV * tmp_maxV - 2 * maxA * (pos - acc_dist))) / maxA;
    }
  } else {
    double uniform_dist = locallength - acc_dist - dec_dist;
    double uniform_time = uniform_dist / maxV;

    if (pos < acc_dist) {
      return (sqrt(startV * startV + 2 * maxA * pos) - startV) / maxA;
    } else if (pos < acc_dist + uniform_dist) {
      return acc_time + (pos - acc_dist) / maxV;
    } else {
      return acc_time + uniform_time +
             (maxV - sqrt(maxV * maxV - 2 * maxA * (pos - acc_dist - uniform_dist))) / maxA;
    }
  }
}

bool JPSPlanner::JPS_check_if_collision(const Eigen::Vector2d& pos) {
  return map_util_->getDistanceReal(pos) < config_.safe_dis;
}


#ifndef ESDF_MAP_HPP
#define ESDF_MAP_HPP

#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

namespace navsim {
namespace perception {

/**
 * @brief ESDFMap 类 - 封装 ESDF 算法和 SDFmap 兼容接口
 * 
 * 本类提供完整的 SDFmap 接口，用于 JPS 规划器的无缝集成。
 * 包含：
 * - 坐标转换函数
 * - 碰撞检测函数
 * - 距离场查询函数
 * - ESDF 计算算法
 */
class ESDFMap {
public:
  // ========== 枚举定义 ==========
  enum CellState : uint8_t {
    Unknown = 0,
    Unoccupied = 1,
    Occupied = 2
  };

  // ========== 配置结构 ==========
  struct Config {
    double resolution = 0.1;        // 栅格分辨率（米）
    double map_width = 30.0;        // 地图宽度（米）
    double map_height = 30.0;       // 地图高度（米）
    double max_distance = 5.0;      // 最大距离（米）
  };

  // ========== 构造/析构 ==========
  ESDFMap();
  ~ESDFMap() = default;

  // ========== 初始化 ==========
  /**
   * @brief 初始化 ESDFMap
   * @param config 配置参数
   */
  void initialize(const Config& config);

  // ========== 主要功能 ==========
  /**
   * @brief 从占据栅格构建地图
   * @param occupancy_grid 占据栅格数据（0=自由, 100=占据）
   * @param origin 地图原点（世界坐标）
   */
  void buildFromOccupancyGrid(const std::vector<uint8_t>& occupancy_grid,
                              const Eigen::Vector2d& origin);

  /**
   * @brief 计算 ESDF
   */
  void computeESDF();

  // ========== SDFmap 兼容接口 - 坐标转换 ==========
  /**
   * @brief 栅格索引 → 世界坐标
   */
  inline Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i &index) const;
  inline Eigen::Vector2d gridIndex2coordd(const int &x, const int &y) const;

  /**
   * @brief 世界坐标 → 栅格索引
   */
  inline Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt) const;

  /**
   * @brief ESDF 坐标 → 栅格索引（带 0.5 偏移）
   */
  inline Eigen::Vector2i ESDFcoord2gridIndex(const Eigen::Vector2d &pt) const;

  // ========== SDFmap 兼容接口 - 索引转换 ==========
  /**
   * @brief 2D 索引 → 1D 索引
   */
  inline int Index2Vectornum(const int &x, const int &y) const;
  inline int Index2Vectornum(const Eigen::Vector2i &index) const;

  /**
   * @brief 1D 索引 → 2D 索引
   */
  inline Eigen::Vector2i vectornum2gridIndex(const int &num) const;

  // ========== SDFmap 兼容接口 - 碰撞检测 ==========
  /**
   * @brief 检查栅格是否被占据
   */
  inline bool isOccupied(const Eigen::Vector2i &index) const;
  inline bool isOccupied(const int &idx, const int &idy) const;

  /**
   * @brief 检查栅格是否自由
   */
  inline bool isUnOccupied(const int &idx, const int &idy) const;
  inline bool isUnOccupied(const Eigen::Vector2i &index) const;

  /**
   * @brief 检查栅格是否未知
   */
  inline bool isUnknown(const Eigen::Vector2i &index) const;
  inline bool isUnknown(const int &idx, const int &idy) const;

  /**
   * @brief 检查栅格是否在安全距离内被占据
   */
  inline bool isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis) const;
  inline bool isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis) const;

  /**
   * @brief 按世界坐标检查碰撞状态
   */
  inline uint8_t CheckCollisionBycoord(const Eigen::Vector2d &pt) const;
  inline uint8_t CheckCollisionBycoord(const double ptx, const double pty) const;

  // ========== SDFmap 兼容接口 - 距离场 ==========
  /**
   * @brief 获取世界坐标点的距离场值
   */
  inline double getDistanceReal(const Eigen::Vector2d& pos) const;

  /**
   * @brief 获取栅格索引的距离场值
   */
  inline double getDistance(const Eigen::Vector2i& id) const;
  inline double getDistance(const int& idx, const int& idy) const;

  /**
   * @brief 双线性插值获取距离场值和梯度
   */
  double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad) const;
  double getDistWithGradBilinear(const Eigen::Vector2d &pos, Eigen::Vector2d& grad, const double &mindis) const;
  double getDistWithGradBilinear(const Eigen::Vector2d &pos) const;

  // ========== SDFmap 兼容接口 - 地图边界 ==========
  /**
   * @brief 检查世界坐标点是否在地图范围内
   */
  inline bool isInGloMap(const Eigen::Vector2d &pt) const;

  /**
   * @brief 获取最近的地图内点
   */
  Eigen::Vector2d closetPointInMap(const Eigen::Vector2d &pt, const Eigen::Vector2d &pos) const;

  // ========== SDFmap 兼容接口 - 工具函数 ==========
  /**
   * @brief Bresenham 直线算法，获取两点之间的所有栅格
   */
  std::vector<Eigen::Vector2i> getGridsBetweenPoints2D(const Eigen::Vector2i &start, const Eigen::Vector2i &end) const;

  /**
   * @brief 角度归一化到 [-π, π]
   */
  inline double normalize_angle(double angle) const;

  // ========== 公有成员变量（SDFmap 兼容） ==========
  int GLX_SIZE_ = 0;              // 全局地图宽度（栅格数）
  int GLY_SIZE_ = 0;              // 全局地图高度（栅格数）
  int GLXY_SIZE_ = 0;             // 全局地图总栅格数
  double grid_interval_ = 0.1;    // 栅格分辨率（米）
  double inv_grid_interval_ = 10.0; // 栅格分辨率倒数
  double global_x_lower_ = 0.0;   // 地图 X 下界（米）
  double global_x_upper_ = 0.0;   // 地图 X 上界（米）
  double global_y_lower_ = 0.0;   // 地图 Y 下界（米）
  double global_y_upper_ = 0.0;   // 地图 Y 上界（米）

private:
  // ========== 内部数据 ==========
  std::vector<uint8_t> gridmap_;           // 占据栅格地图
  std::vector<double> distance_buffer_all_; // 距离场缓冲区
  Eigen::Vector2d origin_;                 // 地图原点（世界坐标）
  double max_distance_ = 5.0;              // 最大距离（米）

  // ========== ESDF 算法 ==========
  /**
   * @brief Felzenszwalb 距离变换（模板函数）
   */
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim_size);

  // ========== 辅助函数 ==========
  /**
   * @brief 检查栅格索引是否有效
   */
  inline bool isValidIndex(const int &idx, const int &idy) const;
  inline bool isValidIndex(const Eigen::Vector2i &index) const;
};

// ========== 内联函数实现 ==========

inline Eigen::Vector2d ESDFMap::gridIndex2coordd(const Eigen::Vector2i &index) const {
  Eigen::Vector2d pt;
  pt(0) = (static_cast<double>(index(0)) + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = (static_cast<double>(index(1)) + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
}

inline Eigen::Vector2d ESDFMap::gridIndex2coordd(const int &x, const int &y) const {
  Eigen::Vector2d pt;
  pt(0) = (static_cast<double>(x) + 0.5) * grid_interval_ + global_x_lower_;
  pt(1) = (static_cast<double>(y) + 0.5) * grid_interval_ + global_y_lower_;
  return pt;
}

inline Eigen::Vector2i ESDFMap::coord2gridIndex(const Eigen::Vector2d &pt) const {
  Eigen::Vector2i idx;
  idx(0) = static_cast<int>(std::floor((pt(0) - global_x_lower_) * inv_grid_interval_));
  idx(1) = static_cast<int>(std::floor((pt(1) - global_y_lower_) * inv_grid_interval_));
  return idx;
}

inline Eigen::Vector2i ESDFMap::ESDFcoord2gridIndex(const Eigen::Vector2d &pt) const {
  Eigen::Vector2i idx;
  idx(0) = static_cast<int>((pt(0) - global_x_lower_) * inv_grid_interval_ + 0.5);
  idx(1) = static_cast<int>((pt(1) - global_y_lower_) * inv_grid_interval_ + 0.5);
  return idx;
}

inline int ESDFMap::Index2Vectornum(const int &x, const int &y) const {
  return x + y * GLX_SIZE_;
}

inline int ESDFMap::Index2Vectornum(const Eigen::Vector2i &index) const {
  return index(0) + index(1) * GLX_SIZE_;
}

inline Eigen::Vector2i ESDFMap::vectornum2gridIndex(const int &num) const {
  Eigen::Vector2i index;
  index(0) = num % GLX_SIZE_;
  index(1) = num / GLX_SIZE_;
  return index;
}

inline bool ESDFMap::isValidIndex(const int &idx, const int &idy) const {
  return idx >= 0 && idx < GLX_SIZE_ && idy >= 0 && idy < GLY_SIZE_;
}

inline bool ESDFMap::isValidIndex(const Eigen::Vector2i &index) const {
  return isValidIndex(index(0), index(1));
}

inline bool ESDFMap::isOccupied(const Eigen::Vector2i &index) const {
  if (!isValidIndex(index)) return true; // 边界外视为占据
  return gridmap_[Index2Vectornum(index)] == Occupied;
}

inline bool ESDFMap::isOccupied(const int &idx, const int &idy) const {
  if (!isValidIndex(idx, idy)) return true; // 边界外视为占据
  return gridmap_[Index2Vectornum(idx, idy)] == Occupied;
}

inline bool ESDFMap::isUnOccupied(const int &idx, const int &idy) const {
  if (!isValidIndex(idx, idy)) return false;
  return gridmap_[Index2Vectornum(idx, idy)] == Unoccupied;
}

inline bool ESDFMap::isUnOccupied(const Eigen::Vector2i &index) const {
  if (!isValidIndex(index)) return false;
  return gridmap_[Index2Vectornum(index)] == Unoccupied;
}

inline bool ESDFMap::isUnknown(const Eigen::Vector2i &index) const {
  if (!isValidIndex(index)) return true; // 边界外视为未知
  return gridmap_[Index2Vectornum(index)] == Unknown;
}

inline bool ESDFMap::isUnknown(const int &idx, const int &idy) const {
  if (!isValidIndex(idx, idy)) return true; // 边界外视为未知
  return gridmap_[Index2Vectornum(idx, idy)] == Unknown;
}

inline bool ESDFMap::isOccWithSafeDis(const Eigen::Vector2i &index, const double &safe_dis) const {
  if (!isValidIndex(index)) return true; // 边界外视为不安全
  // ✅ FIX: getDistance() 返回米单位，safe_dis 也是米单位，直接比较
  return getDistance(index) < safe_dis;
}

inline bool ESDFMap::isOccWithSafeDis(const int &idx, const int &idy, const double &safe_dis) const {
  if (!isValidIndex(idx, idy)) return true; // 边界外视为不安全
  // ✅ FIX: getDistance() 返回米单位，safe_dis 也是米单位，直接比较
  return getDistance(idx, idy) < safe_dis;
}

inline uint8_t ESDFMap::CheckCollisionBycoord(const Eigen::Vector2d &pt) const {
  Eigen::Vector2i idx = coord2gridIndex(pt);
  if (!isValidIndex(idx)) return Occupied; // 边界外视为占据
  return gridmap_[Index2Vectornum(idx)];
}

inline uint8_t ESDFMap::CheckCollisionBycoord(const double ptx, const double pty) const {
  return CheckCollisionBycoord(Eigen::Vector2d(ptx, pty));
}

inline double ESDFMap::getDistanceReal(const Eigen::Vector2d& pos) const {
  Eigen::Vector2i idx = ESDFcoord2gridIndex(pos);
  if (!isValidIndex(idx)) return 0.0; // 边界外返回 0
  // ✅ FIX: getDistance() 已经返回米单位，不需要再乘以 grid_interval_
  return getDistance(idx);
}

inline double ESDFMap::getDistance(const Eigen::Vector2i& id) const {
  if (!isValidIndex(id)) return 0.0; // 边界外返回 0
  return distance_buffer_all_[Index2Vectornum(id)];
}

inline double ESDFMap::getDistance(const int& idx, const int& idy) const {
  if (!isValidIndex(idx, idy)) return 0.0; // 边界外返回 0
  return distance_buffer_all_[Index2Vectornum(idx, idy)];
}

inline bool ESDFMap::isInGloMap(const Eigen::Vector2d &pt) const {
  return pt(0) >= global_x_lower_ && pt(0) <= global_x_upper_ &&
         pt(1) >= global_y_lower_ && pt(1) <= global_y_upper_;
}

inline double ESDFMap::normalize_angle(double angle) const {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

} // namespace perception
} // namespace navsim

#endif // ESDF_MAP_HPP


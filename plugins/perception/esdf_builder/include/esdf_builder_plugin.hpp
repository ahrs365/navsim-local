#pragma once

#include "plugin/framework/perception_plugin_interface.hpp"
#include "plugin/data/perception_input.hpp"
#include "core/planning_context.hpp"
#include "esdf_map.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <memory>

namespace navsim {
namespace plugins {
namespace perception {

/**
 * @brief ESDF (Euclidean Signed Distance Field) 地图构建插件（重构版）
 *
 * 本插件负责：
 * 1. 从 BEV 障碍物和动态障碍物构建占据栅格
 * 2. 委托给 ESDFMap 类进行 ESDF 计算
 * 3. 提供 SDFmap 兼容接口供 JPS 规划器使用
 *
 * 架构：
 * - ESDFBuilderPlugin：插件接口层（本类）
 * - ESDFMap：算法实现层（包含所有 SDFmap 兼容函数）
 *
 * 输入：
 * - context.bev_obstacles (静态障碍物：圆形、矩形、多边形)
 * - context.dynamic_obstacles (动态障碍物)
 *
 * 输出：
 * - context.esdf_map (ESDF 距离场)
 * - 通过 getESDFMap() 提供 SDFmap 兼容接口
 */
class ESDFBuilderPlugin : public plugin::PerceptionPluginInterface {
public:
  ESDFBuilderPlugin() = default;
  ~ESDFBuilderPlugin() override = default;

  /**
   * @brief 获取插件元数据
   */
  plugin::PerceptionPluginMetadata getMetadata() const override;

  /**
   * @brief 初始化插件
   * @param config 配置参数（JSON 格式）
   *   - resolution: 栅格分辨率 (m/cell)
   *   - map_width: 地图宽度 (m)
   *   - map_height: 地图高度 (m)
   *   - max_distance: 最大距离 (m)
   *   - include_dynamic: 是否包含动态障碍物
   */
  bool initialize(const nlohmann::json& config) override;

  /**
   * @brief 处理感知数据
   * @param input 感知输入（包含 BEV 障碍物、动态障碍物等）
   * @param context 规划上下文（将 ESDF 地图存储到 context.esdf_map）
   */
  bool process(const plugin::PerceptionInput& input, planning::PlanningContext& context) override;

  /**
   * @brief 获取 ESDFMap 对象（供 JPS 规划器使用）
   * @return ESDFMap 对象的共享指针
   */
  std::shared_ptr<navsim::perception::ESDFMap> getESDFMap() const {
    return esdf_map_;
  }

  /**
   * @brief 重置插件状态
   */
  void reset() override;

private:
  // ========== 配置参数 ==========

  double resolution_ = 0.1;        // 栅格分辨率 (m/cell)
  double map_width_ = 30.0;        // 地图宽度 (m)
  double map_height_ = 30.0;       // 地图高度 (m)
  double max_distance_ = 5.0;      // 最大距离 (m)
  bool include_dynamic_ = true;    // 是否包含动态障碍物

  // ========== 核心对象（组合模式） ==========

  std::shared_ptr<navsim::perception::ESDFMap> esdf_map_;  // ESDF 地图对象

  // ========== 临时数据 ==========

  std::vector<uint8_t> occupancy_grid_;  // 临时占据栅格 (0=自由, 100=占据)
  int grid_width_ = 0;                   // 栅格宽度 (cells)
  int grid_height_ = 0;                  // 栅格高度 (cells)

  // ========== 辅助函数 ==========

  /**
   * @brief 从 BEV 障碍物构建占据栅格
   * @param input 感知输入
   * @param origin 地图原点
   */
  void buildOccupancyGrid(const plugin::PerceptionInput& input, const planning::Point2d& origin);

  /**
   * @brief 检查点是否在圆形障碍物内
   */
  bool isInCircle(double x, double y, const planning::BEVObstacles::Circle& circle) const;

  /**
   * @brief 检查点是否在矩形障碍物内
   */
  bool isInRectangle(double x, double y, const planning::BEVObstacles::Rectangle& rect) const;

  /**
   * @brief 检查点是否在多边形障碍物内（射线法）
   */
  bool isInPolygon(double x, double y, const planning::BEVObstacles::Polygon& polygon) const;

  /**
   * @brief 检查点是否在动态障碍物内
   */
  bool isInDynamicObstacle(double x, double y, const planning::DynamicObstacle& obs) const;
};

} // namespace perception
} // namespace plugins
} // namespace navsim


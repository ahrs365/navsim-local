#pragma once

#include "plugin/framework/perception_plugin_interface.hpp"
#include "core/planning_context.hpp"
#include <nlohmann/json.hpp>

namespace navsim {
namespace plugins {
namespace perception {

/**
 * @brief 栅格地图构建插件
 * 
 * 从 PerceptionInput 中的 BEV 障碍物构建栅格占据地图。
 * 这是一个基础的感知插件，为路径规划提供栅格地图表示。
 * 
 * 功能：
 * - 从 BEV 障碍物（圆形、矩形、多边形）构建栅格地图
 * - 支持障碍物膨胀（安全距离）
 * - 以自车为中心的局部地图
 * - 可配置的地图大小和分辨率
 * 
 * 输出：
 * - context.occupancy_grid - 栅格占据地图
 */
class GridMapBuilderPlugin : public plugin::PerceptionPluginInterface {
public:
  /**
   * @brief 配置参数
   */
  struct Config {
    double resolution = 0.1;        // 栅格分辨率 (m/cell)
    double map_width = 100.0;       // 地图宽度 (m)
    double map_height = 100.0;      // 地图高度 (m)
    uint8_t obstacle_cost = 100;    // 障碍物代价值
    double inflation_radius = 0.5;  // 膨胀半径 (m)
    
    /**
     * @brief 从 JSON 加载配置
     */
    static Config fromJson(const nlohmann::json& json);
  };
  
  /**
   * @brief 构造函数
   */
  GridMapBuilderPlugin();
  
  /**
   * @brief 带配置的构造函数
   */
  explicit GridMapBuilderPlugin(const Config& config);
  
  // ========== 必须实现的接口 ==========
  
  /**
   * @brief 获取插件元数据
   */
  plugin::PerceptionPluginMetadata getMetadata() const override;
  
  /**
   * @brief 初始化插件
   */
  bool initialize(const nlohmann::json& config) override;
  
  /**
   * @brief 处理感知数据
   * 
   * @param input 标准化的感知输入
   * @param context 规划上下文（输出）
   * @return 处理是否成功
   */
  bool process(const plugin::PerceptionInput& input,
               planning::PlanningContext& context) override;
  
  // ========== 可选实现的接口 ==========
  
  /**
   * @brief 重置插件状态
   */
  void reset() override;
  
  /**
   * @brief 获取统计信息
   */
  nlohmann::json getStatistics() const override;
  
  /**
   * @brief 检查插件是否可用
   */
  bool isAvailable() const override;

private:
  /**
   * @brief 添加 BEV 障碍物到栅格地图
   * 
   * @param bev_obstacles BEV 障碍物
   * @param grid 栅格地图（输出）
   */
  void addBEVObstacles(const planning::BEVObstacles& bev_obstacles,
                      planning::OccupancyGrid& grid);
  
  /**
   * @brief 添加圆形障碍物
   */
  void addCircleObstacle(const planning::BEVObstacles::Circle& circle,
                        planning::OccupancyGrid& grid);
  
  /**
   * @brief 添加矩形障碍物
   */
  void addRectangleObstacle(const planning::BEVObstacles::Rectangle& rect,
                           planning::OccupancyGrid& grid);
  
  /**
   * @brief 添加多边形障碍物
   */
  void addPolygonObstacle(const planning::BEVObstacles::Polygon& polygon,
                         planning::OccupancyGrid& grid);
  
  /**
   * @brief 膨胀障碍物
   * 
   * @param grid 栅格地图
   */
  void inflateObstacles(planning::OccupancyGrid& grid);
  
  /**
   * @brief 世界坐标转栅格坐标
   * 
   * @param world_x 世界坐标 x
   * @param world_y 世界坐标 y
   * @param grid 栅格地图
   * @param grid_x 栅格坐标 x（输出）
   * @param grid_y 栅格坐标 y（输出）
   * @return 是否在地图范围内
   */
  bool worldToGrid(double world_x, double world_y,
                  const planning::OccupancyGrid& grid,
                  int& grid_x, int& grid_y) const;
  
  /**
   * @brief 设置栅格单元的值
   * 
   * @param grid_x 栅格坐标 x
   * @param grid_y 栅格坐标 y
   * @param value 值
   * @param grid 栅格地图
   */
  void setGridCell(int grid_x, int grid_y, uint8_t value,
                  planning::OccupancyGrid& grid);
  
  // 配置参数
  Config config_;
  
  // 统计信息
  struct Statistics {
    size_t total_processed = 0;
    size_t total_obstacles = 0;
    double average_time_ms = 0.0;
  };
  Statistics stats_;
};

} // namespace perception
} // namespace plugins
} // namespace navsim


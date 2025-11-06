/**
 * @file visuals.h
 * @brief Dummy visualization interface for non-ROS builds
 * @note This is a placeholder to allow compilation without ROS
 *       All visualization functions are no-ops
 */

#ifndef ROS_TOOLS_VISUALS_H
#define ROS_TOOLS_VISUALS_H

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace RosTools
{
  enum class Colormap
  {
    BRUNO,
    VIRIDIS,
    PLASMA,
    INFERNO,
    MAGMA,
    CIVIDIS
  };

  class PointMarker
  {
  public:
    void setScale(double scale) { (void)scale; }
    void setScale(double x, double y, double z) { (void)x; (void)y; (void)z; }
    void setColorInt(int id, double alpha = 1.0, Colormap colormap = Colormap::BRUNO)
    {
      (void)id;
      (void)alpha;
      (void)colormap;
    }
    void setColor(double r, double g, double b, double a = 1.0)
    {
      (void)r; (void)g; (void)b; (void)a;
    }
    void addPointMarker(const Eigen::Vector3d &p) { (void)p; }
    void addPointMarker(const Eigen::Vector2d &p) { (void)p; }
    void addPointMarker(const Eigen::Vector3d &p, double value) { (void)p; (void)value; }
    void addPointMarker(const Eigen::Vector2d &p, double value) { (void)p; (void)value; }
  };

  class Line
  {
  public:
    void setScale(double scale) { (void)scale; }
    void setScale(double x, double y, double z) { (void)x; (void)y; (void)z; }
    void setColorInt(int id, double alpha = 1.0, Colormap colormap = Colormap::BRUNO)
    {
      (void)id;
      (void)alpha;
      (void)colormap;
    }
    void setColor(double r, double g, double b, double a = 1.0)
    {
      (void)r; (void)g; (void)b; (void)a;
    }
    void addLine(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
    {
      (void)p1;
      (void)p2;
    }
    void addLine(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
    {
      (void)p1;
      (void)p2;
    }
    void addLine(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, double value)
    {
      (void)p1;
      (void)p2;
      (void)value;
    }
    void addBrokenLine(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, double scale)
    {
      (void)p1;
      (void)p2;
      (void)scale;
    }
  };

  class Publisher
  {
  public:
    Line &getNewLine() { return dummy_line_; }
    PointMarker &getNewPointMarker(const std::string &type = "")
    {
      (void)type;
      return dummy_point_marker_;
    }
    void publish() {}

  private:
    Line dummy_line_;
    PointMarker dummy_point_marker_;
  };

  class VisualsManager
  {
  public:
    Publisher &getPublisher(const std::string &topic) 
    { 
      (void)topic; 
      return dummy_publisher_; 
    }
    
  private:
    Publisher dummy_publisher_;
  };

  // Global instance
  static VisualsManager VISUALS;

} // namespace RosTools

#endif // ROS_TOOLS_VISUALS_H


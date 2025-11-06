#ifndef VISUALIZATION_DUMMY_H
#define VISUALIZATION_DUMMY_H

#include <Eigen/Dense>
#include <string>

// 空的可视化发布器，用于无 ROS 环境
namespace RosTools
{
    // 颜色映射枚举（兼容 ros_tools）
    namespace Colormap
    {
        enum Type
        {
            BRUNO = 0,
            JET = 1,
            RAINBOW = 2
        };
    }

    // 空的标记发布器类
    class DummyMarker
    {
    public:
        void setScale(double x, double y, double z) { (void)x; (void)y; (void)z; }
        void setScale(double x, double y) { setScale(x, y, 0.0); }
        void setScale(double s) { setScale(s, s, s); }
        void setColor(double r, double g, double b, double a = 1.0) { (void)r; (void)g; (void)b; (void)a; }
        void setColorInt(int index, int max = 10, double alpha = 1.0) { (void)index; (void)max; (void)alpha; }
        void setColorInt(int index, double alpha, int colormap) { (void)index; (void)alpha; (void)colormap; }
        void setOrientation(double angle) { (void)angle; }
        void setText(const std::string &text) { (void)text; }

        void addPointMarker(const Eigen::Vector2d &point, double z = 0.0) { (void)point; (void)z; }
        void addPointMarker(const Eigen::Vector3d &point, double z = 0.0) { (void)point; (void)z; }
        void addLine(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, double z = 0.0) { (void)p1; (void)p2; (void)z; }
    };

    // 空的发布器类
    class ROSMarkerPublisher
    {
    public:
        DummyMarker& getNewPointMarker(const std::string &type = "") 
        { 
            (void)type;
            static DummyMarker marker;
            return marker;
        }
        
        DummyMarker& getNewLine() 
        { 
            static DummyMarker marker;
            return marker;
        }
        
        DummyMarker& getNewModelMarker(const std::string &model_path = "") 
        { 
            (void)model_path;
            static DummyMarker marker;
            return marker;
        }
        
        DummyMarker& getNewTextMarker() 
        { 
            static DummyMarker marker;
            return marker;
        }
        
        void publish() { /* 空实现 */ }
    };

    // 空的可视化管理器
    class VisualsManager
    {
    public:
        ROSMarkerPublisher& getPublisher(const std::string &topic_name)
        {
            (void)topic_name;
            static ROSMarkerPublisher publisher;
            return publisher;
        }
    };

    // 全局可视化管理器实例
    static VisualsManager VISUALS_INSTANCE;
    
} // namespace RosTools

// 定义 VISUALS 宏，兼容原有代码
#define VISUALS RosTools::VISUALS_INSTANCE

#endif // VISUALIZATION_DUMMY_H


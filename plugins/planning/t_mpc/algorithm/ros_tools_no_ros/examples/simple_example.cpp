#include <ros_tools/spline.h>
#include <ros_tools/math.h>
#include <ros_tools/logging.h>
#include <ros_tools/profiling.h>

#include <iostream>
#include <vector>

int main()
{
    LOG_INFO("========================================");
    LOG_INFO("ROS Tools No-ROS 示例程序");
    LOG_INFO("========================================");

    // 测试日志功能
    LOG_INFO("这是一条信息日志");
    LOG_WARN("这是一条警告日志");
    LOG_SUCCESS("这是一条成功日志");
    LOG_DEBUG("这是一条调试日志");

    LOG_DIVIDER();

    // 测试数学函数
    Eigen::Vector2d point1(0.0, 0.0);
    Eigen::Vector2d point2(3.0, 4.0);
    double dist = RosTools::distance(point1, point2);
    LOG_VALUE("两点之间的距离", dist);

    LOG_DIVIDER();

    // 测试样条曲线
    LOG_INFO("创建 2D 样条曲线...");
    std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    std::vector<double> y = {0.0, 1.0, 0.5, 1.5, 1.0, 2.0};

    RosTools::Spline2D spline(x, y);
    
    LOG_VALUE("样条段数", spline.numSegments());
    LOG_VALUE("样条总长度", spline.length());

    // 在样条上采样点
    LOG_INFO("在样条上采样点:");
    for (double t = 0.0; t <= 5.0; t += 1.0) {
        Eigen::Vector2d point = spline.getPoint(t);
        std::cout << "  t=" << t << " -> (" << point.x() << ", " << point.y() << ")" << std::endl;
    }

    LOG_DIVIDER();

    // 测试性能分析
    LOG_INFO("测试性能分析功能...");
    RosTools::Benchmarker benchmarker("样条计算");
    
    for (int i = 0; i < 1000; ++i) {
        benchmarker.start();
        for (double t = 0.0; t <= 5.0; t += 0.1) {
            spline.getPoint(t);
        }
        benchmarker.stop();
    }
    
    benchmarker.print();

    LOG_DIVIDER();
    LOG_SUCCESS("示例程序运行完成！");

    return 0;
}


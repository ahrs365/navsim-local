#include "ros/ros.h"

#include <ros_tools/visuals.h>
#include <ros_tools/logging.h>
#include <ros_tools/profiling.h>

int main(int argc, char **argv)
{
    PROFILE_FUNCTION();
    // Initialize the ROS system
    ros::init(argc, argv, "rostools_example");

    ros::NodeHandle nh;

    VISUALS.init(&nh);
    RosTools::Instrumentor::Get().BeginSession("ros_tools");

    ros::Rate rate(1.);
    while (ros::ok())
    {
        PROFILE_SCOPE("Loop");
        auto &publisher = VISUALS.getPublisher("ros_tools/example");
        auto &cube = publisher.getNewPointMarker("CUBE");
        cube.setColorInt(0, 5);                           // Color by index 0 out of 5
        cube.setScale(0.25, 0.25, 0.25);                  // Scale in all directions
        cube.addPointMarker(Eigen::Vector3d(0., 0., 0.)); // Cube position
        cube.setScale(1.0, 1.0, 1.0);                     // Scale in all directions
        cube.addPointMarker(Eigen::Vector3d(1., 0., 0.)); // Cube position

        auto &cylinder = publisher.getNewPointMarker("CYLINDER");
        cylinder.setScale(0.5, 0.5, 0.3);
        cylinder.setColorInt(3, 5);
        cylinder.addPointMarker(Eigen::Vector3d(-1, 0., 2.));

        auto &line = publisher.getNewLine();
        line.setScale(0.3, 0.3);
        line.setColor(0., 1., 0., 0.2);
        line.addLine(Eigen::Vector2d(0., 0.), Eigen::Vector2d(0., 5.), 1.);
        line.setColor(0., 1., 0., 0.5);
        line.addLine(Eigen::Vector2d(1., 0.), Eigen::Vector2d(1., 5.), 1.);
        line.setColor(0., 1., 0., 1.0);
        line.addLine(Eigen::Vector2d(2., 0.), Eigen::Vector2d(2., 5.), 1.);

        {
            PROFILE_SCOPE("Model Visual");

            auto &model = publisher.getNewModelMarker("package://ros_tools/models/walking.dae");
            model.setColorInt(1, 5);
            model.addPointMarker(Eigen::Vector3d(3., 0., 0.0));
        }

        auto &text = publisher.getNewTextMarker();
        text.setText("This is an example");
        text.setColorInt(4, 5);
        text.addPointMarker(Eigen::Vector3d(5., 0., 3.));

        LOG_INFO("Publish Markers!");
        publisher.publish();
        LOG_HOOK();

        rate.sleep();
    }

    RosTools::Instrumentor::Get().EndSession();

    return 1;
}

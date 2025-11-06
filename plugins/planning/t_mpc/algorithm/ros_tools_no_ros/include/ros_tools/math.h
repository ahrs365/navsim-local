#ifndef __ros_tools_MATH_H__
#define __ros_tools_MATH_H__

#include <Eigen/Dense>

#include <vector>

namespace RosTools
{
    double distance(const Eigen::Vector2d &a, const Eigen::Vector2d &b);

    double ExponentialQuantile(double lambda, double p);

    std::vector<double> linspace(double start, double end, int num);

    Eigen::Matrix2d rotationMatrixFromHeading(double heading);

    double Bisection(double low, double high, std::function<double(double)> func, double tol = 1e-3);

    /** @brief Haar distance between angle1 and angle2 as a safe version of angle2 - angle1*/
    double angularDifference(double angle1, double angle2);

    template <typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }

    /**
     * @brief Interpolate a variable linearly between lower and upper by a representative variable "value" that moves
     * between start and end
     *
     * @tparam T The type of variable to interpolate between
     * @param start Lower value of the domain
     * @param end Upper value of the domain
     * @param value Current value in the domain
     * @param lower Lower value of the mapped domain
     * @param upper Upper value of the mapped domain
     */
    template <class T>
    inline T InterpolateLinearly(double start, double end, double value, const T &lower, const T &upper)
    {
        double lambda = (value - start) / (end - start);
        return lambda * upper + (1. - lambda) * lower;
    }
}

#endif // __ros_tools_MATH_H__

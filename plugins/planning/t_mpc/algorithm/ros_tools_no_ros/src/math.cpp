#include "ros_tools/math.h"

namespace RosTools
{
    double distance(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
    {
        return std::sqrt((a - b).transpose() * (a - b));
    }

    // Finds the exponential CDF value at probability p (for a rate of lambda)
    double ExponentialQuantile(double lambda, double p)
    {
        return -std::log(1 - p) / lambda;
    }

    std::vector<double> linspace(double start, double end, int num)
    {
        std::vector<double> result;

        if (num == 0)
            return result;
        if (num == 1)
        {
            result.push_back(end);
            return result;
        }
        if (num == 2)
        {
            result.push_back(start);
            result.push_back(end);
        }

        double delta = (end - start) / (num - 1);

        result.push_back(start);
        for (int i = 1; i < num - 1; ++i)
            result.push_back(start + delta * i);
        result.push_back(end);

        return result;
    }

    Eigen::Matrix2d rotationMatrixFromHeading(double heading)
    {
        Eigen::Matrix2d result;
        result << std::cos(heading), std::sin(heading), -std::sin(heading), std::cos(heading);

        return result;
    }

    double angularDifference(double angle1, double angle2)
    {
        double diff = std::fmod(angle2 - angle1, 2 * M_PI);

        if (diff > M_PI)
        {
            diff -= 2 * M_PI; // Shift difference to be within [-pi, pi]
        }
        else if (diff < -M_PI)
        {
            diff += 2 * M_PI;
        }
        return diff;
    }

    double Bisection(double low, double high, std::function<double(double)> func, double tol)
    {
        if (low > high)
            throw std::runtime_error("Bisection low value was higher than the high value!");

        double value_low = func(low);

        double mid;
        double value_mid;

        for (int iterations = 0; iterations < 1000; iterations++)
        {
            mid = (low + high) / 2.0;
            value_mid = func(mid);

            if (std::abs(value_mid) < tol || (high - low) / 2.0 < tol)
                return mid;

            // Extra check because of integers
            if (high - low == 1)
                return high;

            if (RosTools::sgn(value_mid) == RosTools::sgn(value_low))
            {
                low = mid;
                value_low = value_mid;
            }
            else
            {
                high = mid;
            }
        }

        throw std::runtime_error("Bisection failed!");
    }

} // namespace RosTools
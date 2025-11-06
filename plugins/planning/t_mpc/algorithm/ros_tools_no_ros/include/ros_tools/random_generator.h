#ifndef ros_tools_RANDOM_GENERATOR_H
#define ros_tools_RANDOM_GENERATOR_H

#include <Eigen/Dense>

// #include <boost/random/mersenne_twister.hpp>
// #include <boost/random/normal_distribution.hpp>

#include <random>

namespace RosTools
{
    // Class for generating random ints/doubles
    class RandomGenerator
    {

    public:
        RandomGenerator(int seed = -1);

        double Double();
        int Int(int max);

        double Gaussian(double mean, double stddev);
        Eigen::Vector2d BivariateGaussian(const Eigen::Vector2d &mean,
                                          const double major_axis, const double minor_axis,
                                          double angle);

        static void uniformToGaussian2D(Eigen::Vector2d &uniform_variables);

    private:
        std::mt19937 rng_double_;
        std::mt19937 rng_int_;
        std::mt19937 rng_gaussian_;
        std::uniform_real_distribution<> runif_;
        double epsilon_;
    };
}
#endif
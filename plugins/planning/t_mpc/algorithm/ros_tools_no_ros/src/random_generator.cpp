#include <ros_tools/random_generator.h>

#include <ros_tools/math.h>

namespace RosTools
{

    RandomGenerator::RandomGenerator(int seed)
    {
        if (seed == -1)
        {
            rng_double_ = std::mt19937(std::random_device{}());   // Standard mersenne_twister_engine seeded with rd()
            rng_int_ = std::mt19937(std::random_device{}());      // Standard mersenne_twister_engine seeded with rd()
            rng_gaussian_ = std::mt19937(std::random_device{}()); // Standard mersenne_twister_engine seeded with rd()
        }
        else
        {
            rng_double_ = std::mt19937(seed);   // Standard mersenne_twister_engine seeded with rd()
            rng_int_ = std::mt19937(seed);      // Standard mersenne_twister_engine seeded with rd()
            rng_gaussian_ = std::mt19937(seed); // Standard mersenne_twister_engine seeded with rd()
        }
        runif_ = std::uniform_real_distribution<>(0.0, 1.0);
        epsilon_ = std::numeric_limits<double>::epsilon();
    }

    double RandomGenerator::Double()
    {
        return (double)runif_(rng_double_); //(double)distribution_(random_engine_) /
                                            //(double)std::numeric_limits<uint32_t>::max();
    }

    int RandomGenerator::Int(int max)
    {
        std::uniform_int_distribution<std::mt19937::result_type> new_dist(0, max);
        return new_dist(rng_int_);
    }

    double RandomGenerator::Gaussian(double mean, double stddev)
    {
        std::normal_distribution<> dist(mean, stddev);

        // return BivariateGaussian(Eigen::Vector2d(mean, mean), stddev, stddev, 0.)(0);
        return dist(rng_gaussian_);
    }

    void RandomGenerator::uniformToGaussian2D(Eigen::Vector2d &uniform_variables)
    {
        // Temporarily safe the first variable
        double temp_u1 = uniform_variables(0);

        // Convert the uniform variables to gaussian via Box-Muller
        uniform_variables(0) = std::sqrt(-2 * std::log(temp_u1)) * std::cos(2 * M_PI * uniform_variables(1));
        uniform_variables(1) = std::sqrt(-2 * std::log(temp_u1)) * std::sin(2 * M_PI * uniform_variables(1));
    }

    Eigen::Vector2d RandomGenerator::BivariateGaussian(const Eigen::Vector2d &mean, const double major_axis, const double minor_axis, double angle)
    {
        Eigen::Matrix<double, 2, 2> A, Sigma, R, SVD;

        // // Get the angle of the path
        R = rotationMatrixFromHeading(angle);

        // Generate uniform random numbers in 2D
        double u1, u2;
        do
        {
            u1 = runif_(rng_gaussian_);
        } while (u1 <= epsilon_);
        u2 = runif_(rng_gaussian_);
        Eigen::Vector2d uniform_samples(u1, u2);

        // Convert them to a Gaussian
        uniformToGaussian2D(uniform_samples);

        // Convert the semi axes back to gaussians
        SVD << std::pow(major_axis, 2), 0.0, 0.0, std::pow(minor_axis, 2);

        // Compute Sigma and cholesky decomposition
        Sigma = R * SVD * R.transpose();
        A = Sigma.llt().matrixL(); // ~sqrt

        return Eigen::Vector2d(A(0, 0) * uniform_samples(0) + A(0, 1) * uniform_samples(1) + mean(0),
                               A(1, 0) * uniform_samples(0) + A(1, 1) * uniform_samples(1) + mean(1));
    }
}
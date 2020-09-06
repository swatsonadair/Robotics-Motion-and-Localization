#include <random>

#include "MotionNoise.h"


/**
 * A simple class for modeling noise according to a uniform distribution
 */
MotionNoise::MotionNoise(
        double pdistance_noise_min,
        double pdistance_noise_max,
        double pangle_noise_min,
        double pangle_noise_max) {
    distance_noise_min = pdistance_noise_min;
    distance_noise_max = pdistance_noise_max;
    angle_noise_min = pangle_noise_min;
    angle_noise_max = pangle_noise_max;

    std::uniform_real_distribution<float> distribution_distance(distance_noise_min, distance_noise_max);
    d_dist = distribution_distance;

    std::uniform_real_distribution<float> distribution_angle(angle_noise_min, angle_noise_max);
    d_angle = distribution_angle;
}

/**
 * Get a sample of distance noise according to the uniform distribution
 * @return
 */
double MotionNoise::distNoise() {
    std::default_random_engine generator(std::random_device{}());
    double noise = d_dist(generator);
    return noise;
}

/**
 * Get a sample of rotation noise according to the uniform distribution
 * @return
 */
double MotionNoise::angleNoise() {
    std::default_random_engine generator(std::random_device{}());
    double noise = d_angle(generator);
    return noise;
}

/**
 * A class for modeling noise according to a normal distribution
 */
MotionNoiseNormal::MotionNoiseNormal(
        double pdistance_noise_mean,
        double pdistance_noise_std,
        double pangle_noise_mean,
        double pangle_noise_std) {
    distance_noise_mean = pdistance_noise_mean;
    distance_noise_std = pdistance_noise_std;
    angle_noise_mean = pangle_noise_mean;
    angle_noise_std = pangle_noise_std;

    std::normal_distribution<float> distribution_distance(distance_noise_mean, distance_noise_std);
    d_dist = distribution_distance;

    std::normal_distribution<float> distribution_angle(angle_noise_mean, angle_noise_std);
    d_angle = distribution_angle;
}

/**
 * Get a sample of distance noise according to the normal distribution
 * @return
 */
double MotionNoiseNormal::distNoise() {
    std::default_random_engine generator(std::random_device{}());
    double noise = d_dist(generator);
    return noise;
}

/**
 * Get a sample of rotation noise according to the normal distribution
 * @return
 */
double MotionNoiseNormal::angleNoise() {
    std::default_random_engine generator(std::random_device{}());
    double noise = d_angle(generator);
    return noise;
}

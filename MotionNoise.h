#include <random>

#ifndef ROBOTICS_MOTION_AND_LOCALIZATION_MOTIONNOISE_H
#define ROBOTICS_MOTION_AND_LOCALIZATION_MOTIONNOISE_H


class MotionNoise {

private:

    double distance_noise_min;
    double distance_noise_max;

    double angle_noise_min;
    double angle_noise_max;

    std::uniform_real_distribution<float> d_dist;
    std::uniform_real_distribution<float> d_angle;

public:

    MotionNoise(
            double pdistance_noise_min=0.,
            double pdistance_noise_max=0.,
            double pangle_noise_min=0.,
            double pangle_noise_max=0.);

    virtual double distNoise();
    virtual double angleNoise();

};

class MotionNoiseNormal : public MotionNoise {

private:

    double distance_noise_mean;
    double distance_noise_std;

    double angle_noise_mean;
    double angle_noise_std;

    std::normal_distribution<float> d_dist;
    std::normal_distribution<float> d_angle;

public:

    MotionNoiseNormal(
            double pdistance_noise_mean=0.,
            double pdistance_noise_std=0.,
            double pangle_noise_mean=0.,
            double pangle_noise_std=0.);

    double distNoise();
    double angleNoise();

};


#endif //ROBOTICS_MOTION_AND_LOCALIZATION_MOTIONNOISE_H

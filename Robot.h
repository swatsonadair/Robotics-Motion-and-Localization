#include "MotionNoise.h"

#ifndef ROBOTICS_MOTION_AND_LOCALIZATION_ROBOT_H
#define ROBOTICS_MOTION_AND_LOCALIZATION_ROBOT_H


class Robot {

private:
    double x, expX;
    double y, expY;
    double heading, expHeading;

    MotionNoise *noiseDistribution;

public:

    Robot(
            double px,
            double py,
            double pheading,
            MotionNoise &pnoiseDistribution);

    double getX();
    double getY();
    double getHeading();

    void translate(double distance);
    void rotate(double angle);
    void move(double dist, double angle);
    void moveArc(double dist, double angle);

    double locError();
    double headingError();

    void report();

protected:

    std::vector<double> arcCoordinates(double arcLength, double radians, double heading);
    double angleWrap(double radians);

};


#endif //ROBOTICS_MOTION_AND_LOCALIZATION_ROBOT_H

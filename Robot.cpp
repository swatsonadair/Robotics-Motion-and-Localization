#include <iostream>
#include <cmath>

#include "Robot.h"


Robot::Robot(
        double px,
        double py,
        double pheading,
        MotionNoise &pnoiseDistribution) {
    x = expX = px;
    y = expY = py;
    heading = expHeading = pheading;
    noiseDistribution = &pnoiseDistribution;

}

double Robot::getX() {
    return x;
}

double Robot::getY() {
    return y;
}

double Robot::getHeading() {
    return heading;
}

/**
 * Apply translation in the direction of the current heading, with noise
 * Noise is added to the distance
 * @param distance
 */
void Robot::translate(double distance) {
    double radians = angleWrap((heading * M_PI) / 180.);
    double radiansExp = angleWrap((expHeading * M_PI) / 180.);
    double displacement = distance + noiseDistribution->distNoise();

    x += displacement * cos(radians);
    y += displacement * sin(radians);

    expX += distance * cos(radiansExp);
    expY += distance * sin(radiansExp);
}

/**
 * Apply counter-clockwise rotation, with noise
 * Noise is added to the angle of rotation
 * @param angle
 */
void Robot::rotate(double angle) {
    heading += angle + noiseDistribution->angleNoise();
    expHeading += angle;
}

/**
 * Move the robot via a translation followed by a rotation
 * The robot moves by two subsequent actions: translate and rotate.
 * @param dist
 * @param angle
 */
void Robot::move(double dist, double angle) {
    translate(dist);
    rotate(angle);
}

/**
 * The robot moves along an arc of specified length and angle
 * The robot moves by simultaneously moving the specified distance while rotating the specified angle through the duration of the action.
 * Noise is added to the arc length and angle of rotation
 * @param dist
 * @param angle
 */
void Robot::moveArc(double dist, double angle) {

    if (dist == 0) {
        return move(dist, angle);
    }
    if (angle == 0) {
        return move(dist, angle);
    }

    double distNoise = noiseDistribution->distNoise();
    double angleNoise = noiseDistribution->angleNoise();
    std::vector<double> coords = arcCoordinates(dist + distNoise, angle + angleNoise, heading);
    x += coords[0];
    y += coords[1];
    heading += angle + angleNoise;

    std::vector<double> coordsExp = arcCoordinates(dist, angle, expHeading);
    expX += coordsExp[0];
    expY += coordsExp[1];
    expHeading += angle;

}

/**
 * Get the coordinates from following a curve of specified length, terminating when robot is oriented at specified angle
 * @param arcLength
 * @param angle
 * @param aheading
 * @return
 */
std::vector<double> Robot::arcCoordinates(double arcLength, double angle, double aheading) {

    double radians0 = (aheading * M_PI) / 180.;
    double radians1 = ((aheading + angle) * M_PI) / 180.;
    double theta = (angle * M_PI) / 180.;

    double displacementDistance = (arcLength * 2 * sin(abs(theta) / 2)) / abs(theta);
    double displacementAngle = (radians1 - radians0) / 2 + radians0;

    return { displacementDistance * cos(displacementAngle), displacementDistance * sin(displacementAngle) };

}

/**
 * Calculate location error as Euclidean distance between expected and actual position
 * @return
 */
double Robot::locError() {
    double dist;
    dist = pow((x - expX), 2) + pow((y - expY), 2);
    dist = sqrt(dist);
    return dist;
}

/**
 * Calculate heading error as Euclidean distance between expected and actual heading
 * @return
 */
double Robot::headingError() {
    double dist;
    dist = pow((heading - expHeading), 2);
    dist = sqrt(dist);
    return dist;
}

/**
 * Output a report of the location, heading, and errors
 */
void Robot::report() {
    std::cout << "Location (x, y, heading): " << x << " " << y << " " << heading << std::endl;
    std::cout << "Expected (x, y, heading): " << expX << " " << expY << " " << expHeading << std::endl;
    std::cout << "Error (location, heading): " << locError() << " " << headingError() << std::endl;
}

/**
 * Wrap angles onto unit circle [-pi, pi]
 * @param radians
 * @return
 */
double Robot::angleWrap(double radians) {
    if (abs(radians) == 0) {
        return radians;
    } else if (fmod(radians, M_PI) == 0) {
        return radians;
    }

    double angle = fmod(radians + M_PI, M_PI * 2);

    if (angle < 0) {
        angle += M_PI * 2;
    }
    return angle - M_PI;
}

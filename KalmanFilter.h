#include <vector>
#include <Eigen/Dense>

#ifndef ROBOTICS_MOTION_AND_LOCALIZATION_KALMANFILTER_H
#define ROBOTICS_MOTION_AND_LOCALIZATION_KALMANFILTER_H


/**
 * This is a very simple implementation of the Kalman filter that builds a model for <x, y, dx, dy, ddx, ddy>
 */
class KalmanFilter {

public:

    KalmanFilter();

    Eigen::MatrixXd X; // initial state
    Eigen::MatrixXd P; // initial uncertainty
    Eigen::MatrixXd F; // state transition
    Eigen::MatrixXd H; // measurement function
    Eigen::MatrixXd R; // measurement uncertainty
    Eigen::MatrixXd I; // identity matrix

    Eigen::MatrixXd X_2;
    Eigen::MatrixXd P_2;

    void observe(Eigen::MatrixXd Z);

    void estimate();

    std::vector<float> getEstimate();

};


#endif //ROBOTICS_MOTION_AND_LOCALIZATION_KALMANFILTER_H

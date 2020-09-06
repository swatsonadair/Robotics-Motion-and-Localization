#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {

    X.resize(6, 2);
    X <<    0, 0,
            0, 0,
            0, 0,
            0, 0,
            0, 0,
            0, 0;

    P.resize(6, 6);
    P <<    10, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 10, 0, 0,
            0, 0, 0, 0, 10, 0,
            0, 0, 0, 0, 0, 10;

    F.resize(6, 6);
    F <<    1, 0, 1, 0, 1, 0,
            0, 1, 0, 1, 0, 1,
            0, 0, 1, 0, 1, 0,
            0, 0, 0, 1, 0, 1,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

    H.resize(2, 6);
    H <<    1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0;

    R.resize(2, 2);
    R << 0.1, 0, 0, 0.1;

    I.resize(6, 6);
    I <<    1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
}

/**
 * Observe the sensor input
 * @param Z
 */
void KalmanFilter::observe(Eigen::MatrixXd Z) {

    Eigen::MatrixXd Y = Z - (H * X);
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    X_2 = X + K * Y;
    P_2 = (I - (K * H)) * P;
}

/**
 * Estimate the state space based upon the measurements
 */
void KalmanFilter::estimate() {
    X = F * X_2;
    P = F * P_2 * F.transpose();
}

std::vector<float> KalmanFilter::getEstimate() {
    std::vector<float> estCoords = { static_cast<float>(X.coeff(0, 0)), static_cast<float>(X.coeff(1, 1)) };
    return estCoords;
}

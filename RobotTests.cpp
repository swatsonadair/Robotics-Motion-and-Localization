#include <iostream>
#include <fstream>

#include "Robot.h"
#include "KalmanFilter.h"
#include "MotionNoise.h"

#include "RobotTests.h"

using namespace std;


void RobotTests::noiseless_motion_test() {

    // Select the filename
    string filename;
    cout << "Enter the filename for 'noiseless_motion_test':" << endl;
    cin >> filename;

    ifstream file(filename);

    if (!file) {
        cout << "File not found!" << endl;
        return;
    }

    // Create the robot
    MotionNoise robotNoise;
    Robot robot(0, 0, 0, robotNoise);
    robot.report();

    // Read the file and move the robot accordingly
    string str;
    while (getline(file, str)) {

        vector<float> v;
        istringstream iss(str);
        copy(istream_iterator<float>(iss), istream_iterator<float>(), back_inserter(v));

        robot.move(v[0], v[1]);
        robot.report();

    }

}

void RobotTests::linear_motion_test() {

    // Select the filename
    string filename;
    cout << "Enter the filename for 'linear_motion_test':" << endl;
    cin >> filename;

    ifstream file(filename);

    if (!file) {
        cout << "File not found!" << endl;
        return;
    }

    // Create the robot
    MotionNoiseNormal robotNoise(0, .1, 0, 3);
    Robot robot(0, 0, 0, robotNoise);
    robot.report();

    // Read the file and move the robot accordingly
    string str;
    while (getline(file, str)) {

        vector<float> v;
        istringstream iss(str);
        copy(istream_iterator<float>(iss), istream_iterator<float>(), back_inserter(v));

        robot.move(v[0], v[1]);
        robot.report();

    }

}

void RobotTests::noiseless_arc_motion_test() {

    // Select the filename
    string filename;
    cout << "Enter the filename for 'arc_motion_test':" << endl;
    cin >> filename;

    ifstream file(filename);

    if (!file) {
        cout << "File not found!" << endl;
        return;
    }

    // Create the robot
    MotionNoise robotNoise;
    Robot robot(0, 0, 0, robotNoise);
    robot.report();

    // Read the file and move the robot accordingly
    string str;
    while (getline(file, str)) {

        vector<float> v;
        istringstream iss(str);
        copy(istream_iterator<float>(iss), istream_iterator<float>(), back_inserter(v));

        cout << "====================" << endl;
        robot.moveArc(v[0], v[1]);
        robot.report();

    }

}

void RobotTests::arc_motion_test() {

    // Select the filename
    string filename;
    cout << "Enter the filename for 'arc_motion_test':" << endl;
    cin >> filename;

    ifstream file(filename);

    if (!file) {
        cout << "File not found!" << endl;
        return;
    }

    // Create the robot
    MotionNoiseNormal robotNoise(0, .1, 0, 3);
    Robot robot(0, 0, 0, robotNoise);
    robot.report();

    // Read the file and move the robot accordingly
    string str;
    while (getline(file, str)) {

        vector<float> v;
        istringstream iss(str);
        copy(istream_iterator<float>(iss), istream_iterator<float>(), back_inserter(v));

        cout << "====================" << endl;
        robot.moveArc(v[0], v[1]);
        robot.report();

    }

}

void RobotTests::kalman_test() {

    // Select the filename
    string filename;
    cout << "Enter the filename for 'kalman_test':" << endl;
    cin >> filename;

    ifstream file(filename);

    if (!file) {
        cout << "File not found!" << endl;
        return;
    }

    // Create the robot
    MotionNoiseNormal robotNoise(0, .1, 0, 1.);
    Robot robot(0, 0, 0, robotNoise);
    robot.report();

    // Create the kalman filter
    KalmanFilter kalman = KalmanFilter();
    Eigen::MatrixXd Z(2,2);

    MotionNoiseNormal sensorNoise(0, .5);

    // Read the file and move the robot accordingly
    string str;
    while (getline(file, str)) {

        vector<float> v;
        istringstream iss(str);
        copy(istream_iterator<float>(iss), istream_iterator<float>(), back_inserter(v));

        cout << "====================" << endl;

        Z << robot.getX() + sensorNoise.distNoise(), 0, 0, robot.getY() + sensorNoise.distNoise();
        kalman.observe(Z);

        robot.move(v[0], v[1]);
        robot.report();

        kalman.estimate();
        vector<float> coords = kalman.getEstimate();
        cout << "Kalman Estimate (x, y): " << coords[0] << " " << coords[1] << endl;

    }

}
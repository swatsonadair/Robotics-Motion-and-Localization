#include <iostream>

#include "RobotTests.h"

using namespace std;


int main() {

    RobotTests robotTests = RobotTests();

    /**
     * This test applies motion without noise
     * Try testing with file: "motion-input-1.txt"
     */
    robotTests.noiseless_motion_test();
    cout << "===========================================================" << endl;

    /**
     * This test applies motion with distance & rotation noise
     * Try testing with file: "motion-input-1.txt"
     */
    robotTests.linear_motion_test();
    cout << "===========================================================" << endl;

    /**
     * This test applies motion in an arc, without noise
     * Try testing with file: "motion-input-1.txt"
     */
    robotTests.noiseless_arc_motion_test();
    cout << "===========================================================" << endl;

    /**
     * This test applies motion in an arc, with noise
     * Try testing with file: "motion-input-1.txt"
     */
    robotTests.arc_motion_test();
    cout << "===========================================================" << endl;

    /**
     * This test applies motion and position estimation using a very simple Kalman filter.
     * The filter receives noisy sensory input, which directly corresponds to the robots current (x,y) location.
     * For best results that take into account small, progressive movements of the robot, test with file: "motion-input-2.txt"
     */
    robotTests.kalman_test();
    cout << "===========================================================" << endl;

    return 0;
}

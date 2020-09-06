cmake_minimum_required(VERSION 3.17)
project(Robotics-Motion-and-Localization)

set(CMAKE_CXX_STANDARD 14)

find_path(EIGEN_DIR NAMES signature_of_eigen3_matrix_library
        PATHS
        /YOUR/PATH/eigen-3.3.7
        PATH_SUFFIXES eigen3 eigen
        )
include_directories(${EIGEN_DIR})

add_executable(Robotics-Motion-and-Localization main.cpp Robot.cpp Robot.h MotionNoise.cpp MotionNoise.h KalmanFilter.cpp KalmanFilter.h RobotTests.cpp RobotTests.h)
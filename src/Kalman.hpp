#pragma once

#include <Eigen/Dense>
#include "State.hpp"

namespace Kalman {

class Filter {
    public:
        // constructor
        Filter(double odom_x, double odom_y);

        // main steps of kalman: Predicting and then Updating
        void predict(double prior_odom_x, double prior_odom_y, double odom_x,
                     double odom_y, double imu_heading);

        void update(double imu_heading, double front_sens, double left_sens,
                    double right_sens);

        Pose getEstimation() const;
        
    private:
        Eigen::Vector2d state;
        Eigen::Matrix2d cov; // covariance
        Eigen::Matrix2d noise;
        double heading = 0.0; // current heading
};
}
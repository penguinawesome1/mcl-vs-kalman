#include "Kalman.hpp"

#include <cmath>
#include <numbers>
#include <utility>
#include <vector>

namespace Kalman {

Filter::Filter(double odom_x, double odom_y) : state(odom_x, odom_y) {
  cov = 1 * Eigen::Matrix2d::Identity();           // initial uncertaitny of 1
  noise = 0.000025 * Eigen::Matrix2d::Identity();  // odom drift
}

Pose Filter::getEstimation() const { return {state(0), state(1), heading}; }

// prediction without distance sensor checking
void Filter::predict(double prior_odom_x, double prior_odom_y, double odom_x,
                     double odom_y, double imu_heading) {
  heading = imu_heading;

  double travel_dist_x = odom_x - prior_odom_x;
  double travel_dist_y = odom_y - prior_odom_y;

  // adjust current state to be altered by predicted travel dist based on odom
  // wheels + imu
  state += Eigen::Vector2d(travel_dist_x, travel_dist_y);
  cov += noise;

  state(0) = std::max(0.0, state(0));
  state(1) = std::max(0.0, state(1));
}

// updating using distance sensor checking
void Filter::update(double imu_heading, double front_sens, double left_sens,
                    double right_sens) {
  heading = imu_heading;

  // sensors match up with their location on robot
  std::vector<std::pair<double, double>> sensors = {
      {front_sens, 0.0},
      {left_sens, std::numbers::pi / 2.0},
      {right_sens, -std::numbers::pi / 2.0}};

  for (const auto& [sensor_dist, loc] : sensors) {
    double theta = imu_heading + loc;

    // ensures that the heading stays between -pi and pi like in the csv file
    while (theta > std::numbers::pi) {
      theta -= 2.0 * std::numbers::pi;
    }
    while (theta <= -std::numbers::pi) {
      theta += 2.0 * std::numbers::pi;
    }

    // distance sensors
    double dist_x = 1e9;  // initializing to large number to start
    double dist_y = 1e9;

    // determinine which wall the distance sensor hits first
    if (std::cos(theta) > 0.0001 || std::cos(theta) < -0.0001) {
      double target = (std::cos(theta) > 0) ? 12 : 0.0;
      dist_x = (target - state(0)) / std::cos(theta);
      if (dist_x < 0) {
        dist_x =
            1e9;  // default back to large num because it needs to be positive
      }
    }

    if (std::sin(theta) > 0.0001 || std::sin(theta) < -0.0001) {
      double target = (std::sin(theta) > 0) ? 12 : 0.0;
      dist_y = (target - state(1)) / std::sin(theta);
      if (dist_y < 0) {
        dist_y = 1e9;
      }
    }

    Eigen::Matrix<double, 1, 2> obs;  // setting up observation matrix

    double theoretical_dist;
    bool valid = false;
    if (dist_x < dist_y) {
      valid = true;
      theoretical_dist = dist_x;
      obs(0) = -1.0 / std::cos(theta);
      obs(1) = 0.0;
    } else if (dist_y < 1e8) {
      valid = true;
      theoretical_dist = dist_y;
      obs(0) = 0.0;
      obs(1) = -1.0 / std::sin(theta);
    }

    if (!valid) {
      continue;
    }

    double variance =
        (0.01 + (sensor_dist * 0.01)) * (0.01 + (sensor_dist * 0.01));
    double error = sensor_dist - theoretical_dist;

    if (std::abs(error) > 1.0) {
      continue;
    }

    // creates kalman gain matrix
    double uncertainty = (obs * cov * obs.transpose())(0, 0) + variance;
    Eigen::Matrix<double, 2, 1> kalman_gain =
        cov * obs.transpose() / uncertainty;

    // updates state based on gain and error
    state += kalman_gain * error;

    // prevents negative predictions
    state(0) = std::max(0.0, state(0));
    state(1) = std::max(0.0, state(1));

    cov = (Eigen::Matrix2d::Identity() - kalman_gain * obs) * cov;
  }
}

}  // namespace Kalman
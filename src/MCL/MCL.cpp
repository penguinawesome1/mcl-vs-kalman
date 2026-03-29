#include "MCL.hpp"

#include <cmath>
#include <iostream>
#include <numbers>
#include <random>
#include <vector>

namespace MCL {

/*
`w` - Map Width
`h` - Map Height
`xO` - Robot initial x
`yO` - Robot initial y
`minP` - Minimum number of particles allowed
`maxP` - Maximum number of particles allowed
`xyN` - XY Axis propogation noise
`t` - Resampling threshold
`s` - Sigma / Guassian Distribution sensitivity
*/
MCL::MCL(double w, double h, double xO, double yO, size_t minP, size_t maxP,
         double xyN, double t, double s)
    : mapWidth(w),
      mapHeight(h),
      odom{xO, yO, 0},
      minParticles(minP),
      maxParticles(maxP),
      maxXYNoise(xyN),
      threshold(t),
      sigma(s) {}  // Takes width and height of map
                   // and the default position

double MCL::Particle::getDistance(uint8_t mapWidth, uint8_t mapHeight, int s) {
  double angle = theta +
                 (s * (std::numbers::pi / 2));  // we need to represent the
                                                // sensors on each side
                                                // (represented by enums)

  double cos_val = std::cos(angle);
  double sin_val = std::sin(angle);

  const double epsilon = 1e-9;  // smallest representable value without getting
                                // outlier values (i.e, inf, -inf, NaN)

  double distanceX, distanceY;

  // need to ensure that no unpredicable values come through. if a ray is
  // completely horizontal, the cos func will give a nice good value, but sin
  // can give weird values, especially as it gets closer to +-pi

  if (std::abs(cos_val) <
      epsilon) {  // need to clamp distance to "predictable" value
    distanceX = std::numeric_limits<double>::infinity();
  } else if (cos_val > 0) {
    distanceX = (mapWidth - x) / cos_val;  // heading toward right wall
  } else {
    distanceX = -x / cos_val;  // heading toward left wall (x=0)
  }

  if (std::abs(sin_val) <
      epsilon) {  // need to clamp distance to "predictable" value
    distanceY = std::numeric_limits<double>::infinity();
  } else if (sin_val > 0) {
    distanceY = (mapHeight - y) / sin_val;  // heading toward top wall
  } else {
    distanceY = -y / sin_val;  // heading toward bottom wall (y=0)
  }

  return std::min(distanceX, distanceY);
}

void MCL::spawnParticles() {
  std::mt19937 mt(rd());
  std::uniform_real_distribution<> udw{0, mapWidth};
  std::uniform_real_distribution<> udh{0, mapHeight};

  for (size_t i = 0; i < maxParticles; i++) {
    Particle p{udw(mt), udh(mt), 0, 1};
    particles.push_back(p);
  }
}

void MCL::propogateParticles() {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::normal_distribution<> noise{0, maxXYNoise};
  for (Particle& p : particles) {
    p.x += odom.x - lastOdom.x + noise(mt);  // Odom IRL drifts, so we
    p.y += odom.y - lastOdom.y + noise(mt);  // focus on hypothesises here.
    p.x = std::clamp(p.x, 0.0, mapWidth);
    p.y = std::clamp(p.y, 0.0, mapHeight);
    p.theta = odom.theta;  // More or less absolute
  }
}

void MCL::assignWeight(MCL::Particle& p) {
  double diffLeft =
      distSensors.left - p.getDistance(mapWidth, mapHeight, Sensor::Left);
  double diffFront =
      distSensors.front - p.getDistance(mapWidth, mapHeight, Sensor::Front);
  double diffRight =
      distSensors.right - p.getDistance(mapWidth, mapHeight, Sensor::Right);
  double weightLeft = std::exp((-(diffLeft * diffLeft) / (2 * sigma * sigma)));
  double weightFront =
      std::exp((-(diffFront * diffFront) / (2 * sigma * sigma)));
  double weightRight =
      std::exp((-(diffRight * diffRight) / (2 * sigma * sigma)));

  p.weight = weightLeft * weightFront * weightRight;
}

void MCL::resampleParticles() {
  std::mt19937 mt(rd());
  std::normal_distribution<> noise{0, maxXYNoise};
  for (Particle& p : particles) {
    assignWeight(p);
  }

  std::vector<Particle> newParticles;
  // push weights to separate vector in order to keep index
  std::vector<double> weights;
  for (Particle p : particles) {
    weights.push_back(p.weight);
  }
  std::discrete_distribution<size_t> dd(
      weights.begin(), weights.end());  // create distribution by weights

  // push particles that have the higher probability (weight) of getting chosen
  // into new vector
  for (size_t i = 0; i < maxParticles; i++) {
    Particle p = particles[dd(mt)];
    newParticles.push_back(p);
  }

  particles = newParticles;
}

Pose MCL::getEstimation() {
  Particle bestParticle = particles[0];
  for (Particle p : particles) {
    if (p.weight > bestParticle.weight) {
      bestParticle = p;
    }
  }

  return Pose{bestParticle.x, bestParticle.y, bestParticle.theta};
}

}  // namespace MCL
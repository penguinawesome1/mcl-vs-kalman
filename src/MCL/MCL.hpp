#include <cstdint>
#include <numbers>
#include <random>
#include <stdexcept>
#include <vector>

#include "State.hpp"

namespace MCL {

// Barebones implementation of the Monte Carlo Localization algorithm
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
class MCL {
  enum Sensor {  // Define radian offsets for sensor data
    Left = 1,
    Front = 0,
    Right = -1
  };

  struct Particle {
    double x, y, theta, weight;
    double getDistance(uint8_t mapWidth, uint8_t mapHeight, int s);
  };

  double mapWidth, mapHeight;
  Pose odom;
  Pose lastOdom;
  SensorSet distSensors;

  // TUNING
  size_t minParticles;
  size_t maxParticles;
  double maxXYNoise;
  double threshold;  // 0-1 / Gaussian Distribution
  double sigma;      // sigma sigma :)

  std::vector<Particle> particles;
  std::random_device rd;

  void assignWeight(
      Particle& p);  // Assigns weight based off difference from actual reading

 public:
  MCL(double w, double h, double xO, double yO, size_t minP, size_t maxP,
      double xyN, double t, double s);  // Constructor
  ~MCL() = default;                     // Destructor

  void spawnParticles();  // Uniformly distributes particles throughout the map
  void propogateParticles();  // Updates all the particles according to measured
                              // movement delta and heading
  void resampleParticles();   // Assigns weights to particles and discards far
                              // off particles for optimizations

  void updateOdom(Pose p) {
    lastOdom = odom;
    odom = p;
  }
  void updateSensors(SensorSet s) { distSensors = s; }

  std::vector<Particle> getParticles() { return particles; }
  Pose getEstimation();
};

}  // namespace MCL
#pragma once

struct Pose {
  double x, y, theta;
};

struct SensorSet {
  double imu, front, left, right;
};

struct State {
  double time;
  Pose truth;
  Pose odom;
  SensorSet sensors;
};

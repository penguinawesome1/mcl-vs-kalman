#pragma once

struct State {
  double time;
  Pose truth;
  Pose odom;
  SensorSet sensors;
};

struct SensorSet {
  double imu, front, left, right;
};

struct Pose {
  double x, y, theta;
};
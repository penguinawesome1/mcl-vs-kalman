#pragma once

struct SensorSet {
  double front, left, right;
};

struct Pose {
  double x, y, theta;
};

struct State {
  double time;
  Pose truth;
  Pose odom;
  SensorSet sensors;
};
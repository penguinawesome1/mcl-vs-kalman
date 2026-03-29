# Robot Localization: Monte Carlo Localization vs. Kalman Filter

![Build Status](https://github.com/penguinawesome1/mcl-vs-kalman/actions/workflows/cmake-tests.yml/badge.svg)
![Build Status](https://github.com/penguinawesome1/mcl-vs-kalman/actions/workflows/deploy.yml/badge.svg)

## Build and Run Tests

```bash
emcmake cmake -G "Ninja" -S . -B build
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```

## Overview

This repo explores the differences between Monte Carlo Localization vs. Kalman Filter for addressing the problem of inaccurate robot localization.

Accurate localization is crucial for consistent autonomous robot navigation. As the use of robots continues to grow in everyday life, it is crucial that these machines can accurately navigate the world for productivity and safety. If a robot thinks that it’s 10 inches farther to the right than it actually is, it may make poor and even potentially dangerous decisions. 

## Requirements

C++ 23

All information on libraries can be found in the CMakeLists.txt

## Dataset

The main dataset is a csv file that can be found in /data  
The setup:  

time = the current time (increments by 0.1s each iteration)  
true_x = the current x coordinate of the robot (ft) (initialized at 6 ft, the middle)  
true_y = the current y coordinate of the robot (ft) (initialized at 6 ft, the middle)  
true_theta = the current angle (radians) that the robot is facing (initialized at 0 rad, right)  
odom_x = theoretical value of encoder sensor that tracks x position (true_x + random noise)  
odom_y = theoretical value of encoder sensor that tracks y position (true_y + random noise)  
imu_reading = theoretical value of IMU (Inertial Measurement Unit) that tracks the angle the robot is facing in radians (true_theta + random noise)  
front_sensor = distance from the front of the robot to the wall in front of it (ft) (with increasing noise depending on how far it is from the wall)  
left_sensor = distance from the left of the robot to the wall to the left of it (ft) (with increasing noise depending on how far it is from the wall)  
right_sensor = distance from the right of the robot to the wall to the right of it (ft) (with increasing noise depending on how far it is from the wall)  



## Credits

Robot favicon from [Flaticon](https://www.flaticon.com/free-icon/robot_6134346#).

# Robot Localization: Monte Carlo Localization vs. Kalman Filter

![Build Status](https://github.com/penguinawesome1/mcl-vs-kalman/actions/workflows/cmake-tests.yml/badge.svg)
![Build Status](https://github.com/penguinawesome1/mcl-vs-kalman/actions/workflows/deploy.yml/badge.svg)

## Build and Run Tests

```bash
emcmake cmake -G "Ninja" -S . -B build
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```

## Credits

Robot favicon from [Flaticon](https://www.flaticon.com/free-icon/robot_6134346#).
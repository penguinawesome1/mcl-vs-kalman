#include <emscripten/bind.h>

#include "Kalman/Runner.hpp"
#include "MCL/Runner.hpp"
#include "State.hpp"

using namespace emscripten;

EMSCRIPTEN_BINDINGS(pose_module) {
  value_object<Pose>("Pose")
      .field("x", &Pose::x)
      .field("y", &Pose::y)
      .field("theta", &Pose::theta);
}

EMSCRIPTEN_BINDINGS(mcl_module) {
  class_<MCL::Runner>("MCLRunner")
      .constructor<std::string>()
      .function("predictAndUpdate", &MCL::Runner::predict_and_update)
      .function("getTruth", &MCL::Runner::get_truth);
}

EMSCRIPTEN_BINDINGS(kalman_module) {
  class_<Kalman::Runner>("KalmanRunner")
      .constructor<std::string>()
      .function("predictAndUpdate", &Kalman::Runner::predict_and_update)
      .function("getTruth", &Kalman::Runner::get_truth);
}
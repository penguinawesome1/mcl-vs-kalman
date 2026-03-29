#include "Runner.hpp"

namespace MCL {

Runner::Runner(const fs::path& path)
    : reader_(path),
      curr_state_(reader_.bake_next_state()),
      mcl_(12, 12, 6, 6, 500, 500, 1, 0.1, 0.5) {}

auto Runner::predict_and_update() -> Pose {
  curr_state_ = reader_.bake_next_state();
  mcl_.updateOdom(curr_state_.odom);
  mcl_.updateSensors(curr_state_.sensors);
  mcl_.propogateParticles();
  mcl_.resampleParticles();

  return mcl_.getEstimation();
}

}  // namespace MCL

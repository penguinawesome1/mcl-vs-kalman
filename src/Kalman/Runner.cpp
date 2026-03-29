#include "Runner.hpp"

#include "Reader.hpp"

namespace Kalman {

Runner::Runner(const fs::path& path)
    : reader_(path),
      curr_state_(reader_.bake_next_state()),
      kalman_(curr_state_.truth.x, curr_state_.truth.y) {}

auto Runner::predict_and_update() -> Pose {
  State prior_state = curr_state_;
  curr_state_ = reader_.bake_next_state();

  kalman_.predict(prior_state.odom.x, prior_state.odom.y, curr_state_.odom.x,
                  curr_state_.odom.y, curr_state_.odom.theta);

  kalman_.update(curr_state_.odom.theta, curr_state_.sensors.front,
                 curr_state_.sensors.left, curr_state_.sensors.right);

  return kalman_.getEstimation();
}

}  // namespace Kalman

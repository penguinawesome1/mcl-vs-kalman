#pragma once

#include <filesystem>

#include "MCL.hpp"
#include "Reader.hpp"

namespace fs = std::filesystem;

namespace MCL {

class Runner {
 public:
  /**
   * Create an instance given a path to the data to init the reader.
   */
  Runner(const fs::path& path);

  /**
   * Predicts the next step which will be returned.
   * Then updates its state based on the next line of input data.
   */
  auto predict_and_update() -> Pose;

  auto get_truth() -> Pose { return curr_state_.truth; }

 private:
  Reader reader_;
  State curr_state_;
  MCL mcl_;
};

}  // namespace MCL
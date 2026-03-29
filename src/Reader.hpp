#pragma once

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "State.hpp"

namespace fs = std::filesystem;

class Reader {
 public:
  Reader(const fs::path& path);

  /**
   * Grab the state parsed from the next line in the data.
   * Automatically increment current line.
   */
  auto bake_next_state() -> State;

 private:
  std::ifstream in_;
};

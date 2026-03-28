#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "State.hpp"

namespace fs = std::filesystem;

class Reader {
 public:
  Reader(const fs::path& path) : in_(path) {
    if (!in_) {
      throw std::runtime_error("Failed to open path");
    }
    std::string header;
    std::getline(in_, header); // Throw away header text
    if (header == "") { throw std::runtime_error("No data"); }
  }

  auto bake_next_state() -> State {
    std::string line;
    std::getline(in_, line);
    std::stringstream ss(line);
    std::string item;

    auto next_num = [&]() {
      std::getline(ss, item, ',');
      return std::stod(item);
    };

    return State{.time = next_num(),
                 .truth =
                     Pose{
                         .x = next_num(),
                         .y = next_num(),
                         .theta = next_num(),
                     },
                 .odom =
                     Pose{
                         .x = next_num(),
                         .y = next_num(),
                         .theta = next_num(),
                     },
                 .sensors = SensorSet{
                     .front = next_num(),
                     .left = next_num(),
                     .right = next_num(),
                 }};
  }

 private:
  std::ifstream in_;
};

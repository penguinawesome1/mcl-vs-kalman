#include <emscripten/emscripten.h>

#include <iostream>
#include <print>

#include "Kalman.hpp"
#include "MCL.hpp"
#include "Reader.hpp"

extern "C" EMSCRIPTEN_KEEPALIVE void hello_world() {
  std::println("Hello from hello_world!");
  std::cout << std::flush;
}

int main() {
  Reader reader("data/robot_log.csv");
  Kalman::Filter kalman;
  MCL::Filter mcl;

  std::println("Hello from main!");
  std::cout << std::flush;
  return 0;
}
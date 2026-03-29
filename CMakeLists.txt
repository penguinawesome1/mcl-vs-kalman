cmake_minimum_required(VERSION 3.30)
project(mcl-vs-kalman)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(loc_logic 
    src/Kalman/Kalman.cpp 
    src/Kalman/Runner.cpp
    src/Reader.cpp
    src/bindings.cpp
)
target_include_directories(loc_logic PUBLIC src)

include(FetchContent)
FetchContent_Declare(
  Eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)
FetchContent_MakeAvailable(Eigen3)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

target_link_libraries(loc_logic PUBLIC Eigen3::Eigen)

enable_testing()

add_executable(
  kalman_test
  tests/kalman_test.cpp
)
target_link_libraries(
  kalman_test
  GTest::gtest_main
  loc_logic
)

include(GoogleTest)
gtest_discover_tests(kalman_test)

file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/data" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
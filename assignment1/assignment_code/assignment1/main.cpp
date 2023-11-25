#include <iostream>
#include <chrono>

#include "SplineViewerApp.hpp"

using namespace GLOO;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0]
              << " SPLINE_FILE where SPLINE_FILE is "
                 "relative to assets/assignment1"
              << std::endl;
    return -1;
  }
  std::unique_ptr<SplineViewerApp> app =
      make_unique<SplineViewerApp>("Assignment1", glm::ivec2(1440, 900),
                                   "assignment1/" + std::string(argv[1]));

  app->SetupScene();

  using Clock = std::chrono::high_resolution_clock;
  using TimePoint =
      std::chrono::time_point<Clock, std::chrono::duration<double>>;
  TimePoint last_tick_time = Clock::now();
  TimePoint start_tick_time = last_tick_time;
  while (!app->IsFinished()) {
    TimePoint current_tick_time = Clock::now();
    double delta_time = (current_tick_time - last_tick_time).count();
    double total_elapsed_time = (current_tick_time - start_tick_time).count();
    app->Tick(delta_time, total_elapsed_time);
    last_tick_time = current_tick_time;
  }
  return 0;
}
